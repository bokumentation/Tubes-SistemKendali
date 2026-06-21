#include "motor_control.h"
#include "pin_definition.h"

#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "esp_check.h"

#define TAG "motor"

#define PWM_FREQ_HZ      1000
#define TIMER_RESOLUTION 1000000
#define PWM_PERIOD_TICKS (TIMER_RESOLUTION / PWM_FREQ_HZ)

#define SPEED_TO_TICKS(s) ((uint32_t)((s) * 10))

/* Motor dead zone: DRV8833 + N20 motors need at least this PWM% to overcome static friction */
#define MOTOR_MIN_SPEED 35

/* Each motor has its own operator, 2 comparators, 2 generators */
typedef struct {
    mcpwm_oper_handle_t  oper;
    mcpwm_cmpr_handle_t  cmp_fwd;
    mcpwm_cmpr_handle_t  cmp_rev;
    mcpwm_gen_handle_t   gen_fwd;
    mcpwm_gen_handle_t   gen_rev;
} motor_channel_t;

static motor_channel_t motor_fr;
static motor_channel_t motor_fl;
static motor_channel_t motor_br;
static motor_channel_t motor_bl;

static mcpwm_timer_handle_t timer_g0;  /* shared by FR, FL, BR */
static mcpwm_timer_handle_t timer_g1;  /* used by BL */

static esp_err_t create_shared_timer(int group_id, mcpwm_timer_handle_t *out_timer)
{
    mcpwm_timer_config_t timer_cfg = {
        .group_id       = group_id,
        .clk_src        = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz  = TIMER_RESOLUTION,
        .count_mode     = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks   = PWM_PERIOD_TICKS,
    };
    esp_err_t ret = mcpwm_new_timer(&timer_cfg, out_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_timer g%d failed: %d", group_id, ret);
        return ret;
    }

    ret = mcpwm_timer_enable(*out_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_timer_enable g%d failed: %d", group_id, ret);
        return ret;
    }
    ret = mcpwm_timer_start_stop(*out_timer, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_timer_start_stop g%d failed: %d", group_id, ret);
        return ret;
    }
    return ESP_OK;
}

static esp_err_t motor_channel_init(motor_channel_t *ch,
                                    int group_id,
                                    mcpwm_timer_handle_t timer,
                                    int gpio_fwd,
                                    int gpio_rev)
{
    esp_err_t ret;

    mcpwm_operator_config_t oper_cfg = {
        .group_id = group_id,
    };
    ret = mcpwm_new_operator(&oper_cfg, &ch->oper);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_operator failed: %d", ret);
        return ret;
    }

    ret = mcpwm_operator_connect_timer(ch->oper, timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_operator_connect_timer failed: %d", ret);
        return ret;
    }

    mcpwm_comparator_config_t cmp_cfg = {
        .flags.update_cmp_on_tez = 1,
    };
    ret = mcpwm_new_comparator(ch->oper, &cmp_cfg, &ch->cmp_fwd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_comparator fwd failed: %d", ret);
        return ret;
    }
    mcpwm_comparator_set_compare_value(ch->cmp_fwd, 0);

    ret = mcpwm_new_comparator(ch->oper, &cmp_cfg, &ch->cmp_rev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_comparator rev failed: %d", ret);
        return ret;
    }
    mcpwm_comparator_set_compare_value(ch->cmp_rev, 0);

    mcpwm_generator_config_t gen_cfg_fwd = {
        .gen_gpio_num = gpio_fwd,
    };
    ret = mcpwm_new_generator(ch->oper, &gen_cfg_fwd, &ch->gen_fwd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_generator fwd failed: %d", ret);
        return ret;
    }

    mcpwm_generator_config_t gen_cfg_rev = {
        .gen_gpio_num = gpio_rev,
    };
    ret = mcpwm_new_generator(ch->oper, &gen_cfg_rev, &ch->gen_rev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_generator rev failed: %d", ret);
        return ret;
    }

    ret = mcpwm_generator_set_action_on_timer_event(ch->gen_fwd,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                         MCPWM_TIMER_EVENT_EMPTY,
                                         MCPWM_GEN_ACTION_HIGH));
    if (ret != ESP_OK) return ret;

    ret = mcpwm_generator_set_action_on_compare_event(ch->gen_fwd,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                           ch->cmp_fwd,
                                           MCPWM_GEN_ACTION_LOW));
    if (ret != ESP_OK) return ret;

    ret = mcpwm_generator_set_action_on_timer_event(ch->gen_rev,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                         MCPWM_TIMER_EVENT_EMPTY,
                                         MCPWM_GEN_ACTION_HIGH));
    if (ret != ESP_OK) return ret;

    ret = mcpwm_generator_set_action_on_compare_event(ch->gen_rev,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                           ch->cmp_rev,
                                           MCPWM_GEN_ACTION_LOW));
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

void motor_control_init(void)
{
    static bool initialized = false;
    if (initialized) return;
    initialized = true;

    ESP_LOGI(TAG, "Initialising MCPWM motors (ESP-IDF v6)...");

    ESP_ERROR_CHECK(create_shared_timer(0, &timer_g0));
    ESP_ERROR_CHECK(create_shared_timer(1, &timer_g1));

    ESP_ERROR_CHECK(motor_channel_init(&motor_fr, 0, timer_g0,
                                       MOTOR_FR_AIN1, MOTOR_FR_AIN2));
    ESP_ERROR_CHECK(motor_channel_init(&motor_fl, 0, timer_g0,
                                       MOTOR_FL_BIN1, MOTOR_FL_BIN2));
    ESP_ERROR_CHECK(motor_channel_init(&motor_br, 0, timer_g0,
                                       MOTOR_BR_AIN1, MOTOR_BR_AIN2));
    ESP_ERROR_CHECK(motor_channel_init(&motor_bl, 1, timer_g1,
                                       MOTOR_BL_BIN1, MOTOR_BL_BIN2));

    ESP_LOGI(TAG, "All motors initialised (1000 Hz PWM, 2 timers)");
}

static void motor_set_single(motor_channel_t *ch, int32_t speed)
{
    uint32_t ticks;

    if (speed < -100) speed = -100;
    if (speed >  100) speed =  100;

    /* Dead-zone remapping: logical 1-100 → physical 35-100 */
    if (speed > 0) {
        speed = MOTOR_MIN_SPEED + (speed * (100 - MOTOR_MIN_SPEED)) / 100;
        ticks = SPEED_TO_TICKS(speed);
        mcpwm_comparator_set_compare_value(ch->cmp_fwd, ticks);
        mcpwm_comparator_set_compare_value(ch->cmp_rev, 0);
    } else if (speed < 0) {
        speed = -MOTOR_MIN_SPEED + (speed * (100 - MOTOR_MIN_SPEED)) / 100;
        ticks = SPEED_TO_TICKS(-speed);
        mcpwm_comparator_set_compare_value(ch->cmp_fwd, 0);
        mcpwm_comparator_set_compare_value(ch->cmp_rev, ticks);
    } else {
        mcpwm_comparator_set_compare_value(ch->cmp_fwd, 0);
        mcpwm_comparator_set_compare_value(ch->cmp_rev, 0);
    }
}

void motor_set_speed(int32_t fr_speed, int32_t fl_speed,
                     int32_t br_speed, int32_t bl_speed)
{
    motor_set_single(&motor_fr, fr_speed);
    motor_set_single(&motor_fl, fl_speed);
    motor_set_single(&motor_br, br_speed);
    motor_set_single(&motor_bl, bl_speed);
}

void motor_all_stop(void)
{
    motor_set_single(&motor_fr, 0);
    motor_set_single(&motor_fl, 0);
    motor_set_single(&motor_br, 0);
    motor_set_single(&motor_bl, 0);
}