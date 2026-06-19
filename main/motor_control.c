#include "motor_control.h"
#include "pin_definition.h"

#include "driver/mcpwm.h"

void motor_control_init(void)
{
    /*
     * FR: UNIT_0 TIMER_0 GEN_A=GPIO14(forward) GEN_B=GPIO27(reverse)
     * FL: UNIT_0 TIMER_1 GEN_A=GPIO26(forward) GEN_B=GPIO32(reverse)
     * BR: UNIT_0 TIMER_2 GEN_A=GPIO19(forward) GEN_B=GPIO18(reverse)
     * BL: UNIT_1 TIMER_0 GEN_A=GPIO15(forward) GEN_B=GPIO33(reverse)
     */

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_FR_AIN1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_FR_AIN2);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MOTOR_FL_BIN1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, MOTOR_FL_BIN2);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, MOTOR_BR_AIN1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, MOTOR_BR_AIN2);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, MOTOR_BL_BIN1);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, MOTOR_BL_BIN2);

    mcpwm_config_t pwm_config = {
        .frequency = 1000,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);
}

void motor_set_speed(int32_t fr_speed, int32_t fl_speed,
                     int32_t br_speed, int32_t bl_speed)
{
    if (fr_speed > 0) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, fr_speed);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, 0);
    } else if (fr_speed < 0) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, -fr_speed);
    } else {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, 0);
    }

    if (fl_speed > 0) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, fl_speed);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, 0);
    } else if (fl_speed < 0) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, -fl_speed);
    } else {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, 0);
    }

    if (br_speed > 0) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, br_speed);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_B, 0);
    } else if (br_speed < 0) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_B, -br_speed);
    } else {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_B, 0);
    }

    if (bl_speed > 0) {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_A, bl_speed);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_B, 0);
    } else if (bl_speed < 0) {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_B, -bl_speed);
    } else {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_B, 0);
    }
}

void motor_all_stop(void)
{
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_B, 0);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_B, 0);
}
