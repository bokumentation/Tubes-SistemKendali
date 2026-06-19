#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/mcpwm.h"

#include "pin_definition.h"

static void in2_pin_initialize(void)
{
    gpio_set_direction(MOTOR_FR_AIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_FL_BIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_BR_AIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_BL_BIN2, GPIO_MODE_OUTPUT);

    gpio_set_level(MOTOR_FR_AIN2, 0);
    gpio_set_level(MOTOR_FL_BIN2, 0);
    gpio_set_level(MOTOR_BR_AIN2, 0);
    gpio_set_level(MOTOR_BL_BIN2, 0);
}

static void stby_pin_initialize(void)
{
    gpio_set_direction(DRV8833_1_STBY, GPIO_MODE_OUTPUT);
    gpio_set_direction(DRV8833_2_STBY, GPIO_MODE_OUTPUT);

    gpio_set_level(DRV8833_1_STBY, 1);
    gpio_set_level(DRV8833_2_STBY, 1);
}

static void mcpwm_gpio_initialize(void)
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_FR_AIN1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_FL_BIN1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MOTOR_BR_AIN1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, MOTOR_BL_BIN1);
}

static void motor_forward(mcpwm_timer_t timer, uint32_t speed_percent)
{
    uint32_t duty = speed_percent * DUTY_CYCLE_MAX / 100;

    mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_GEN_A, duty);
    mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_GEN_B, duty);
}

static void motor_stop(mcpwm_timer_t timer)
{
    mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_GEN_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_GEN_B, 0);
}

static void all_wheels_forward(uint32_t speed_percent)
{
    motor_forward(MCPWM_TIMER_0, speed_percent);
    motor_forward(MCPWM_TIMER_1, speed_percent);
}

static void all_wheels_stop(void)
{
    motor_stop(MCPWM_TIMER_0);
    motor_stop(MCPWM_TIMER_1);
}

static void motor_forward_task(void *arg)
{
    stby_pin_initialize();
    in2_pin_initialize();
    mcpwm_gpio_initialize();

    mcpwm_config_t pwm_config = {
        .frequency = 1000,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);

    while (1) {
        printf("All wheels forward at 70%%\n");
        all_wheels_forward(70);
        vTaskDelay(pdMS_TO_TICKS(5000));

        printf("All wheels stop\n");
        all_wheels_stop();
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void start_motor_forward_example(void)
{
    printf("Motor forward example starting...\n");
    xTaskCreate(motor_forward_task, "motor_forward_task", 4096, NULL, 5, NULL);
}
