/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "components_example.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

int example_compute_sum(int a, int b)
{
    return a + b;
}

int example_fibonacci(int n)
{
    if (n <= 0) {
        return 0;
    }
    if (n == 1) {
        return 1;
    }
    return example_fibonacci(n - 1) + example_fibonacci(n - 2);
}

int example_factorial(int n)
{
    if (n <= 0) {
        return 1;
    }
    return n * example_factorial(n - 1);
}

void example_configure_led(gpio_num_t gpio)
{
    gpio_reset_pin(gpio);
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
}

void example_blink_led(gpio_num_t gpio)
{
    gpio_set_level(gpio, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(gpio, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
}
