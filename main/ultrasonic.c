#include "ultrasonic.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void ultrasonic_init(ultrasonic_sensor_t *sensor)
{
    gpio_set_direction(sensor->trig_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(sensor->echo_pin, GPIO_MODE_INPUT);

    gpio_set_level(sensor->trig_pin, 0);
}

float ultrasonic_measure_cm(ultrasonic_sensor_t *sensor)
{
    gpio_set_level(sensor->trig_pin, 0);
    esp_rom_delay_us(2);
    gpio_set_level(sensor->trig_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(sensor->trig_pin, 0);

    int64_t start = esp_timer_get_time();
    while (gpio_get_level(sensor->echo_pin) == 0) {
        if (esp_timer_get_time() - start > 30000) {
            return -1;
        }
    }

    int64_t pulse_start = esp_timer_get_time();
    while (gpio_get_level(sensor->echo_pin) == 1) {
        if (esp_timer_get_time() - pulse_start > 30000) {
            return -1;
        }
    }

    int64_t pulse_end = esp_timer_get_time();
    float duration = (float)(pulse_end - pulse_start);

    return duration / 58.0f;
}
