#include "ultrasonic.h"

#include <stdio.h>

#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#define TAG "ultrasonic"

static bool IRAM_ATTR rmt_rx_done_callback(rmt_channel_handle_t channel,
                                           const rmt_rx_done_event_data_t *edata,
                                           void *user_data)
{
    ultrasonic_sensor_t *sensor = (ultrasonic_sensor_t *)user_data;
    rmt_symbol_word_t *symbols = (rmt_symbol_word_t *)edata->received_symbols;
    size_t num_symbols = edata->num_symbols;
    (void)channel;

    if (num_symbols == 0) {
        sensor->pulse_us = 0;
        sensor->new_data = true;
        return false;
    }

    uint32_t total_duration_us = 0;
    for (size_t i = 0; i < num_symbols && i < ULTRASONIC_RMT_SYMBOLS; i++) {
        if (symbols[i].duration0 > 0 && symbols[i].duration0 < ULTRASONIC_RMT_TIMEOUT_US) {
            total_duration_us += symbols[i].duration0;
        }
    }

    sensor->pulse_us = total_duration_us;
    sensor->new_data = true;

    return false;
}

void ultrasonic_init(ultrasonic_sensor_t *sensor)
{
    gpio_set_direction(sensor->trig_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(sensor->trig_pin, 0);

    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num = sensor->echo_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = ULTRASONIC_RMT_RESOLUTION_HZ,
        .mem_block_symbols = ULTRASONIC_RMT_SYMBOLS,
        .flags = {
            .invert_in = false,
            .with_dma = false,
        },
    };
    if (rmt_new_rx_channel(&rx_cfg, &sensor->rx_channel) != ESP_OK) {
        printf("ultrasonic: rmt_new_rx_channel failed for GPIO%d\n", sensor->echo_pin);
        return;
    }

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    rmt_rx_register_event_callbacks(sensor->rx_channel, &cbs, sensor);

    rmt_enable(sensor->rx_channel);

    sensor->pulse_us = 0;
    sensor->new_data = false;
}

void ultrasonic_trigger(ultrasonic_sensor_t *sensor)
{
    if (!sensor->rx_channel) return;

    rmt_disable(sensor->rx_channel);
    rmt_enable(sensor->rx_channel);

    rmt_receive_config_t rx_cfg = {
        .signal_range_min_ns = 0,
        .signal_range_max_ns = 30000000,
    };
    rmt_receive(sensor->rx_channel, sensor->rx_buf,
                sizeof(sensor->rx_buf), &rx_cfg);

    gpio_set_level(sensor->trig_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(sensor->trig_pin, 0);

    sensor->new_data = false;
    sensor->trigger_time = esp_timer_get_time();
}

float ultrasonic_get_cm(ultrasonic_sensor_t *sensor)
{
    if (!sensor->rx_channel) return -1;

    if (!sensor->new_data) {
        int64_t elapsed = esp_timer_get_time() - sensor->trigger_time;
        if (elapsed > (ULTRASONIC_RMT_TIMEOUT_US + 5000)) {
            sensor->pulse_us = 0;
            sensor->new_data = true;
        }
    }

    if (!sensor->new_data) return -1;
    if (sensor->pulse_us == 0) return -1;
    return (float)sensor->pulse_us / 58.0f;
}

bool ultrasonic_has_new_data(ultrasonic_sensor_t *sensor)
{
    return sensor->new_data;
}