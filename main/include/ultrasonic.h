#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include <stdbool.h>

#include "driver/rmt_rx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ULTRASONIC_RMT_RESOLUTION_HZ 1000000
#define ULTRASONIC_RMT_TIMEOUT_US    40000
#define ULTRASONIC_RMT_SYMBOLS       64

typedef struct {
    uint8_t trig_pin;
    uint8_t echo_pin;
    rmt_channel_handle_t rx_channel;
    rmt_symbol_word_t rx_buf[ULTRASONIC_RMT_SYMBOLS];
    volatile uint32_t pulse_us;
    volatile bool new_data;
    volatile int64_t trigger_time;
} ultrasonic_sensor_t;

void ultrasonic_init(ultrasonic_sensor_t *sensor);
void ultrasonic_trigger(ultrasonic_sensor_t *sensor);
float ultrasonic_get_cm(ultrasonic_sensor_t *sensor);
bool ultrasonic_has_new_data(ultrasonic_sensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif /* ULTRASONIC_H */