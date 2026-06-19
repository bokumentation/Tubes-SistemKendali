#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t trig_pin;
    uint8_t echo_pin;
} ultrasonic_sensor_t;

void ultrasonic_init(ultrasonic_sensor_t *sensor);
float ultrasonic_measure_cm(ultrasonic_sensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif /* ULTRASONIC_H */
