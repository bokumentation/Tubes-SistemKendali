#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

void motor_control_init(void);
void motor_set_speed(int32_t fr_speed, int32_t fl_speed,
                     int32_t br_speed, int32_t bl_speed);
void motor_all_stop(void);

#endif /* MOTOR_CONTROL_H */