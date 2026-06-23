#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include "pid_controller.h"

#define GRAPH_SIZE 60

#define DRIVE_MODE_CRUISE    0
#define DRIVE_MODE_MAINTAIN  1


extern volatile float sensor_front_cm;
extern volatile float sensor_front_left_cm;
extern volatile float sensor_front_right_cm;
extern volatile float sensor_back_cm;

extern volatile int motor_fr_speed;
extern volatile int motor_fl_speed;
extern volatile int motor_br_speed;
extern volatile int motor_bl_speed;

extern pid_ctrl_t speed_pid;

extern volatile int manual_override;
extern volatile int override_count;
extern volatile int auto_mode;
extern volatile float speed_setpoint;
extern volatile int drive_mode;
extern volatile float pid_output_display;

extern float graph_error[GRAPH_SIZE];
extern float graph_output[GRAPH_SIZE];
extern float graph_setpoint[GRAPH_SIZE];
extern int graph_index;
extern int graph_sample_count;

#endif /* SHARED_DATA_H */
