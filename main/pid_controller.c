#include "pid_controller.h"

void pid_ctrl_init(pid_ctrl_t *pid, float kp, float ki, float kd, float dt,
                   float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral = 0;
    pid->prev_error = 0;
}

float pid_ctrl_compute(pid_ctrl_t *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    pid->integral += error * pid->dt;
    if (pid->integral > pid->out_max) {
        pid->integral = pid->out_max;
    } else if (pid->integral < pid->out_min) {
        pid->integral = pid->out_min;
    }

    float derivative = (error - pid->prev_error) / pid->dt;

    float output = (pid->kp * error)
                 + (pid->ki * pid->integral)
                 + (pid->kd * derivative);

    if (output > pid->out_max) {
        output = pid->out_max;
    } else if (output < pid->out_min) {
        output = pid->out_min;
    }

    pid->prev_error = error;

    return output;
}

void pid_ctrl_reset(pid_ctrl_t *pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
}
