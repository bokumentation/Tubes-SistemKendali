#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float dt;
    float out_min;
    float out_max;
} pid_ctrl_t;

void pid_ctrl_init(pid_ctrl_t *pid, float kp, float ki, float kd, float dt,
                   float out_min, float out_max);
float pid_ctrl_compute(pid_ctrl_t *pid, float setpoint, float measurement);
void pid_ctrl_reset(pid_ctrl_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
