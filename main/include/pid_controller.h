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
    float integral_min;       /* symmetric integral bound (usually -out_max) */
    float derivative_filter;  /* low-pass filter coefficient for D-term (0-1, 0 = raw) */
    float prev_measurement;   /* for derivative-on-measurement */
    float prev_derivative;    /* for D-term low-pass filter */
} pid_ctrl_t;

void pid_ctrl_init(pid_ctrl_t *pid, float kp, float ki, float kd, float dt,
                   float out_min, float out_max);
float pid_ctrl_compute(pid_ctrl_t *pid, float setpoint, float measurement);
void pid_ctrl_reset(pid_ctrl_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
