#include "pid_controller.h"

#include <stdio.h>
#include "nvs_flash.h"
#include "nvs.h"

#define NVS_NAMESPACE "pid"
#define NVS_KEY_KP     "kp"
#define NVS_KEY_KI     "ki"
#define NVS_KEY_KD     "kd"
#define NVS_KEY_SET    "setpoint"

void pid_ctrl_init(pid_ctrl_t *pid, float kp, float ki, float kd, float dt,
                   float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;

    /* Symmetric integral bound: allow both positive and negative accumulation */
    pid->integral_min = -out_max;

    /* Derivative filter coefficient: 0.5 = moderate low-pass (reduces noise)
     * Set via pid_ctrl_init_ex() or tune the field after init */
    pid->derivative_filter = 0.5f;

    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_measurement = 0;
    pid->prev_derivative = 0;
}

float pid_ctrl_compute(pid_ctrl_t *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    /* ---- Proportional ---- */
    float p_term = pid->kp * error;

    /* ---- Integral with symmetric clamping ---- */
    pid->integral += error * pid->dt;
    if (pid->integral > pid->out_max) {
        pid->integral = pid->out_max;
    } else if (pid->integral < pid->integral_min) {
        pid->integral = pid->integral_min;
    }
    float i_term = pid->ki * pid->integral;

    /* ---- Derivative-on-measurement (reduces derivative kick) ----
     * d(error)/dt = d(setpoint - meas)/dt = -d(meas)/dt  (setpoint changes rarely)
     * Using -measurement diffs filters out setpoint-change transients.
     */
    float d_raw = -(measurement - pid->prev_measurement) / pid->dt;

    /* Low-pass filter on derivative term (suppresses sensor noise) */
    float alpha = pid->derivative_filter;
    float d_filtered = alpha * d_raw + (1.0f - alpha) * pid->prev_derivative;
    pid->prev_derivative = d_filtered;

    float d_term = pid->kd * d_filtered;

    /* ---- Sum ---- */
    float output = p_term + i_term + d_term;

    /* ---- Output clamping with back-calculation anti-windup ---- */
    if (output > pid->out_max) {
        output = pid->out_max;
        /* Back-calculation: prevent integral from pushing beyond saturation */
        if (pid->ki > 0 && pid->prev_error > 0) {
            pid->integral -= error * pid->dt;
        }
    } else if (output < pid->out_min) {
        output = pid->out_min;
        if (pid->ki > 0 && pid->prev_error < 0) {
            pid->integral -= error * pid->dt;
        }
    }

    /* Update state */
    pid->prev_error = error;
    pid->prev_measurement = measurement;

    return output;
}

void pid_ctrl_reset(pid_ctrl_t *pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_measurement = 0;
    pid->prev_derivative = 0;
}

void pid_ctrl_save_to_nvs(pid_ctrl_t *pid, float setpoint)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) return;
    nvs_set_i32(handle, NVS_KEY_KP, (int32_t)(pid->kp * 1000.0f));
    nvs_set_i32(handle, NVS_KEY_KI, (int32_t)(pid->ki * 1000.0f));
    nvs_set_i32(handle, NVS_KEY_KD, (int32_t)(pid->kd * 1000.0f));
    nvs_set_i32(handle, NVS_KEY_SET, (int32_t)(setpoint * 1000.0f));
    nvs_commit(handle);
    nvs_close(handle);
    printf("PID params saved to NVS\n");
}

int pid_ctrl_load_from_nvs(pid_ctrl_t *pid, float *setpoint)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return -1;

    int32_t val;
    if (nvs_get_i32(handle, NVS_KEY_KP, &val) != ESP_OK) { nvs_close(handle); return -1; }
    pid->kp = val / 1000.0f;

    if (nvs_get_i32(handle, NVS_KEY_KI, &val) != ESP_OK) { nvs_close(handle); return -1; }
    pid->ki = val / 1000.0f;

    /* Migrate: if saved Ki < 0.10, treat NVS as stale (old defaults) */
    if (pid->ki < 0.10f) {
        nvs_close(handle);
        printf("PID NVS stale, using new defaults\n");
        return -1;
    }

    if (nvs_get_i32(handle, NVS_KEY_KD, &val) != ESP_OK) { nvs_close(handle); return -1; }
    pid->kd = val / 1000.0f;

    if (nvs_get_i32(handle, NVS_KEY_SET, &val) != ESP_OK) { nvs_close(handle); return -1; }
    *setpoint = val / 1000.0f;

    nvs_close(handle);
    printf("PID params loaded from NVS\n");
    return 0;
}
