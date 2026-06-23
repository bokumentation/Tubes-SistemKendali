/*
 * obstacle_avoidance.c — obstacle avoidance with PID speed control + steering
 *
 * Key improvements over original:
 *   1. Dual drive modes: CRUISE (Tesla AEB-style) + MAINTAIN (PID wall-follow)
 *   2. Side sensors assist main sensor via get_min_front() — speed adjusts, no rotation.
 *   3. Smooth transition zone — blends PID output into MAX_SPEED gradually.
 *   4. Retuned PID: Kp=2.5, Ki=0.05, Kd=0.3, DT=0.05 (20 Hz loop).
 *   5. EMA_ALPHA=0.5 — less filter lag, still smooths jitter.
 */

#include "obstacle_avoidance.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

#include "pin_definition.h"
#include "ultrasonic.h"
#include "motor_control.h"
#include "pid_controller.h"
#include "shared_data.h"

/* ================================================================ */
/* Tunable Constants                                                */
/* ================================================================ */
#define MAX_SPEED           100
#define PID_DT              0.05f         /* 20 Hz control loop */
#define BACK_THRESHOLD      15.0f          /* min distance behind before blocking reverse */
#define OVERRIDE_TIMEOUT    40             /* ~2 s @ 20 Hz before manual override expires */
#define EMA_ALPHA           0.8f           /* exponential moving average (0 = no update, 1 = raw) */

/* Smooth‑transition zone (40 cm → 60 cm) */
#define TRANSITION_LOW      40.0f          /* below this ≡ full PID authority */
#define TRANSITION_HIGH     60.0f          /* above this ≡ full MAX_SPEED */

/* CRUISE mode distance-proportional braking */
#define CRUISE_FULL_SPEED    80.0f         /* distance above which → full speed */
#define CRUISE_STOP_DIST     15.0f         /* distance below which → stop */

/* Emergency reverse */
#define EMERGENCY_THRESHOLD 10.0f          /* below this → reverse (if back is clear) */
#define REVERSE_SPEED       -50

/* Feed‑forward brake (when closing speed > BRAKE_VEL_THRESH while < BRAKE_RANGE) */
#define BRAKE_RANGE         60.0f
#define BRAKE_VEL_THRESH    10.0f          /* cm/s */
#define BRAKE_GAIN          0.6f

/* ================================================================ */
/* Globals (shared with web dashboard via shared_data.h)             */
/* ================================================================ */
static ultrasonic_sensor_t front_sensor;
static ultrasonic_sensor_t front_left_sensor;
static ultrasonic_sensor_t front_right_sensor;
static ultrasonic_sensor_t back_sensor;

volatile float sensor_front_cm        = 0;
volatile float sensor_front_left_cm   = 0;
volatile float sensor_front_right_cm  = 0;
volatile float sensor_back_cm         = 0;

volatile int motor_fr_speed = 0;
volatile int motor_fl_speed = 0;
volatile int motor_br_speed = 0;
volatile int motor_bl_speed = 0;

pid_ctrl_t speed_pid;

volatile int manual_override = 0;
volatile int override_count  = 0;
volatile int auto_mode       = 0;

volatile float speed_setpoint = 40.0f;

volatile int drive_mode      = DRIVE_MODE_CRUISE;

volatile float pid_output_display = 0;

float graph_error[GRAPH_SIZE];
float graph_output[GRAPH_SIZE];
float graph_setpoint[GRAPH_SIZE];
int   graph_index = 0;
int   graph_sample_count = 0;

/* ---- internal state ---- */
static float prev_min_front = 999.0f;
static bool  pid_was_active = false;   /* tracks whether PID was in control last iteration */

/* ================================================================ */
/* Sensor helpers                                                    */
/* ================================================================ */

static void read_all_sensors(void)
{
    float val;
    val = ultrasonic_get_cm(&front_sensor);
    if (val >= 0)
        sensor_front_cm = sensor_front_cm * (1.0f - EMA_ALPHA) + val * EMA_ALPHA;

    val = ultrasonic_get_cm(&front_left_sensor);
    if (val >= 0)
        sensor_front_left_cm = sensor_front_left_cm * (1.0f - EMA_ALPHA) + val * EMA_ALPHA;

    val = ultrasonic_get_cm(&front_right_sensor);
    if (val >= 0)
        sensor_front_right_cm = sensor_front_right_cm * (1.0f - EMA_ALPHA) + val * EMA_ALPHA;

    val = ultrasonic_get_cm(&back_sensor);
    if (val >= 0)
        sensor_back_cm = sensor_back_cm * (1.0f - EMA_ALPHA) + val * EMA_ALPHA;

    /* Fire all sensors for the next loop */
    ultrasonic_trigger(&front_sensor);
    ultrasonic_trigger(&front_left_sensor);
    ultrasonic_trigger(&front_right_sensor);
    ultrasonic_trigger(&back_sensor);
}

static float get_min_front(void)
{
    float min_val = 999.0f;
    if (sensor_front_cm       >= 0 && sensor_front_cm       < min_val) min_val = sensor_front_cm;
    if (sensor_front_left_cm  >= 0 && sensor_front_left_cm  < min_val) min_val = sensor_front_left_cm;
    if (sensor_front_right_cm >= 0 && sensor_front_right_cm < min_val) min_val = sensor_front_right_cm;
    return (min_val > 998.0f) ? -1 : min_val;
}

/* ================================================================ */
/* Smooth transition: interpolates between PID output and MAX_SPEED  */
/* ================================================================ */
static float smooth_transition(float min_front, float pid_output)
{
    if (min_front >= TRANSITION_HIGH)
        return (float)MAX_SPEED;

    if (min_front <= TRANSITION_LOW)
        return pid_output;

    /* Linear blend between TRANSITION_LOW and TRANSITION_HIGH */
    float t = (min_front - TRANSITION_LOW) / (TRANSITION_HIGH - TRANSITION_LOW);
    return pid_output * (1.0f - t) + (float)MAX_SPEED * t;
}

/* ================================================================ */
/* Main avoidance task                                               */
/* ================================================================ */
static void avoid_task(void *arg)
{
    esp_task_wdt_add(NULL);

    /* Tuned PID: Kp=2.5  Ki=0.05  Kd=0.3  dt=0.05  out[0, 100] */
    pid_ctrl_init(&speed_pid, 2.5f, 0.15f, 0.3f, PID_DT, -50, MAX_SPEED);
    speed_pid.derivative_filter = 0.4f;   /* moderate D‑filtering */

    /* Restore PID params from NVS if available, otherwise save defaults */
    if (pid_ctrl_load_from_nvs(&speed_pid, (float *)&speed_setpoint) != 0) {
        pid_ctrl_save_to_nvs(&speed_pid, speed_setpoint);
    }

    while (1) {
        read_all_sensors();

        /* ---- manual override timeout ---- */
        if (manual_override) {
            override_count++;
            if (override_count > OVERRIDE_TIMEOUT) {
                manual_override = 0;
                override_count  = 0;
            }
        }

        /* ==================================================== */
        /* AUTO mode                                             */
        /* ==================================================== */
        if (auto_mode && !manual_override) {
            float speed_out  = 0;
            float steer_bias = 0;
            float min_front  = get_min_front();

            if (min_front >= 0) {

                if (min_front < EMERGENCY_THRESHOLD) {
                    /* ---------------------------------------------------- */
                    /* Emergency: obstacle < 10 cm                            */
                    /* ---------------------------------------------------- */
                    if (sensor_back_cm >= BACK_THRESHOLD || sensor_back_cm < 0) {
                        speed_out = REVERSE_SPEED;
                    } else {
                        speed_out = 0;   /* back blocked → freeze */
                    }
                    pid_ctrl_reset(&speed_pid);
                    pid_was_active = false;
                    steer_bias = 0;

                } else if (drive_mode == DRIVE_MODE_CRUISE) {
                    /* ========================================== */
                    /* CRUISE mode: distance-proportional braking  */
                    /* ========================================== */
                    if (min_front >= CRUISE_FULL_SPEED) {
                        speed_out = MAX_SPEED;
                    } else if (min_front <= CRUISE_STOP_DIST) {
                        speed_out = 0;
                    } else {
                        float t = (min_front - CRUISE_STOP_DIST) / (CRUISE_FULL_SPEED - CRUISE_STOP_DIST);
                        speed_out = (float)MAX_SPEED * t;
                    }
                    pid_was_active = false;
                    steer_bias = 0;

                } else {
                    /* ========================================== */
                    /* MAINTAIN mode: PID wall-follow              */
                    /* ========================================== */
                    if (!pid_was_active) {
                        pid_ctrl_reset(&speed_pid);
                    }

                    float pid_out = pid_ctrl_compute(&speed_pid, min_front, speed_setpoint);
                    speed_out = smooth_transition(min_front, pid_out);

                    /* Feed-forward braking (only when moving forward) */
                    if (speed_out > 0) {
                        float velocity = (prev_min_front - min_front) / PID_DT;
                        if (min_front < BRAKE_RANGE && velocity > BRAKE_VEL_THRESH) {
                            speed_out -= velocity * BRAKE_GAIN;
                            if (speed_out < 0) speed_out = 0;
                        }
                    }

                    pid_was_active = true;
                }

                prev_min_front = min_front;
            }

            /* All wheels same speed — no rotation */
            int32_t fr = (int32_t)speed_out;
            int32_t fl = (int32_t)speed_out;
            int32_t br = (int32_t)speed_out;
            int32_t bl = (int32_t)speed_out;

            /* Clamp */
            #define CLAMP(v, lo, hi) do { if ((v) < (lo)) (v) = (lo); if ((v) > (hi)) (v) = (hi); } while(0)
            CLAMP(fr, -MAX_SPEED, MAX_SPEED);
            CLAMP(fl, -MAX_SPEED, MAX_SPEED);
            CLAMP(br, -MAX_SPEED, MAX_SPEED);
            CLAMP(bl, -MAX_SPEED, MAX_SPEED);
            #undef CLAMP

            motor_set_speed(fr, fl, br, bl);
            motor_fr_speed = fr;
            motor_fl_speed = fl;
            motor_br_speed = br;
            motor_bl_speed = bl;
            pid_output_display = speed_out;

            /* Record graph data */
            graph_error[graph_index]   = min_front - speed_setpoint;
            graph_output[graph_index]  = speed_out;
            graph_setpoint[graph_index] = speed_setpoint;
            graph_index = (graph_index + 1) % GRAPH_SIZE;
            if (graph_sample_count < GRAPH_SIZE) graph_sample_count++;
        }

        /* ==================================================== */
        /* Manual back-safety (always active)                    */
        /* ==================================================== */
        if (manual_override) {
            if (sensor_back_cm > 0 && sensor_back_cm < BACK_THRESHOLD &&
                (motor_fr_speed < 0 || motor_fl_speed < 0 ||
                 motor_br_speed < 0 || motor_bl_speed < 0)) {
                motor_all_stop();
                motor_fr_speed = 0;
                motor_fl_speed = 0;
                motor_br_speed = 0;
                motor_bl_speed = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS((int)(PID_DT * 1000)));
        esp_task_wdt_reset();
    }
}

/* ================================================================ */
/* Public: start avoidance subsystem                                 */
/* ================================================================ */
void obstacle_avoidance_start(void)
{
    front_sensor.trig_pin       = SENSOR_FRONT_TRIG;
    front_sensor.echo_pin       = SENSOR_FRONT_ECHO;
    front_left_sensor.trig_pin  = SENSOR_FRONT_LEFT_TRIG;
    front_left_sensor.echo_pin  = SENSOR_FRONT_LEFT_ECHO;
    front_right_sensor.trig_pin = SENSOR_FRONT_RIGHT_TRIG;
    front_right_sensor.echo_pin = SENSOR_FRONT_RIGHT_ECHO;
    back_sensor.trig_pin        = SENSOR_BACK_TRIG;
    back_sensor.echo_pin        = SENSOR_BACK_ECHO;

    ultrasonic_init(&front_sensor);
    ultrasonic_init(&front_left_sensor);
    ultrasonic_init(&front_right_sensor);
    ultrasonic_init(&back_sensor);

    xTaskCreate(avoid_task, "avoid_task", 4096, NULL, 5, NULL);
}