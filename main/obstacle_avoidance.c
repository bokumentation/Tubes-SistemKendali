/*
 * obstacle_avoidance.c — obstacle avoidance with PID speed control + steering
 *
 * Key improvements over original:
 *   1. Differential steering — compares left/right sensors and turns toward
 *      the clearer path instead of just stopping.
 *   2. Smooth transition zone — blends PID output into MAX_SPEED gradually
 *      (no 50% PWM jump).
 *   3. Retuned PID: Kp=2.5, Ki=0.05, Kd=0.3, DT=0.05 (20 Hz loop).
 *   4. EMA_ALPHA=0.5 — less filter lag, still smooths jitter.
 */

#include "obstacle_avoidance.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
#define EMA_ALPHA           0.5f           /* exponential moving average (0 = no update, 1 = raw) */

/* Steering */
#define STEER_GAIN          0.6f           /* how aggressively to turn (0..1) */
#define STEER_DEADBAND      5.0f           /* ignore L/R sensor diff below this (cm) */

/* Smooth‑transition zone (40 cm → 60 cm) */
#define TRANSITION_LOW      40.0f          /* below this ≡ full PID authority */
#define TRANSITION_HIGH     60.0f          /* above this ≡ full MAX_SPEED */

/* Emergency reverse */
#define EMERGENCY_THRESHOLD 10.0f          /* below this → reverse (if back is clear) */
#define REVERSE_SPEED       -50

/* Feed‑forward brake (when closing speed > BRAKE_VEL_THRESH while < BRAKE_RANGE) */
#define BRAKE_RANGE         60.0f
#define BRAKE_VEL_THRESH    10.0f          /* cm/s */
#define BRAKE_GAIN          0.3f

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

float graph_error[GRAPH_SIZE];
float graph_output[GRAPH_SIZE];
float graph_setpoint[GRAPH_SIZE];
int   graph_index = 0;

/* ---- internal state ---- */
static float prev_min_front = 999.0f;
static bool  pid_was_active = false;   /* tracks whether PID was in control last iteration */

/* ================================================================ */
/* Sensor helpers                                                    */
/* ================================================================ */

static void read_all_sensors(void)
{
    float val;

    val = ultrasonic_measure_cm(&front_sensor);
    if (val >= 0)
        sensor_front_cm = sensor_front_cm * (1.0f - EMA_ALPHA) + val * EMA_ALPHA;

    val = ultrasonic_measure_cm(&front_left_sensor);
    if (val >= 0)
        sensor_front_left_cm = sensor_front_left_cm * (1.0f - EMA_ALPHA) + val * EMA_ALPHA;

    val = ultrasonic_measure_cm(&front_right_sensor);
    if (val >= 0)
        sensor_front_right_cm = sensor_front_right_cm * (1.0f - EMA_ALPHA) + val * EMA_ALPHA;

    val = ultrasonic_measure_cm(&back_sensor);
    if (val >= 0)
        sensor_back_cm = sensor_back_cm * (1.0f - EMA_ALPHA) + val * EMA_ALPHA;
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
/* Steering logic                                                    */
/* ================================================================ */
/*
 * Returns a steering bias in the range [-MAX_SPEED, +MAX_SPEED].
 * Positive  → steer right (left wheels faster)
 * Negative  → steer left  (right wheels faster)
 *
 * Only active when both left and right sensors are valid and the
 * difference exceeds STEER_DEADBAND.
 */
static float compute_steering(float min_front)
{
    /* Only steer if we have both side sensors and obstacle is close */
    if (sensor_front_left_cm < 0 || sensor_front_right_cm < 0)
        return 0;

    if (min_front > 80.0f)
        return 0;   /* plenty of room → no need to steer */

    float diff = sensor_front_left_cm - sensor_front_right_cm;

    if (fabsf(diff) < STEER_DEADBAND)
        return 0;

    /* diff > 0 → left side clearer → turn left */
    float steer = diff * STEER_GAIN;

    /* Scale so that extreme differences (~40 cm) produce ~MAX_SPEED bias */
    if (steer >  MAX_SPEED) steer =  MAX_SPEED;
    if (steer < -MAX_SPEED) steer = -MAX_SPEED;

    return steer;
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

    /* Tuned PID: Kp=2.5  Ki=0.05  Kd=0.3  dt=0.05  out[0, 100] */
    pid_ctrl_init(&speed_pid, 2.5f, 0.05f, 0.3f, PID_DT, 0, MAX_SPEED);
    speed_pid.derivative_filter = 0.4f;   /* moderate D‑filtering */

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

                } else {
                    /* ---------------------------------------------------- */
                    /* Normal PID zone (≥ 10 cm)                            */
                    /* ---------------------------------------------------- */
                    if (!pid_was_active) {
                        pid_ctrl_reset(&speed_pid);
                    }

                    float pid_out = pid_ctrl_compute(&speed_pid, speed_setpoint, min_front);
                    if (pid_out < 0) pid_out = 0;

                    /* Smooth transition to MAX_SPEED when clear path */
                    speed_out = smooth_transition(min_front, pid_out);

                    /* Feed-forward braking */
                    float velocity = (prev_min_front - min_front) / PID_DT;
                    if (min_front < BRAKE_RANGE && velocity > BRAKE_VEL_THRESH) {
                        speed_out -= velocity * BRAKE_GAIN;
                        if (speed_out < 0) speed_out = 0;
                    }

                    /* Steering bias — only when we're moving forward and not emergency */
                    steer_bias = compute_steering(min_front);

                    pid_was_active = true;
                }

                prev_min_front = min_front;
            }

            /* ---- Apply steering to wheel speeds ---- */
            int32_t fr = (int32_t)speed_out;
            int32_t fl = (int32_t)speed_out;
            int32_t br = (int32_t)speed_out;
            int32_t bl = (int32_t)speed_out;

            if (speed_out > 0) {
                /* Only steer when moving forward */
                if (steer_bias > 0) {
                    /* steer right: left side faster, right side slower */
                    fl = (int32_t)(speed_out + steer_bias);
                    bl = (int32_t)(speed_out + steer_bias);
                    fr = (int32_t)(speed_out - steer_bias);
                    br = (int32_t)(speed_out - steer_bias);
                } else if (steer_bias < 0) {
                    /* steer left: right side faster */
                    float bias = -steer_bias;
                    fr = (int32_t)(speed_out + bias);
                    br = (int32_t)(speed_out + bias);
                    fl = (int32_t)(speed_out - bias);
                    bl = (int32_t)(speed_out - bias);
                }
            }

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

            /* Record graph data */
            graph_error[graph_index]   = speed_setpoint - min_front;
            graph_output[graph_index]  = speed_out;
            graph_setpoint[graph_index] = speed_setpoint;
            graph_index = (graph_index + 1) % GRAPH_SIZE;
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