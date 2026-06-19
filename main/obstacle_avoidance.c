#include "obstacle_avoidance.h"

#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pin_definition.h"
#include "ultrasonic.h"
#include "motor_control.h"
#include "pid_controller.h"
#include "shared_data.h"

#define MAX_SPEED         100
#define PID_DT            0.03f
#define BACK_THRESHOLD    15.0f
#define OVERRIDE_TIMEOUT  40
#define BRAKE_THRESHOLD   60.0f
#define EMA_ALPHA         0.3f

static ultrasonic_sensor_t front_sensor;
static ultrasonic_sensor_t front_left_sensor;
static ultrasonic_sensor_t front_right_sensor;
static ultrasonic_sensor_t back_sensor;

volatile float sensor_front_cm = 0;
volatile float sensor_front_left_cm = 0;
volatile float sensor_front_right_cm = 0;
volatile float sensor_back_cm = 0;

volatile int motor_fr_speed = 0;
volatile int motor_fl_speed = 0;
volatile int motor_br_speed = 0;
volatile int motor_bl_speed = 0;

pid_ctrl_t speed_pid;

volatile int manual_override = 0;
volatile int override_count = 0;
volatile int auto_mode = 0;

volatile float speed_setpoint = 40.0f;

float graph_error[GRAPH_SIZE];
float graph_output[GRAPH_SIZE];
float graph_setpoint[GRAPH_SIZE];
int graph_index = 0;

static float prev_min_front = 999.0f;

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
    if (sensor_front_cm >= 0 && sensor_front_cm < min_val) min_val = sensor_front_cm;
    if (sensor_front_left_cm >= 0 && sensor_front_left_cm < min_val) min_val = sensor_front_left_cm;
    if (sensor_front_right_cm >= 0 && sensor_front_right_cm < min_val) min_val = sensor_front_right_cm;
    return (min_val > 998.0f) ? -1 : min_val;
}

static void avoid_task(void *arg)
{
    motor_control_init();

    pid_ctrl_init(&speed_pid, 1.0f, 0.1f, 0.05f, PID_DT, 0, MAX_SPEED);

    while (1) {
        read_all_sensors();

        if (manual_override) {
            override_count++;
            if (override_count > OVERRIDE_TIMEOUT) {
                manual_override = 0;
                override_count = 0;
            }
        }

        if (auto_mode && !manual_override) {
            float speed_out = 0;
            float min_front = get_min_front();

            if (min_front >= 0) {
                if (min_front > speed_setpoint) {
                    speed_out = MAX_SPEED;
                } else if (min_front < 10.0f) {
                    if (sensor_back_cm >= BACK_THRESHOLD || sensor_back_cm < 0) {
                        speed_out = -50;
                    } else {
                        speed_out = 0;
                    }
                } else {
                    speed_out = pid_ctrl_compute(&speed_pid, speed_setpoint, min_front);
                    if (speed_out < 0) speed_out = 0;

                    float velocity = (prev_min_front - min_front) / PID_DT;
                    if (min_front < BRAKE_THRESHOLD && velocity > 10.0f) {
                        speed_out -= velocity * 0.3f;
                        if (speed_out < 0) speed_out = 0;
                    }
                }
                prev_min_front = min_front;
            }

            motor_set_speed(speed_out, speed_out, speed_out, speed_out);
            motor_fr_speed = (int)speed_out;
            motor_fl_speed = (int)speed_out;
            motor_br_speed = (int)speed_out;
            motor_bl_speed = (int)speed_out;

            graph_error[graph_index] = speed_setpoint - min_front;
            graph_output[graph_index] = speed_out;
            graph_setpoint[graph_index] = speed_setpoint;
            graph_index = (graph_index + 1) % GRAPH_SIZE;
        }

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
