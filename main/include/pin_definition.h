#ifndef PIN_DEFINITION_H
#define PIN_DEFINITION_H

#include "soc/gpio_num.h"

/*
 * ==============================================================
 * PIN CONFIGURATION FOR ESP32 DEVKIT
 * ==============================================================
 *
 * DRV8833 STBY → connect directly to 3.3V (no GPIO needed)
 *
 * AVOIDED PINS (tested problematic):
 *   GPIO12, GPIO13 → strapping pins, MCPWM didn't work
 *   GPIO25         → MCPWM didn't work on this board
 *
 * MCPWM MAPPING (8 signals — forward & reverse both PWM):
 *   UNIT_0 TIMER_0 GEN_A=GPIO14=FR_fwd  GEN_B=GPIO27=FR_rev
 *   UNIT_0 TIMER_1 GEN_A=GPIO26=FL_fwd  GEN_B=GPIO32=FL_rev
 *   UNIT_0 TIMER_2 GEN_A=GPIO19=BR_fwd  GEN_B=GPIO18=BR_rev
 *   UNIT_1 TIMER_0 GEN_A=GPIO15=BL_fwd  GEN_B=GPIO33=BL_rev
 *
 *   Forward: GEN_A = speed%, GEN_B = 0%
 *   Reverse: GEN_A = 0%,     GEN_B = speed%
 * ==============================================================
 */

/* =============================
 * DRV8833 #1 - FRONT WHEELS
 * =============================
 */
#define DRV8833_1_AIN1    GPIO_NUM_26
#define DRV8833_1_AIN2    GPIO_NUM_32
#define DRV8833_1_BIN1    GPIO_NUM_14
#define DRV8833_1_BIN2    GPIO_NUM_27

#define MOTOR_FR_AIN1     DRV8833_1_BIN1
#define MOTOR_FR_AIN2     DRV8833_1_BIN2
#define MOTOR_FL_BIN1     DRV8833_1_AIN1
#define MOTOR_FL_BIN2     DRV8833_1_AIN2

/* =============================
 * DRV8833 #2 - BACK WHEELS
 * =============================
 */
#define DRV8833_2_AIN1    GPIO_NUM_15
#define DRV8833_2_AIN2    GPIO_NUM_33
#define DRV8833_2_BIN1    GPIO_NUM_19
#define DRV8833_2_BIN2    GPIO_NUM_18

#define MOTOR_BR_AIN1     DRV8833_2_BIN1
#define MOTOR_BR_AIN2     DRV8833_2_BIN2
#define MOTOR_BL_BIN1     DRV8833_2_AIN1
#define MOTOR_BL_BIN2     DRV8833_2_AIN2

/* =============================
 * Ultrasonic Sensors (RCWL-1601)
 *   VCC → 5V, ECHO → voltage divider to ESP32 GPIO
 * =============================
 */
#define SENSOR_FRONT_TRIG      GPIO_NUM_4
#define SENSOR_FRONT_ECHO      GPIO_NUM_5

#define SENSOR_FRONT_LEFT_TRIG  GPIO_NUM_2
#define SENSOR_FRONT_LEFT_ECHO  GPIO_NUM_16

#define SENSOR_FRONT_RIGHT_TRIG GPIO_NUM_17
#define SENSOR_FRONT_RIGHT_ECHO GPIO_NUM_21

#define SENSOR_BACK_TRIG       GPIO_NUM_22
#define SENSOR_BACK_ECHO       GPIO_NUM_23

#endif /* PIN_DEFINITION_H */
