#ifndef _MOTOR_DRIVER_H_
#define _MOTOR_DRIVER_H_

#include <math.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"

// Pin definitions for DRV8962
#define EN1 15
#define EN2 13
#define EN3 12
#define EN4 14

#define IN1 3
#define IN2 2
#define IN3 32
#define IN4 33

#define SLEEP 1

// PWM channel definitions
#define LEFT_FORWARD_PWM_CHANNEL LEDC_CHANNEL_1
#define RIGHT_FORWARD_PWM_CHANNEL LEDC_CHANNEL_2
#define LEFT_REVERSE_PWM_CHANNEL LEDC_CHANNEL_3
#define RIGHT_REVERSE_PWM_CHANNEL LEDC_CHANNEL_4

#define MAX_MOTOR_SPEED 255
#define MOTOR_LOGIC_1 256

/**
 * Registers the PWMs and GPIOs needed to control the motors, initializing them to an active, braked state.
 * Note this function must be called before the motors can be controlled.
 */
void motor_init();

/**
 * Brake both motors
 */
void motor_brake();

/**
 * Allow both motors to coast
 */
void motor_coast();

/**
 * Drive both motors forward at the desired speed
 * @param speed a number from 0 (minimum speed) to 255 (maximum speed)
 */
void motor_forward(uint8_t speed);

/**
 * Drive both motors in reverse at the desired speed
 * @param speed a number from 0 (minimum speed) to 255 (maximum speed)
 */
void motor_reverse(uint8_t speed);

/**
 * Turns right at the desired speed
 * Drives the left motor forward and the right motor in reverse
 * @param speed a number from 0 (stopped) to 255 (maximum speed)
 */
void motor_right(uint8_t speed);

/**
 * Turns left at the desired speed
 * Drives the right motor forward and the left motor in reverse
 * @param speed a number from 0 (minimum speed) to 255 (maximum speed)
 */
void motor_left(uint8_t speed);

/**
 * Enable/disable the DRV8962's low-power sleep mode
 * @param sleep true to enable, false to disable
 */
void motor_sleep(bool sleep);

#endif