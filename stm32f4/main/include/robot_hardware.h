// MIT License

// Copyright (c) 2020 thanhphong98

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef _ROBOT_HARDWARE_H_
#define _ROBOT_HARDWARE_H_ 

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "stm_err.h"
#include "stm_log.h"

#include "imu.h"
#include "stepmotor.h"
#include "software_resolver.h"
#include "mpu9250.h"
#include "madgwick.h"

#define PI                  3.14159265359

/* Convert constant */
#define DEG2RAD(x)      (x * PI / 180.0f)     /*!< Convert from degree to radian (PI/180) */
#define RAD2DEG(x)      (x * 180.0f / PI)     /*!< convert from radian to degree (180/PI) */

/* Step motor direction index */
#define MOTORLEFT_DIR_FORWARD       0
#define MOTORLEFT_DIR_BACKWARD      1
#define MOTORRIGHT_DIR_FORWARD      1
#define MOTORRIGHT_DIR_BACKWARD     0

/* Robot parameters */
#define WHEEL_RADIUS                0.033                                   /*!< Wheel radius in meter */
#define WHEEL_SEPARATION            0.165                                   /*!< Wheel separate distance in meter */
#define TURNING_RADIUS              0.08                                    /*!< Turning radius in degree */
#define ROBOT_RADIUS                0.1                                     /*!< Robot radius in meter    */
#define MAX_LINEAR_VELOCITY         (WHEEL_RADIUS * 2 * PI * 60 / 60)       /*!< Max linear velocity */
#define MAX_ANGULAR_VELOCITY        (MAX_LINEAR_VELOCITY / TURNING_RADIUS)  /*!< Max angular velocity */
#define MIN_LINEAR_VELOCITY         -MAX_LINEAR_VELOCITY                    /*!< Min linear velocity */
#define MIN_ANGULAR_VELOCITY        -MAX_ANGULAR_VELOCITY                   /*!< Min angular velocity */

/* Step driver parameters */
#define MICROSTEP_DIV               4           /*!< Step driver microstep divider */
#define NUM_PULSE_PER_ROUND         200         /*!< The number of pulse per round of motor */

/*
 *  Convert from velocity (m/s) to frequency (Hz) for motor driver.
 *
 *                      2*pi*WHELL_RADIUS
 *  velocity (m/s) =  ------------------------------
 *                    NUM_PULSE_PER_ROUND * STEP_DIV
 *
 */
#define VEL2FREQ        ((NUM_PULSE_PER_ROUND*MICROSTEP_DIV)/(2*PI*WHEEL_RADIUS))

/* Convert motor tick to angular in radian */
#define TICK2RAD        360.0/(NUM_PULSE_PER_ROUND*MICROSTEP_DIV)*PI/180

/* STM32 hardware */
#define MOTORLEFT_PULSE_TIMER_NUM           TIMER_NUM_1
#define MOTORLEFT_PULSE_TIMER_CHANNEL       TIMER_CHNL_1
#define MOTORLEFT_PULSE_TIMER_PINSPACK      TIMER_PINS_PACK_2
#define MOTORLEFT_DIR_GPIO_PORT             GPIO_PORT_A
#define MOTORLEFT_DIR_GPIO_NUM              GPIO_NUM_3
#define MOTORLEFT_TICK_TIMER_NUM            TIMER_NUM_2
#define MOTORLEFT_TICK_TIMER_PINSPACK       TIMER_PINS_PACK_2

#define MOTORRIGHT_PULSE_TIMER_NUM          TIMER_NUM_14
#define MOTORRIGHT_PULSE_TIMER_CHANNEL      TIMER_CHNL_1
#define MOTORRIGHT_PULSE_TIMER_PINSPACK     TIMER_PINS_PACK_1
#define MOTORRIGHT_DIR_GPIO_PORT            GPIO_PORT_C
#define MOTORRIGHT_DIR_GPIO_NUM             GPIO_NUM_5
#define MOTORRIGHT_TICK_TIMER_NUM           TIMER_NUM_4
#define MOTORRIGHT_TICK_TIMER_PINSPACK      TIMER_PINS_PACK_1

#define IMU_I2C_NUM                         I2C_NUM_1
#define IMU_I2C_PINSPACK                    I2C_PINS_PACK_1
#define IMU_CLOCK_SPEED                     100000

#define MADGWICK_BETA                       0.1f
#define MADGWICK_SAMPLE_FREQ                10.0f

#define STEP_DRIVER_PWM_DUTYCYCLE           50

stm_err_t robot_motor_init(void);
stm_err_t robot_imu_init(void);
stm_err_t robot_madgwick_filter_init(void);
stm_err_t robot_encoder_init(void);

stm_err_t robot_motor_left_start(void);
stm_err_t robot_motor_left_stop(void);
stm_err_t robot_motor_left_forward(void);
stm_err_t robot_motor_left_backward(void);
stm_err_t robot_motor_left_set_speed(float speed);

stm_err_t robot_motor_right_start(void);
stm_err_t robot_motor_right_stop(void);
stm_err_t robot_motor_right_forward(void);
stm_err_t robot_motor_right_backward(void);
stm_err_t robot_motor_right_set_speed(float speed);

stm_err_t robot_imu_update_quat(void);
stm_err_t robot_imu_get_quat(float *quat);
stm_err_t robot_imu_get_accel(float *accel);
stm_err_t robot_imu_get_gyro(float *gyro);

stm_err_t robot_encoder_left_get_tick(uint32_t *left_tick);
stm_err_t robot_encoder_right_get_tick(uint32_t *right_tick);


#ifdef __cplusplus
}
#endif

#endif /* _ROBOT_HARDWARE_H_ */