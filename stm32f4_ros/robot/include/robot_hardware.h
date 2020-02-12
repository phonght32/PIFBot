#ifndef _ROBOT_HARDWARE_H_
#define _ROBOT_HARDWARE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "../../stm32f4_library/driver/include/timer.h"
#include "../../stm32f4_library/driver/include/i2c.h"
#include "../../stm32f4_library/driver/include/gpio.h"
#include "../../stm32f4_library/driver/include/uart.h"
#include "../../stm32f4_library/imu/include/mpu6050.h"
#include "../../stm32f4_library/motor/include/step_motor.h"

#define PI                          3.14159265359

/* Model parameters */
#define WHEEL_RADIUS                0.033       /*!< meters */
#define WHEEL_SEPARATION            0.173       /*!< meters */
#define TURNING_RADIUS              0.08        /*!< rad    */
#define ROBOT_RADIUS                0.1         /*!< rad    */

#define MAX_LINEAR_VELOCITY         (WHEEL_RADIUS * 2 * PI * 60 / 60)
#define MAX_ANGULAR_VELOCITY        (MAX_LINEAR_VELOCITY / TURNING_RADIUS)

#define MIN_LINEAR_VELOCITY         -MAX_LINEAR_VELOCITY
#define MIN_ANGULAR_VELOCITY        -MAX_ANGULAR_VELOCITY


/* Step driver parameters */
#define STEP_DIV                    4
#define NUM_PULSE_PER_ROUND         200

/* Convert from velocity (m/s) to frequency (Hz) for motor driver
 *                      2*pi*WHELL_RADIUS
 *  velocity (m/s) =  ------------------------------
 *                    NUM_PULSE_PER_ROUND * STEP_DIV
 *
 */
#define VEL2FREQ                    ((NUM_PULSE_PER_ROUND*STEP_DIV)/(2*PI*WHEEL_RADIUS))

/* Robot hardware define */
#define MOTORLEFT_TIMER_NUM         TIMER_NUM_4
#define MOTORLEFT_TIMER_CHANNEL     TIMER_CHANNEL_1
#define MOTORLEFT_TIMER_PINSPACK    TIMER_PINS_PACK_2
#define MOTORLEFT_GPIO_PORT         GPIO_PORT_A
#define MOTORLEFT_GPIO_NUM          GPIO_NUM_3

#define MOTORRIGHT_TIMER_NUM        TIMER_NUM_3
#define MOTORRIGHT_TIMER_CHANNEL    TIMER_CHANNEL_2
#define MOTORRIGHT_TIMER_PINSPACK   TIMER_PINS_PACK_1
#define MOTORRIGHT_GPIO_PORT        GPIO_PORT_C
#define MOTORRIGHT_GPIO_NUM         GPIO_NUM_5

#define MPU6050_I2C_NUM             I2C_NUM_1
#define MPU6050_I2C_PINSPACK        I2C_PINS_PACK_1

#define ROSSERIAL_BAUDRATE          57600

/* Robot control function */
void robot_motor_init(void);
void robot_imu_init(void);
void robot_rosserial_init(void);

void robot_motor_left_forward(void);
void robot_motor_left_backward(void);
void robot_motor_left_set_speed(float speed);

void robot_motor_right_forward(void);
void robot_motor_right_backward(void);
void robot_motor_right_set_speed(float speed);


#ifdef __cplusplus
}
#endif

#endif /* _ROBOT_HARDWARE_H_ */
