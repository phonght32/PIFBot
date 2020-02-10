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


/* Model parameters */
#define WHEEL_RADIUS				0.033		/*!< meters */
#define WHEEL_SEPARATION 			0.173		/*!< meters */
#define TURNING_RADIUS 				0.08		/*!< rad	*/
#define ROBOT_RADIUS 				0.1			/*!< rad 	*/

#define MAX_LINEAR_VELOCITY 		(WHEEL_RADIUS * 2 * 3.14159265359 * 60 / 60)
#define MAX_ANGULAR_VELOCITY 		(MAX_LINEAR_VELOCITY / TURNING_RADIUS)

#define MIN_LINEAR_VELOCITY			-MAX_LINEAR_VELOCITY
#define MIN_ANGULAR_VELOCITY		-MAX_ANGULAR_VELOCITY


/* 						    2*pi*WHELL_RADIUS
 *	velocity (m/s) = ------------------------------
 *					  NUM_PULSE_PER_ROUND * STEP_DIV
 *
 *	NUM_PULSE_PER_ROUND: 200
 *	STEP_DIV           : 16
 *	WHEEL_RADIUS	   : 0.033
 */
#define VEL2FREQ					15433

/* Step driver parameters */
#define STEP_DIV					16

/* Motor hardware define */
#define MOTORLEFT_TIMER_NUM			TIMER_NUM_4
#define MOTORLEFT_TIMER_CHANNEL		TIMER_CHANNEL_1
#define MOTORLEFT_TIMER_PINSPACK	TIMER_PINS_PACK_2
#define MOTORLEFT_GPIO_PORT			GPIO_PORT_A
#define MOTORLEFT_GPIO_NUM			GPIO_NUM_3

#define MOTORRIGHT_TIMER_NUM		TIMER_NUM_3
#define MOTORRIGHT_TIMER_CHANNEL	TIMER_CHANNEL_2
#define MOTORRIGHT_TIMER_PINSPACK	TIMER_PINS_PACK_1
#define MOTORRIGHT_GPIO_PORT		GPIO_PORT_C
#define MOTORRIGHT_GPIO_NUM			GPIO_NUM_5

/* MPU6050 hardware define */
#define MPU6050_I2C_NUM				I2C_NUM_1
#define MPU6050_I2C_PINSPACK		I2C_PINS_PACK_1

/* Control motor macros */
#define MOTOR_LEFT_FORWARD(_handle_)    step_motor_set_dir(_handle_, 0)
#define MOTOR_LEFT_BACKWARD(_handle_)   step_motor_set_dir(_handle_, 1)
#define MOTOR_RIGHT_FORWARD(_handle_)   step_motor_set_dir(_handle_, 1)
#define MOTOR_RIGHT_BACKWARD(_handle_)  step_motor_set_dir(_handle_, 0)

#define MOTOR_SET_SPEED(_handle_,speed) step_motor_set_freq(_handle_, (uint32_t)(speed*VEL2FREQ))

#define MOTOR_START(_handle_)			step_motor_start(_handle_)
#define MOTOR_STOP(_handle_)			step_motor_stop(_handle_)


void robot_motor_init(void);
void robot_mpu6050_init(void);

#ifdef __cplusplus
}
#endif

#endif /* _ROBOT_HARDWARE_H_ */
