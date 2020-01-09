#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"



#include "../components/driver/include/timer.h"
#include "../components/driver/include/i2c.h"
#include "../components/driver/include/gpio.h"
#include "../components/driver/include/uart.h"
#include "../components/mpu6050/include/mpu6050.h"
#include "../components/step_driver/include/step_driver.h"

/* Motor hardware define */
#define MOTORLEFT_TIMER_NUM			TIMER_NUM_3
#define MOTORLEFT_TIMER_CHANNEL		TIMER_CHANNEL_2
#define MOTORLEFT_TIMER_PINSPACK	TIMER_PINS_PACK_1
#define MOTORLEFT_GPIO_PORT			GPIO_PORT_C
#define MOTORLEFT_GPIO_NUM			GPIO_NUM_5

#define MOTORRIGHT_TIMER_NUM		TIMER_NUM_2
#define MOTORRIGHT_TIMER_CHANNEL	TIMER_CHANNEL_1
#define MOTORRIGHT_TIMER_PINSPACK	TIMER_PINS_PACK_2
#define MOTORRIGHT_GPIO_PORT		GPIO_PORT_A
#define MOTORRIGHT_GPIO_NUM			GPIO_NUM_3

/* MPU6050 hardware define */
#define MPU6050_I2C_NUM			I2C_NUM_1
#define MPU6050_I2C_PINSPACK	I2C_PINS_PACK_1

/* Control motor macros */
#define MOTOR_LEFT_FORWARD(_handle_)    step_driver_set_dir(_handle_, 0);
#define MOTOR_LEFT_BACKWARD(_handle_)   step_driver_set_dir(_handle_, 1);
#define MOTOR_RIGHT_FORWARD(_handle_)   step_driver_set_dir(_handle_, 1);
#define MOTOR_RIGHT_BACKWARD(_handle_)  step_driver_set_dir(_handle_, 0);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


