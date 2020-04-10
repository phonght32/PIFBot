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

#ifndef _STEPMOTOR_H_
#define _STEPMOTOR_H_ 

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#include "stm32f4xx_hal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "stm_err.h"
#include "stm_log.h"
#include "driver/gpio.h"
#include "driver/timer.h"

typedef struct stepmotor *stepmotor_handle_t;

typedef enum {
    STEPMOTOR_READY = 0,
    STEPMOTOR_RUNNING,
    STEPMOTOR_STOP,
    STEPMOTOR_ERROR
} stepmotor_status_t;

typedef struct {
    gpio_port_t         dir_gpio_port;          /*!< Pin dir GPIO Port */
    gpio_num_t          dir_gpio_num;           /*!< Pin dir GPIO Num */
    timer_num_t         pulse_timer_num;        /*!< Pin pulse Timer Num */
    timer_pins_pack_t   pulse_timer_pins_pack;  /*!< Pin pulse Timer Pins Pack */
    timer_channel_t     pulse_timer_channel;    /*!< Pin pulse Timer Channel */
} stepmotor_config_t;

/* 
 * @brief   Configure pin dir and pulse to control step drivers.
 * @param   config Struct pointer.
 * @return  
 *      - Step motor handle structure: Success.
 *      - (-1): Fail
 */
stepmotor_handle_t stepmotor_config(stepmotor_config_t *config);

/*
 * @brief   Set step motor direction.
 * @param   handle Handle structure.
 * @param   dir Direction.
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t stepmotor_set_dir(stepmotor_handle_t handle, bool dir);

/*
 * @brief   Toggle step motor direction.
 * @param   handle Handle structure.
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t stepmotor_toggle_dir(stepmotor_handle_t handle);

/*
 * @brief   Set step motor frequency.
 * @param   handle Handle structure.
 * @param   freq_hz Frequency in Hz.
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t stepmotor_set_pwm_freq(stepmotor_handle_t handle, uint32_t freq_hz);

/*
 * @brief   Set pwm duty cycle.
 * @param   handle Handle structure.
 * @param   duty Duty cycle in %.
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t stepmotor_set_pwm_duty(stepmotor_handle_t handle, uint8_t duty);

/*
 * @brief   Start step motor.
 * @param   handle Handle structure.
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t stepmotor_start(stepmotor_handle_t handle);

/*
 * @brief   Stop step motor.
 * @param   handle Handle structure.
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t stepmotor_stop(stepmotor_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* _STEPMOTOR_H_ */