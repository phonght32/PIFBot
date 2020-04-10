// MIT License

// Copyright (c) 2020 thanhphong98 & thuanpham98

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

#ifndef _TIMER_H_
#define _TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stdlib.h"

#include "stm32f4xx_hal.h"

#include "stm_err.h"
#include "stm_log.h"

/*
 * Table below shows all Timer's clock description.
 *
 *          |          |            |           |          |   Max     |  Max     |       |
 *  TIMER   |   Type   | Resolution | Prescalder| Channels | Interface |  Timer   |  APB  |
 *          |          |            |           |          |   Clock   |  Clock   |       |
 *  ----------------------------------------------------------------------------------------------
 *  TIM 1,  | Advanced |    16bit   |   16bit   |    4     | SysClk/2  | SysClk   |   2   |
 *  TIM 8   |          |            |           |          |           |          |       |
 *  ----------------------------------------------------------------------------------------------
 *  TIM 2,  | General  |    32bit   |   16bit   |    4     | SysClk/4  | SysClk,  |   1   |
 *  TIM 5   | purpose  |            |           |          |           | SysClk/2 |       |
 *  ----------------------------------------------------------------------------------------------
 *  TIM 3,  | General  |    16bit   |   16bit   |    4     | SysClk/4  | SysClk,  |   1   |
 *  TIM 4   | purpose  |            |           |          |           | SysClk/2 |       |
 *  ----------------------------------------------------------------------------------------------
 *  TIM 9   | General  |    16bit   |   16bit   |    2     | SysClk/2  | SysClk   |   2   |
 *          | purpose  |            |           |          |           |          |       |
 *  ----------------------------------------------------------------------------------------------
 *  TIM 10  | General  |    16bit   |   16bit   |    1     | SysClk/2  | SysClk   |   2   |
 *  TIM 11  | purpose  |            |           |          |           |          |       |
 *  ----------------------------------------------------------------------------------------------
 *  TIM 12  | General  |    16bit   |   16bit   |    2     | SysClk/4  | SysClk,  |   1   |
 *          | purpose  |            |           |          |           | SysClk/2 |       |
 *  ----------------------------------------------------------------------------------------------
 *  TIM 13  | General  |    16bit   |   16bit   |    1     | SysClk/4  | SysClk,  |   1   |
 *  TIM 14  | purpose  |            |           |          |           | SysClk/2 |       |
 *  ----------------------------------------------------------------------------------------------
 *  TIM 6   |  Basic   |    16bit   |   16bit   |    0     | SysClk/4  | SysClk,  |   1   |
 *  TIM 7   |          |            |           |          |           | SysClk/2 |       |
 *  ----------------------------------------------------------------------------------------------
 */


/*
 * Table below shows all possible pins for each timer and channel.
 * You can select any of max 3 pins for each output channel.
 *
 *    TIMER |      CHANNEL1       |      CHANNEL 2      |      CHANNEL 3      |      CHANNEL 4
 *          |PP1    PP2    PP3    |PP1    PP2    PP3    |PP1    PP2    PP3    |PP1    PP2    PP3
 *  ----------------------------------------------------------------------------------------------
 *  TIM 1   |PA8    PE9    -      |PA9    PE10   -      |PA10   PE13   -      |PA11   PE14   -
 *  TIM 2   |PA0    PA5    PA15   |PA1    PB3    -      |PA2    PB10   -      |PA3    PB11   -
 *  TIM 3   |PA6    PB4    PC6    |PA7    PB5    PC7    |PB0    PC8    -      |PB1    PC9    -
 *  TIM 4   |PB6    PD12   -      |PB7    PD13   -      |PB8    PD14   -      |PB9    PD15   -
 *  TIM 5   |PA0    PH10   -      |PA1    PH11   -      |PA2    PH12   -      |PA3    PI0    -
 *  TIM 8   |PC6    PI5    -      |PC7    PI6    -      |PC8    PI7    -      |PC9    PI2    -
 *  TIM 9   |PA2    PE5    -      |PA3    PE6    -      |-      -      -      |-      -      -
 *  TIM 10  |PB8    PF6    -      |-      -      -      |-      -      -      |-      -      -
 *  TIM 11  |PB9    PF7    -      |-      -      -      |-      -      -      |-      -      -
 *  TIM 12  |PB14   PH6    -      |PB15   PH9    -      |-      -      -      |-      -      -
 *  TIM 13  |PA6    PF8    -      |-      -      -      |-      -      -      |-      -      -
 *  TIM 14  |PA7    PF9    -      |-      -      -      |-      -      -      |-      -      -
 *  ----------------------------------------------------------------------------------------------
 *
 * Notes on table above:
 *  - Not all timers are available on all STM32F4xx devices.
 *  - All timers have 16-bit prescaler.
 *  - TIM6 and TIM7 don't have PWM feature, they are only basic timers.
 *  - TIM2 and TIM5 are 32bit timers.
 *  - TIM9 and TIM12 have two PWM channels.
 *  - TIM10, TIM11, TIM13 and TIM14 have only one PWM channel.
 *  - All channels at one timer have the same PWM frequency!
 */

/*
 * Table below shows all possible pins for external input counter.
 *
 *  TIMER   |PP1    PP2     PP3
 *  -----------------------------
 *  TIM 1   |PA12   PE7     -
 *  TIM 2   |PA0    PA5     PA15
 *  TIM 3   |PD2    -       -
 *  TIM 4   |PE0    -       -
 *  TIM 5   |-      -       -
 *  TIM 8   |PA0    -       -
 *  TIM 9   |-      -       -
 *  TIM 10  |-      -       -
 *  TIM 11  |-      -       -
 *  TIM 12  |-      -       -
 *  TIM 13  |-      -       -
 *  TIM 14  |-      -       -
 *  -------------------------------
 *
 */

typedef enum {
    TIMER_NUM_1 = 0,        /*!< Timer Num 1 selected */
    TIMER_NUM_2,            /*!< Timer Num 2 selected */
    TIMER_NUM_3,            /*!< Timer Num 3 selected */
    TIMER_NUM_4,            /*!< Timer Num 4 selected */
    TIMER_NUM_5,            /*!< Timer Num 5 selected */
    TIMER_NUM_6,            /*!< Timer Num 6 selected */
    TIMER_NUM_7,            /*!< Timer Num 7 selected */
    TIMER_NUM_8,            /*!< Timer Num 8 selected */
    TIMER_NUM_9,            /*!< Timer Num 9 selected */
    TIMER_NUM_10,           /*!< Timer Num 10 selected */
    TIMER_NUM_11,           /*!< Timer Num 11 selected */
    TIMER_NUM_12,           /*!< Timer Num 12 selected */
    TIMER_NUM_13,           /*!< Timer Num 13 selected */
    TIMER_NUM_14,           /*!< Timer Num 14 selected */
    TIMER_NUM_MAX
} timer_num_t;

typedef enum {
    TIMER_CHANNEL_1 = 0,    /*!< Timer Channel 1 selected */
    TIMER_CHANNEL_2,        /*!< Timer Channel 2 selected */
    TIMER_CHANNEL_3,        /*!< Timer Channel 3 selected */
    TIMER_CHANNEL_4,        /*!< Timer Channel 4 selected */
    TIMER_CHANNEL_MAX
} timer_channel_t;

typedef enum {
    TIMER_PINS_PACK_1 = 0,  /*!< Timer Pins Pack 1 selected */
    TIMER_PINS_PACK_2,      /*!< Timer Pins Pack 2 selected */
    TIMER_PINS_PACK_3,      /*!< Timer Pins Pack 3 selected */
    TIMER_PINS_PACK_MAX
} timer_pins_pack_t;

typedef enum {
    TIMER_COUNTER_UP = 0,   /*!< Timer counter mode up */
    TIMER_COUNTER_DOWN,     /*!< Timer counter mode down */
    TIMER_COUNTER_MODE_MAX
} timer_counter_mode_t;

typedef struct {
    timer_num_t         timer_num;              /*!< Timer Num */
    timer_channel_t     timer_channel;          /*!< Timer Channel */
    timer_pins_pack_t   timer_pins_pack;        /*!< Timer Pins Pack */
} pwm_config_t;

typedef struct {
    timer_num_t             timer_num;          /*!< Timer Num */
    timer_pins_pack_t       timer_pins_pack;    /*!< Timer Pins Pack */
    uint32_t                max_reload;         /*!< Timer Max Reload value */
    timer_counter_mode_t    counter_mode;       /*!< Timer Counter Mode */
} etr_config_t;

/*
 * @brief   Initialize PWM.
 * @param   config Struct pointer.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t pwm_config(pwm_config_t *config);

/*
 * @brief   Start PWM generate.
 * @param   timer_num Timer num.
 * @param   timer_channel Timer channel.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t pwm_start(timer_num_t timer_num, timer_channel_t timer_channel);

/*
 * @brief   Stop PWM generate.
 * @param   timer_num Timer num.
 * @param   timer_channel Timer channel.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t pwm_stop(timer_num_t timer_num, timer_channel_t timer_channel);

/*
 * @brief   Set PWM frequency in Hz.
 * @param   timer_num Timer num.
 * @param   timer_channel Timer channel.
 * @param   freq_hz Frequency in Hz.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t pwm_set_frequency(timer_num_t timer_num, timer_channel_t timer_channel, uint32_t freq_hz);

/*
 * @brief   Set PWM duty cycle in %.
 * @param   timer_num Timer num.
 * @param   timer_channel Timer channel.
 * @param   duty_percent Duty cycle in %.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t pwm_set_duty(timer_num_t timer_num, timer_channel_t timer_channel, uint8_t duty_percent);

/*
 * @brief   Set PWM set frequency and duty cycle.
 * @param   timer_num Timer num.
 * @param   freq_hz Frequency in Hz.
 * @param   timer_channel Timer channel.
 * @param   duty_percent Duty cycle in %.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t pwm_set_params(timer_num_t timer_num, timer_channel_t timer_channel, uint32_t freq_hz, uint8_t duty_percent);

/*
 * @brief   Initialize External Counter input.
 * @param   config Struct pointer.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t etr_config(etr_config_t *config);

/*
 * @brief   Start counter external pulse input.
 * @param   timer_num Timer num.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t etr_start(timer_num_t timer_num);

/*
 * @brief   Start counter external pulse input.
 * @param   timer_num Timer num.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t etr_stop(timer_num_t timer_num);

/*
 * @brief   Get counter value.
 * @param   timer_num Timer num.
 * @param   value Counter value.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t etr_get_value(timer_num_t timer_num, uint32_t *value);

/*
 * @brief   Set counter value.
 * @param   timer_num Timer num.
 * @param   value Counter value.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t etr_set_value(timer_num_t timer_num, uint32_t value);

/*
 * @brief   Set counter mode.
 * @param   timer_num Timer num.
 * @param   counter_mode Counter mode.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t etr_set_mode(timer_num_t timer_num, timer_counter_mode_t counter_mode);


#ifdef __cplusplus
}
#endif

#endif /* _TIMER_H_ */
