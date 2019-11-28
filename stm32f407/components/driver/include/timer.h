#ifndef _TIMER_H
#define _TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/* Table below shows all possible pins for each timer and channel.
 * You can select any of max 3 pins for each output channel.
 *
	  TIMER |       CHANNEL       |      CHANNEL 2      |      CHANNEL 3      |      CHANNEL 4
        	|PP1    PP2    PP3    |PP1    PP2    PP3    |PP1    PP2    PP3    |PP1    PP2    PP3
    ____________________________________________________________________________________________
	TIM 1   |PA8    PE9    -      |PA9    PE10   -      |PA10   PE13   -      |PA11   PE14   -
	TIM 2   |PA0    PA5    PA15   |PA1    PB3    -      |PA2    PB10   -      |PA3    PB11   -
	TIM 3   |PA6    PB4    PC6    |PA7    PB5    PC7    |PB0    PC8    -      |PB1    PC9    -
	TIM 4   |PB6    PD12   -      |PB7    PD13   -      |PB8    PD14   -      |PB9    PD15   -
	TIM 5   |PA0    PH10   -      |PA1    PH11   -      |PA2    PH12   -      |PA3    PI0    -
	TIM 8   |PC6    PI5    -      |PC7    PI6    -      |PC8    PI7    -      |PC9    PI2    -
	TIM 9   |PA2    PE5    -      |PA3    PE6    -      |-      -      -      |-      -      -
	TIM 10  |PB8    PF6    -      |-      -      -      |-      -      -      |-      -      -
	TIM 11  |PB9    PF7    -      |-      -      -      |-      -      -      |-      -      -
	TIM 12  |PB14   PH6    -      |PB15   PH9    -      |-      -      -      |-      -      -
	TIM 13  |PA6    PF8    -      |-      -      -      |-      -      -      |-      -      -
	TIM 14  |PA7    PF9    -      |-      -      -      |-      -      -      |-      -      -
    ____________________________________________________________________________________________
*
* Notes on table above:
* 	- Not all timers are available on all STM32F4xx devices
* 	- All timers have 16-bit prescaler
* 	- TIM6 and TIM7 don't have PWM feature, they are only basic timers
* 	- TIM2 and TIM5 are 32bit timers
* 	- TIM9 and TIM12 have two PWM channels
* 	- TIM10, TIM11, TIM13 and TIM14 have only one PWM channel
* 	- All channels at one timer have the same PWM frequency!
*/

typedef struct pwm_param *pwm_handle_t;

typedef enum {
    TIMER_NUM_1 = 0,
    TIMER_NUM_2,
    TIMER_NUM_3,
    TIMER_NUM_4,
    TIMER_NUM_5,
    TIMER_NUM_6,
    TIMER_NUM_7,
    TIMER_NUM_8,
    TIMER_NUM_9,
    TIMER_NUM_10,
    TIMER_NUM_11,
    TIMER_NUM_12,
    TIMER_NUM_13,
    TIMER_NUM_14,
    TIMER_NUM_MAX
} timer_num_t;

typedef enum {
    PWM_PINS_PACK_1 = 0,
    PWM_PINS_PACK_2,
    PWM_PINS_PACK_3,
    PWM_PINS_PACK_MAX
} pwm_pins_pack_t;

typedef enum {
    PWM_CHANNEL_1 = 0,
    PWM_CHANNEL_2,
    PWM_CHANNEL_3,
    PWM_CHANNEL_4,
    PWM_CHANNEL_MAX
} pwm_channel_t;

typedef struct {
    timer_num_t 		timer;
    uint32_t 			timer_period;
    uint16_t 			timer_prescaler;
    pwm_channel_t 		pwm_channel;
    pwm_pins_pack_t 	pwm_pins_pack;
    uint8_t 			pwm_duty;
} pwm_config_t;

pwm_handle_t pwm_init(pwm_config_t *config);

#ifdef __cplusplus
}
#endif

#endif
