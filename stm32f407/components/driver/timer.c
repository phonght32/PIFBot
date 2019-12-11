/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_conf.h"

#include "stdlib.h"

#include "include/timer.h"


/* Internal define -----------------------------------------------------------*/
#define GPIO_SPEED_FREQ	GPIO_Speed_100MHz
#define GPIO_PUPD 		GPIO_PuPd_UP

#define SYSTEM_CLOCK	168000000
#define APB1_CLOCK		(SYSTEM_CLOCK/2)
#define APB2_CLOCK 		(SYSTEM_CLOCK)

#define TIMER_MAX_RELOAD	0xFFFF


/* Internal typedef ----------------------------------------------------------*/
typedef enum {
	PWM_PARAM_MAPPING_GPIOx = 0,
	PWM_PARAM_MAPPING_GPIO_Pin_x,
	PWM_PARAM_MAPPING_RCC_AHBxPeriph_GPIOx,
	PWM_PARAM_MAPPING_PinSourcex,
	PWM_PARAM_MAPPING_RCC_APBxPeriph_TIMx,
	PWM_PARAM_MAPPING_GPIO_AF_TIMx,
	PWM_PARAM_MAPPING_MAX_INDEX
} pwm_param_mapping_index_t;

typedef struct pwm_param {
	timer_num_t 	timer;
	uint32_t 		timer_period;
	uint16_t 		timer_prescaler;
	pwm_channel_t 	pwm_channel;
	pwm_pins_pack_t pwm_pins_pack;
	uint8_t 		pwm_duty;
	uint32_t 		pwm_freq_hz;
} pwm_param_t;


/* Internal variable ---------------------------------------------------------*/
/*
 * PWM Parameters map
 * Column:	Selected by Timer
 * Row:   	Selected by PWM channel
 * Index:
 *	- 0:	(GPIO_TypeDef *) GPIOx
 *  - 1:    (uint16_t)       GPIO_Pin_x
 *  - 2:	(uint32_t)       RCC_AHBxPeriph_GPIOx
 *  - 3:    (uint8_t)        GPIO_PinSourcex
 *  - 4:	(uint32_t        RCC_APBxPeriph_TIMx
 *  - 5:    (uint8_t)        GPIO_AF_TIMx
 */
uint32_t PWM_PARAM_MAPPING_PP1[PWM_CHANNEL_MAX][TIMER_NUM_MAX][PWM_PARAM_MAPPING_MAX_INDEX] = {
	{	{(uint32_t)GPIOA,  GPIO_Pin_8, RCC_AHB1Periph_GPIOA,  GPIO_PinSource8,  RCC_APB2Periph_TIM1,  GPIO_AF_TIM1},
		{(uint32_t)GPIOA,  GPIO_Pin_0, RCC_AHB1Periph_GPIOA,  GPIO_PinSource0,  RCC_APB1Periph_TIM2,  GPIO_AF_TIM2},
		{(uint32_t)GPIOA,  GPIO_Pin_6, RCC_AHB1Periph_GPIOA,  GPIO_PinSource6,  RCC_APB1Periph_TIM3,  GPIO_AF_TIM3},
		{(uint32_t)GPIOB,  GPIO_Pin_6, RCC_AHB1Periph_GPIOB,  GPIO_PinSource6,  RCC_APB1Periph_TIM4,  GPIO_AF_TIM4},
		{(uint32_t)GPIOA,  GPIO_Pin_0, RCC_AHB1Periph_GPIOA,  GPIO_PinSource0,  RCC_APB1Periph_TIM5,  GPIO_AF_TIM5},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{(uint32_t)GPIOC,  GPIO_Pin_6, RCC_AHB1Periph_GPIOC,  GPIO_PinSource6,  RCC_APB2Periph_TIM8,  GPIO_AF_TIM8},
		{(uint32_t)GPIOA,  GPIO_Pin_2, RCC_AHB1Periph_GPIOA,  GPIO_PinSource2,  RCC_APB2Periph_TIM9,  GPIO_AF_TIM9},
		{(uint32_t)GPIOB,  GPIO_Pin_8, RCC_AHB1Periph_GPIOB,  GPIO_PinSource8, RCC_APB2Periph_TIM10, GPIO_AF_TIM10},
		{(uint32_t)GPIOB,  GPIO_Pin_9, RCC_AHB1Periph_GPIOB,  GPIO_PinSource9, RCC_APB2Periph_TIM11, GPIO_AF_TIM11},
		{(uint32_t)GPIOB, GPIO_Pin_14, RCC_AHB1Periph_GPIOB, GPIO_PinSource14, RCC_APB1Periph_TIM12, GPIO_AF_TIM12},
		{(uint32_t)GPIOA,  GPIO_Pin_6, RCC_AHB1Periph_GPIOA,  GPIO_PinSource6, RCC_APB1Periph_TIM13, GPIO_AF_TIM13},
		{(uint32_t)GPIOA,  GPIO_Pin_7, RCC_AHB1Periph_GPIOA,  GPIO_PinSource7, RCC_APB1Periph_TIM14, GPIO_AF_TIM14}
	},

	{	{(uint32_t)GPIOA,  GPIO_Pin_9, RCC_AHB1Periph_GPIOA,  GPIO_PinSource9,  RCC_APB2Periph_TIM1,  GPIO_AF_TIM1},
		{(uint32_t)GPIOA,  GPIO_Pin_1, RCC_AHB1Periph_GPIOA,  GPIO_PinSource1,  RCC_APB1Periph_TIM2,  GPIO_AF_TIM2},
		{(uint32_t)GPIOA,  GPIO_Pin_7, RCC_AHB1Periph_GPIOA,  GPIO_PinSource7,  RCC_APB1Periph_TIM3,  GPIO_AF_TIM3},
		{(uint32_t)GPIOB,  GPIO_Pin_7, RCC_AHB1Periph_GPIOB,  GPIO_PinSource7,  RCC_APB1Periph_TIM4,  GPIO_AF_TIM4},
		{(uint32_t)GPIOA,  GPIO_Pin_1, RCC_AHB1Periph_GPIOA,  GPIO_PinSource1,  RCC_APB1Periph_TIM5,  GPIO_AF_TIM5},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{(uint32_t)GPIOC,  GPIO_Pin_7, RCC_AHB1Periph_GPIOC,  GPIO_PinSource7,  RCC_APB2Periph_TIM8,  GPIO_AF_TIM8},
		{(uint32_t)GPIOA,  GPIO_Pin_3, RCC_AHB1Periph_GPIOA,  GPIO_PinSource3,  RCC_APB2Periph_TIM9,  GPIO_AF_TIM9},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{(uint32_t)GPIOB, GPIO_Pin_15, RCC_AHB1Periph_GPIOB, GPIO_PinSource15, RCC_APB1Periph_TIM12, GPIO_AF_TIM12},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0}
	},

	{	{(uint32_t)GPIOA, GPIO_Pin_10, RCC_AHB1Periph_GPIOA, GPIO_PinSource10,  RCC_APB2Periph_TIM1,  GPIO_AF_TIM1},
		{(uint32_t)GPIOA,  GPIO_Pin_2, RCC_AHB1Periph_GPIOA,  GPIO_PinSource2,  RCC_APB1Periph_TIM2,  GPIO_AF_TIM2},
		{(uint32_t)GPIOB,  GPIO_Pin_0, RCC_AHB1Periph_GPIOB,  GPIO_PinSource0,  RCC_APB1Periph_TIM3,  GPIO_AF_TIM3},
		{(uint32_t)GPIOB,  GPIO_Pin_8, RCC_AHB1Periph_GPIOB,  GPIO_PinSource8,  RCC_APB1Periph_TIM4,  GPIO_AF_TIM4},
		{(uint32_t)GPIOA,  GPIO_Pin_2, RCC_AHB1Periph_GPIOA,  GPIO_PinSource2,  RCC_APB1Periph_TIM5,  GPIO_AF_TIM5},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{(uint32_t)GPIOC,  GPIO_Pin_8, RCC_AHB1Periph_GPIOC,  GPIO_PinSource8,  RCC_APB2Periph_TIM8,  GPIO_AF_TIM8},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0}
	},

	{	{(uint32_t)GPIOA, GPIO_Pin_11, RCC_AHB1Periph_GPIOA, GPIO_PinSource11,  RCC_APB2Periph_TIM1,  GPIO_AF_TIM1},
		{(uint32_t)GPIOA,  GPIO_Pin_3, RCC_AHB1Periph_GPIOA,  GPIO_PinSource3,  RCC_APB1Periph_TIM2,  GPIO_AF_TIM2},
		{(uint32_t)GPIOB,  GPIO_Pin_1, RCC_AHB1Periph_GPIOB,  GPIO_PinSource1,  RCC_APB1Periph_TIM3,  GPIO_AF_TIM3},
		{(uint32_t)GPIOB,  GPIO_Pin_9, RCC_AHB1Periph_GPIOB,  GPIO_PinSource9,  RCC_APB1Periph_TIM4,  GPIO_AF_TIM4},
		{(uint32_t)GPIOA,  GPIO_Pin_3, RCC_AHB1Periph_GPIOA,  GPIO_PinSource3,  RCC_APB1Periph_TIM5,  GPIO_AF_TIM5},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{(uint32_t)GPIOC,  GPIO_Pin_9, RCC_AHB1Periph_GPIOC,  GPIO_PinSource9,  RCC_APB2Periph_TIM8,  GPIO_AF_TIM8},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0}
	}
};

uint32_t PWM_PARAM_MAPPING_PP2[PWM_CHANNEL_MAX][TIMER_NUM_MAX][PWM_PARAM_MAPPING_MAX_INDEX] = {
	{	{(uint32_t)GPIOE,  GPIO_Pin_9, RCC_AHB1Periph_GPIOE,  GPIO_PinSource9,  RCC_APB2Periph_TIM1,  GPIO_AF_TIM1},
		{(uint32_t)GPIOA,  GPIO_Pin_5, RCC_AHB1Periph_GPIOA,  GPIO_PinSource5,  RCC_APB1Periph_TIM2,  GPIO_AF_TIM2},
		{(uint32_t)GPIOB,  GPIO_Pin_4, RCC_AHB1Periph_GPIOB,  GPIO_PinSource4,  RCC_APB1Periph_TIM3,  GPIO_AF_TIM3},
		{(uint32_t)GPIOD, GPIO_Pin_12, RCC_AHB1Periph_GPIOD, GPIO_PinSource12,  RCC_APB1Periph_TIM4,  GPIO_AF_TIM4},
		{(uint32_t)GPIOH, GPIO_Pin_10, RCC_AHB1Periph_GPIOH, GPIO_PinSource10,  RCC_APB1Periph_TIM5,  GPIO_AF_TIM5},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{(uint32_t)GPIOI,  GPIO_Pin_5, RCC_AHB1Periph_GPIOI,  GPIO_PinSource5,  RCC_APB2Periph_TIM8,  GPIO_AF_TIM8},
		{(uint32_t)GPIOE,  GPIO_Pin_5, RCC_AHB1Periph_GPIOE,  GPIO_PinSource5,  RCC_APB2Periph_TIM9,  GPIO_AF_TIM9},
		{(uint32_t)GPIOF,  GPIO_Pin_6, RCC_AHB1Periph_GPIOF,  GPIO_PinSource6, RCC_APB2Periph_TIM10, GPIO_AF_TIM10},
		{(uint32_t)GPIOF,  GPIO_Pin_7, RCC_AHB1Periph_GPIOF,  GPIO_PinSource7, RCC_APB2Periph_TIM11, GPIO_AF_TIM11},
		{(uint32_t)GPIOH,  GPIO_Pin_6, RCC_AHB1Periph_GPIOH,  GPIO_PinSource6, RCC_APB1Periph_TIM12, GPIO_AF_TIM12},
		{(uint32_t)GPIOF,  GPIO_Pin_8, RCC_AHB1Periph_GPIOF,  GPIO_PinSource8, RCC_APB1Periph_TIM13, GPIO_AF_TIM13},
		{(uint32_t)GPIOF,  GPIO_Pin_9, RCC_AHB1Periph_GPIOF,  GPIO_PinSource9, RCC_APB1Periph_TIM14, GPIO_AF_TIM14}
	},

	{	{(uint32_t)GPIOE, GPIO_Pin_10, RCC_AHB1Periph_GPIOE, GPIO_PinSource10,  RCC_APB2Periph_TIM1,  GPIO_AF_TIM1},
		{(uint32_t)GPIOB,  GPIO_Pin_3, RCC_AHB1Periph_GPIOB,  GPIO_PinSource3,  RCC_APB1Periph_TIM2,  GPIO_AF_TIM2},
		{(uint32_t)GPIOB,  GPIO_Pin_5, RCC_AHB1Periph_GPIOB,  GPIO_PinSource5,  RCC_APB1Periph_TIM3,  GPIO_AF_TIM3},
		{(uint32_t)GPIOD, GPIO_Pin_13, RCC_AHB1Periph_GPIOD, GPIO_PinSource13,  RCC_APB1Periph_TIM4,  GPIO_AF_TIM4},
		{(uint32_t)GPIOH, GPIO_Pin_11, RCC_AHB1Periph_GPIOH, GPIO_PinSource11,  RCC_APB1Periph_TIM5,  GPIO_AF_TIM5},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{(uint32_t)GPIOI,  GPIO_Pin_6, RCC_AHB1Periph_GPIOI,  GPIO_PinSource6,  RCC_APB2Periph_TIM8,  GPIO_AF_TIM8},
		{(uint32_t)GPIOE,  GPIO_Pin_6, RCC_AHB1Periph_GPIOE,  GPIO_PinSource6,  RCC_APB2Periph_TIM9,  GPIO_AF_TIM9},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{(uint32_t)GPIOH,  GPIO_Pin_9, RCC_AHB1Periph_GPIOH,  GPIO_PinSource9, RCC_APB1Periph_TIM12, GPIO_AF_TIM12},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0}
	},

	{	{(uint32_t)GPIOE, GPIO_Pin_13, RCC_AHB1Periph_GPIOE, GPIO_PinSource13,  RCC_APB2Periph_TIM1,  GPIO_AF_TIM1},
		{(uint32_t)GPIOB, GPIO_Pin_10, RCC_AHB1Periph_GPIOB, GPIO_PinSource10,  RCC_APB1Periph_TIM2,  GPIO_AF_TIM2},
		{(uint32_t)GPIOC,  GPIO_Pin_8, RCC_AHB1Periph_GPIOC,  GPIO_PinSource8,  RCC_APB1Periph_TIM3,  GPIO_AF_TIM3},
		{(uint32_t)GPIOD, GPIO_Pin_14, RCC_AHB1Periph_GPIOD, GPIO_PinSource14,  RCC_APB1Periph_TIM4,  GPIO_AF_TIM4},
		{(uint32_t)GPIOH, GPIO_Pin_12, RCC_AHB1Periph_GPIOH, GPIO_PinSource12,  RCC_APB1Periph_TIM5,  GPIO_AF_TIM5},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{(uint32_t)GPIOI,  GPIO_Pin_7, RCC_AHB1Periph_GPIOI,  GPIO_PinSource7,  RCC_APB2Periph_TIM8,  GPIO_AF_TIM8},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0}
	},

	{	{(uint32_t)GPIOE, GPIO_Pin_14, RCC_AHB1Periph_GPIOE, GPIO_PinSource14,  RCC_APB2Periph_TIM1,  GPIO_AF_TIM1},
		{(uint32_t)GPIOB, GPIO_Pin_11, RCC_AHB1Periph_GPIOB, GPIO_PinSource11,  RCC_APB1Periph_TIM2,  GPIO_AF_TIM2},
		{(uint32_t)GPIOC,  GPIO_Pin_9, RCC_AHB1Periph_GPIOC,  GPIO_PinSource9,  RCC_APB1Periph_TIM3,  GPIO_AF_TIM3},
		{(uint32_t)GPIOD, GPIO_Pin_15, RCC_AHB1Periph_GPIOD, GPIO_PinSource15,  RCC_APB1Periph_TIM4,  GPIO_AF_TIM4},
		{(uint32_t)GPIOI,  GPIO_Pin_0, RCC_AHB1Periph_GPIOI,  GPIO_PinSource0,  RCC_APB1Periph_TIM5,  GPIO_AF_TIM5},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{(uint32_t)GPIOI,  GPIO_Pin_2, RCC_AHB1Periph_GPIOI,  GPIO_PinSource2,  RCC_APB2Periph_TIM8,  GPIO_AF_TIM8},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0}
	}
};

uint32_t PWM_PARAM_MAPPING_PP3[PWM_CHANNEL_MAX][TIMER_NUM_MAX][PWM_PARAM_MAPPING_MAX_INDEX] = {
	{	{              0,           0,                    0,                0,                    0,             0},
		{(uint32_t)GPIOA, GPIO_Pin_15, RCC_AHB1Periph_GPIOA, GPIO_PinSource15,  RCC_APB1Periph_TIM2,  GPIO_AF_TIM2},
		{(uint32_t)GPIOC,  GPIO_Pin_6, RCC_AHB1Periph_GPIOC,  GPIO_PinSource6,  RCC_APB1Periph_TIM3,  GPIO_AF_TIM3},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0}
	},

	{	{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{(uint32_t)GPIOC,  GPIO_Pin_7, RCC_AHB1Periph_GPIOC,  GPIO_PinSource7,  RCC_APB1Periph_TIM3,  GPIO_AF_TIM3},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0}
	},

	{	{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0}
	},

	{	{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0},
		{              0,           0,                    0,                0,                    0,             0}
	},
};

/*
 * TIMx map
 */

TIM_TypeDef *TIMx_MAPPING[TIMER_NUM_MAX] = {
	TIM1,
	TIM2,
	TIM3,
	TIM4,
	TIM5,
	TIM6,
	TIM7,
	TIM8,
	TIM9,
	TIM10,
	TIM11,
	TIM12,
	TIM13,
	TIM14
};

uint32_t PWM_CLOCK_SOURCE_MAPPING[TIMER_NUM_MAX] = {
	APB2_CLOCK,
	APB1_CLOCK,
	APB1_CLOCK,
	APB1_CLOCK,
	APB1_CLOCK,
	APB1_CLOCK,
	APB1_CLOCK,
	APB1_CLOCK,
	APB2_CLOCK,
	APB2_CLOCK,
	APB2_CLOCK,
	APB1_CLOCK,
	APB1_CLOCK,
	APB1_CLOCK
};



/* Internal function ---------------------------------------------------------*/


/* External function ---------------------------------------------------------*/
pwm_handle_t pwm_init(pwm_config_t *config)
{
	/*Mapping implement */
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin_x;
	uint32_t RCC_AHBxPeriph_GPIOx;
	uint8_t GPIO_PinSourcex;
	uint32_t RCC_APBxPeriph_TIMx;
	uint8_t GPIO_AF_TIMx;
	TIM_TypeDef *TIMx;

	if (config->pwm_pins_pack == PWM_PINS_PACK_1)
	{
		GPIOx                = (GPIO_TypeDef *)PWM_PARAM_MAPPING_PP1[config->pwm_channel][config->timer][PWM_PARAM_MAPPING_GPIOx];
		GPIO_Pin_x           = (uint16_t)      PWM_PARAM_MAPPING_PP1[config->pwm_channel][config->timer][PWM_PARAM_MAPPING_GPIO_Pin_x];
		RCC_AHBxPeriph_GPIOx = (uint32_t)      PWM_PARAM_MAPPING_PP1[config->pwm_channel][config->timer][PWM_PARAM_MAPPING_RCC_AHBxPeriph_GPIOx];
		GPIO_PinSourcex      = (uint8_t)       PWM_PARAM_MAPPING_PP1[config->pwm_channel][config->timer][PWM_PARAM_MAPPING_PinSourcex];
		RCC_APBxPeriph_TIMx  = (uint32_t)      PWM_PARAM_MAPPING_PP1[config->pwm_channel][config->timer][PWM_PARAM_MAPPING_RCC_APBxPeriph_TIMx];
		GPIO_AF_TIMx         = (uint8_t)       PWM_PARAM_MAPPING_PP1[config->pwm_channel][config->timer][PWM_PARAM_MAPPING_GPIO_AF_TIMx];
	}

	if (config->pwm_pins_pack == PWM_PINS_PACK_2)
	{
		GPIOx                = (GPIO_TypeDef *)PWM_PARAM_MAPPING_PP2[config->pwm_channel][config->timer][PWM_PARAM_MAPPING_GPIOx];
		GPIO_Pin_x           = (uint16_t)      PWM_PARAM_MAPPING_PP2[config->pwm_channel][config->timer][PWM_PARAM_MAPPING_GPIO_Pin_x];
		RCC_AHBxPeriph_GPIOx = (uint32_t)      PWM_PARAM_MAPPING_PP2[config->pwm_channel][config->timer][PWM_PARAM_MAPPING_RCC_AHBxPeriph_GPIOx];
		GPIO_PinSourcex      = (uint8_t)       PWM_PARAM_MAPPING_PP2[config->pwm_channel][config->timer][PWM_PARAM_MAPPING_PinSourcex];
		RCC_APBxPeriph_TIMx  = (uint32_t)      PWM_PARAM_MAPPING_PP2[config->pwm_channel][config->timer][PWM_PARAM_MAPPING_RCC_APBxPeriph_TIMx];
		GPIO_AF_TIMx         = (uint8_t)       PWM_PARAM_MAPPING_PP2[config->pwm_channel][config->timer][PWM_PARAM_MAPPING_GPIO_AF_TIMx];
	}

	if (config->pwm_pins_pack == PWM_PINS_PACK_3)
	{
		GPIOx                = (GPIO_TypeDef *)PWM_PARAM_MAPPING_PP3[config->pwm_channel][config->timer][PWM_PARAM_MAPPING_GPIOx];
		GPIO_Pin_x           = (uint16_t)      PWM_PARAM_MAPPING_PP3[config->pwm_channel][config->timer][PWM_PARAM_MAPPING_GPIO_Pin_x];
		RCC_AHBxPeriph_GPIOx = (uint32_t)      PWM_PARAM_MAPPING_PP3[config->pwm_channel][config->timer][PWM_PARAM_MAPPING_RCC_AHBxPeriph_GPIOx];
		GPIO_PinSourcex      = (uint8_t)       PWM_PARAM_MAPPING_PP3[config->pwm_channel][config->timer][PWM_PARAM_MAPPING_PinSourcex];
		RCC_APBxPeriph_TIMx  = (uint32_t)      PWM_PARAM_MAPPING_PP3[config->pwm_channel][config->timer][PWM_PARAM_MAPPING_RCC_APBxPeriph_TIMx];
		GPIO_AF_TIMx         = (uint8_t)       PWM_PARAM_MAPPING_PP3[config->pwm_channel][config->timer][PWM_PARAM_MAPPING_GPIO_AF_TIMx];
	}

	TIMx = TIMx_MAPPING[config->timer];

	/* Enable Timer clock source */
	if ((config->timer == TIMER_NUM_1) || (config->timer == TIMER_NUM_8) || (config->timer == TIMER_NUM_9) || (config->timer == TIMER_NUM_10) || (config->timer == TIMER_NUM_11))
	{
		RCC_APB2PeriphClockCmd(RCC_APBxPeriph_TIMx, ENABLE);
	}
	else
	{
		RCC_APB1PeriphClockCmd(RCC_APBxPeriph_TIMx, ENABLE);
	}

	/* Enable GPIO clock source */
	RCC_AHB1PeriphClockCmd(RCC_AHBxPeriph_GPIOx, ENABLE);

	/*GPIO configuration */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_x;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_FREQ;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD ;
	GPIO_Init(GPIOx, &GPIO_InitStructure);

	/* Connect TIMx pin to AFx */
	GPIO_PinAFConfig(GPIOx, GPIO_PinSourcex, GPIO_AF_TIMx);

	uint16_t CCR_Val = (uint16_t)((config->pwm_duty) * (config->timer_period) / 100);

	/* Time base configuration */
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = config->timer_period;
	TIM_TimeBaseStructure.TIM_Prescaler = config->timer_prescaler;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

	/* PWM Mode configuration */
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	switch (config->pwm_channel) {
	case PWM_CHANNEL_1: {
		TIM_OC1Init(TIMx, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
		break;
	}
	case PWM_CHANNEL_2: {
		TIM_OC2Init(TIMx, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
		break;
	}
	case PWM_CHANNEL_3: {
		TIM_OC3Init(TIMx, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
		break;
	}
	case PWM_CHANNEL_4: {
		TIM_OC4Init(TIMx, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);
		break;
	}
	default:
		break;
	}

	TIM_ARRPreloadConfig(TIMx, ENABLE);

	pwm_handle_t handle = calloc(1, sizeof(pwm_param_t));
	if(!handle)
    {
        return -1;
    }

	handle->timer           = config->timer;
	handle->timer_period    = config->timer_period;
	handle->timer_prescaler = config->timer_prescaler;
	handle->pwm_channel     = config->pwm_channel;
	handle->pwm_pins_pack   = config->pwm_pins_pack;
	handle->pwm_duty        = config->pwm_duty;
	return handle;
}

int pwm_start(pwm_handle_t handle)
{
	/* TIM enable counter */
	TIM_Cmd(TIMx_MAPPING[handle->timer], ENABLE);

	return 0;
}

int pwm_set_timer_prescaler(pwm_handle_t handle, uint16_t timer_prescaler)
{
	assert_param(IS_TIM_ALL_PERIPH(TIMx));
	assert_param(IS_TIM_PRESCALER_RELOAD(TIM_PSCReloadMode));
	TIMx_MAPPING[handle->timer]->PSC = timer_prescaler;
	TIMx_MAPPING[handle->timer]->EGR = TIM_PSCReloadMode_Immediate;

	handle->timer_prescaler = timer_prescaler;
	return 0;
}

int pwm_set_timer_period(pwm_handle_t handle, uint32_t timer_period)
{
	assert_param(IS_TIM_ALL_PERIPH(TIMx_MAPPING[handle->timer]));
	TIMx_MAPPING[handle->timer]->ARR = timer_period;
	TIMx_MAPPING[handle->timer]->EGR = TIM_PSCReloadMode_Immediate;

	assert_param(IS_TIM_ALL_PERIPH(TIMx_MAPPING[handle->timer]));
	TIMx_MAPPING[handle->timer]->CCR1 = (handle->pwm_duty) * timer_period / 100;
	TIMx_MAPPING[handle->timer]->EGR = TIM_PSCReloadMode_Immediate;

	handle->timer_period = timer_period;
	return 0;
}

int pwm_set_freq(pwm_handle_t handle, uint32_t freq_hz)
{
//	uint32_t conduct = (uint32_t)

	return 0;
}

int pwm_set_duty(pwm_handle_t handle, uint8_t pwm_duty)
{
	assert_param(IS_TIM_ALL_PERIPH(TIMx_MAPPING[handle->timer]));
	TIMx_MAPPING[handle->timer]->CCR1 = pwm_duty * (handle->timer_period) / 100;
	TIMx_MAPPING[handle->timer]->EGR = TIM_PSCReloadMode_Immediate;

	handle->pwm_duty = pwm_duty;
	return 0;
}

int pwm_stop(pwm_handle_t handle)
{
	/* TIM disable counter */
	TIM_Cmd(TIMx_MAPPING[handle->timer], DISABLE);

	return 0;
}

int pwm_deinit(pwm_handle_t handle)
{
	TIM_DeInit(TIMx_MAPPING[handle->timer]);
	free(handle);

	return 0;
}



