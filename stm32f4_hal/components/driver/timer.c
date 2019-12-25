#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f407xx.h"

#include "stdlib.h"

#include "include/timer.h"

#define SYSTEM_CLOCK					168000000
#define APB1_CLOCK						(SYSTEM_CLOCK/2)
#define APB2_CLOCK 						(SYSTEM_CLOCK)

#define TIMER_MAX_RELOAD				0xFFFF

#define PWM_COUNTERMODE_DEFAULT			TIM_COUNTERMODE_UP
#define PWM_TIM_CLOCK_DIV_DEFAULT 		TIM_CLOCKDIVISION_DIV1

typedef struct timer {
	timer_num_t 		timer_num;
	timer_channel_t 	timer_channel;
	timer_pins_pack_t 	timer_pins_pack;
	uint32_t 			pwm_freq_hz;
	uint8_t				pwm_duty_percent;
	TIM_HandleTypeDef 	hal_handle;
} timer_t;

typedef enum {
	TIMER_PARAM_MAPPING_RCC_AHB1ENR_GPIOxEN = 0,
	TIMER_PARAM_MAPPING_GPIO_PIN_x,
	TIMER_PARAM_MAPPING_GPIO_AFx_TIMx,
	TIMER_PARAM_MAPPING_GPIOx,
	TIMER_PARAM_INDEX_MAX
} timer_param_mapping_t;

uint32_t TIMER_PARAM_MAPPING_PP1[TIMER_CHANNEL_MAX][TIMER_NUM_MAX][TIMER_PARAM_INDEX_MAX] = {
	{	{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_8 , GPIO_AF1_TIM1 , (uint32_t)GPIOA},
		{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_0 , GPIO_AF1_TIM2 , (uint32_t)GPIOA},
		{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_6 , GPIO_AF2_TIM3 , (uint32_t)GPIOA},
		{RCC_AHB1ENR_GPIOBEN, GPIO_PIN_6 , GPIO_AF2_TIM4 , (uint32_t)GPIOB},
		{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_0 , GPIO_AF2_TIM5 , (uint32_t)GPIOA},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{RCC_AHB1ENR_GPIOCEN, GPIO_PIN_6 , GPIO_AF3_TIM8 , (uint32_t)GPIOC},
		{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_2 , GPIO_AF3_TIM9 , (uint32_t)GPIOA},
		{RCC_AHB1ENR_GPIOBEN, GPIO_PIN_8 , GPIO_AF3_TIM10, (uint32_t)GPIOB},
		{RCC_AHB1ENR_GPIOBEN, GPIO_PIN_9 , GPIO_AF3_TIM11, (uint32_t)GPIOB},
		{RCC_AHB1ENR_GPIOBEN, GPIO_PIN_14, GPIO_AF9_TIM12, (uint32_t)GPIOB},
		{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_6 , GPIO_AF9_TIM13, (uint32_t)GPIOA},
		{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_7 , GPIO_AF9_TIM14, (uint32_t)GPIOA},
	},

	{	{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_9 , GPIO_AF1_TIM1 , (uint32_t)GPIOA},
		{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_1 , GPIO_AF1_TIM2 , (uint32_t)GPIOA},
		{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_7 , GPIO_AF2_TIM3 , (uint32_t)GPIOA},
		{RCC_AHB1ENR_GPIOBEN, GPIO_PIN_7 , GPIO_AF2_TIM4 , (uint32_t)GPIOB},
		{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_1 , GPIO_AF2_TIM5 , (uint32_t)GPIOA},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{RCC_AHB1ENR_GPIOCEN, GPIO_PIN_7 , GPIO_AF3_TIM8 , (uint32_t)GPIOC},
		{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_3 , GPIO_AF3_TIM9 , (uint32_t)GPIOA},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{RCC_AHB1ENR_GPIOBEN, GPIO_PIN_15, GPIO_AF9_TIM12, (uint32_t)GPIOB},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
	},

	{	{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_10, GPIO_AF1_TIM1 , (uint32_t)GPIOA},
		{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_2 , GPIO_AF1_TIM2 , (uint32_t)GPIOA},
		{RCC_AHB1ENR_GPIOBEN, GPIO_PIN_0 , GPIO_AF2_TIM3 , (uint32_t)GPIOB},
		{RCC_AHB1ENR_GPIOBEN, GPIO_PIN_8 , GPIO_AF2_TIM4 , (uint32_t)GPIOB},
		{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_2 , GPIO_AF2_TIM5 , (uint32_t)GPIOA},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{RCC_AHB1ENR_GPIOCEN, GPIO_PIN_8 , GPIO_AF3_TIM8 , (uint32_t)GPIOC},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
	},

	{	{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_11, GPIO_AF1_TIM1 , (uint32_t)GPIOA},
		{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_3 , GPIO_AF1_TIM2 , (uint32_t)GPIOA},
		{RCC_AHB1ENR_GPIOBEN, GPIO_PIN_1 , GPIO_AF2_TIM3 , (uint32_t)GPIOB},
		{RCC_AHB1ENR_GPIOBEN, GPIO_PIN_9 , GPIO_AF2_TIM4 , (uint32_t)GPIOB},
		{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_3 , GPIO_AF2_TIM5 , (uint32_t)GPIOA},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{RCC_AHB1ENR_GPIOCEN, GPIO_PIN_9 , GPIO_AF3_TIM8 , (uint32_t)GPIOC},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
	}
};

uint32_t TIMER_PARAM_MAPPING_PP2[TIMER_CHANNEL_MAX][TIMER_NUM_MAX][TIMER_PARAM_INDEX_MAX] = {
	{	{RCC_AHB1ENR_GPIOEEN, GPIO_PIN_9 , GPIO_AF1_TIM1 , (uint32_t)GPIOE},
		{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_5 , GPIO_AF1_TIM2 , (uint32_t)GPIOA},
		{RCC_AHB1ENR_GPIOBEN, GPIO_PIN_4 , GPIO_AF2_TIM3 , (uint32_t)GPIOB},
		{RCC_AHB1ENR_GPIODEN, GPIO_PIN_12, GPIO_AF2_TIM4 , (uint32_t)GPIOD},
		{RCC_AHB1ENR_GPIOHEN, GPIO_PIN_10, GPIO_AF2_TIM5 , (uint32_t)GPIOH},
		{                  0,           0,             0 ,               0},
		{                  0,           0,             0 ,               0},
		{RCC_AHB1ENR_GPIOIEN, GPIO_PIN_5 , GPIO_AF3_TIM8 , (uint32_t)GPIOI},
		{RCC_AHB1ENR_GPIOEEN, GPIO_PIN_5 , GPIO_AF3_TIM9 , (uint32_t)GPIOE},
		{RCC_AHB1ENR_GPIOFEN, GPIO_PIN_6 , GPIO_AF3_TIM10, (uint32_t)GPIOF},
		{RCC_AHB1ENR_GPIOFEN, GPIO_PIN_7 , GPIO_AF3_TIM11, (uint32_t)GPIOF},
		{RCC_AHB1ENR_GPIOHEN, GPIO_PIN_6 , GPIO_AF9_TIM12, (uint32_t)GPIOH},
		{RCC_AHB1ENR_GPIOFEN, GPIO_PIN_8 , GPIO_AF9_TIM13, (uint32_t)GPIOF},
		{RCC_AHB1ENR_GPIOFEN, GPIO_PIN_9 , GPIO_AF9_TIM14, (uint32_t)GPIOF},
	},

	{	{RCC_AHB1ENR_GPIOEEN, GPIO_PIN_10, GPIO_AF1_TIM1 , (uint32_t)GPIOE},
		{RCC_AHB1ENR_GPIOBEN, GPIO_PIN_3 , GPIO_AF1_TIM2 , (uint32_t)GPIOB},
		{RCC_AHB1ENR_GPIOBEN, GPIO_PIN_5 , GPIO_AF2_TIM3 , (uint32_t)GPIOB},
		{RCC_AHB1ENR_GPIODEN, GPIO_PIN_13, GPIO_AF2_TIM4 , (uint32_t)GPIOD},
		{RCC_AHB1ENR_GPIOHEN, GPIO_PIN_11, GPIO_AF2_TIM5 , (uint32_t)GPIOH},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{RCC_AHB1ENR_GPIOIEN, GPIO_PIN_6 , GPIO_AF3_TIM8 , (uint32_t)GPIOI},
		{RCC_AHB1ENR_GPIOEEN, GPIO_PIN_6 , GPIO_AF3_TIM9 , (uint32_t)GPIOE},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{RCC_AHB1ENR_GPIOHEN, GPIO_PIN_9 , GPIO_AF9_TIM12, (uint32_t)GPIOH},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
	},

	{	{RCC_AHB1ENR_GPIOEEN, GPIO_PIN_13, GPIO_AF1_TIM1 , (uint32_t)GPIOE},
		{RCC_AHB1ENR_GPIOBEN, GPIO_PIN_10, GPIO_AF1_TIM2 , (uint32_t)GPIOB},
		{RCC_AHB1ENR_GPIOCEN, GPIO_PIN_8 , GPIO_AF2_TIM3 , (uint32_t)GPIOC},
		{RCC_AHB1ENR_GPIODEN, GPIO_PIN_14, GPIO_AF2_TIM4 , (uint32_t)GPIOD},
		{RCC_AHB1ENR_GPIOHEN, GPIO_PIN_12, GPIO_AF2_TIM5 , (uint32_t)GPIOH},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{RCC_AHB1ENR_GPIOIEN, GPIO_PIN_7 , GPIO_AF3_TIM8 , (uint32_t)GPIOI},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
	},

	{	{RCC_AHB1ENR_GPIOEEN, GPIO_PIN_14, GPIO_AF1_TIM1 , (uint32_t)GPIOE},
		{RCC_AHB1ENR_GPIOBEN, GPIO_PIN_11, GPIO_AF1_TIM2 , (uint32_t)GPIOB},
		{RCC_AHB1ENR_GPIOCEN, GPIO_PIN_9 , GPIO_AF2_TIM3 , (uint32_t)GPIOC},
		{RCC_AHB1ENR_GPIODEN, GPIO_PIN_15, GPIO_AF2_TIM4 , (uint32_t)GPIOD},
		{RCC_AHB1ENR_GPIOIEN, GPIO_PIN_0 , GPIO_AF2_TIM5 , (uint32_t)GPIOI},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{RCC_AHB1ENR_GPIOIEN, GPIO_PIN_2 , GPIO_AF3_TIM8 , (uint32_t)GPIOI},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
	}
};

uint32_t TIMER_PARAM_MAPPING_PP3[TIMER_CHANNEL_MAX][TIMER_NUM_MAX][TIMER_PARAM_INDEX_MAX] = {
	{	{                  0,          0 ,             0 ,               0},
		{RCC_AHB1ENR_GPIOAEN, GPIO_PIN_15, GPIO_AF1_TIM1 , (uint32_t)GPIOA},
		{RCC_AHB1ENR_GPIOCEN, GPIO_PIN_6 , GPIO_AF1_TIM2 , (uint32_t)GPIOC},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0}
	},

	{	{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{RCC_AHB1ENR_GPIOCEN, GPIO_PIN_7 , GPIO_AF2_TIM3 , (uint32_t)GPIOC},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0}
	},

	{	{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0}
	},

	{	{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0},
		{                  0,          0 ,             0 ,               0}
	}
};

uint32_t RCC_APBxENR_TIMxEN_MAPPING[TIMER_NUM_MAX] = {
	RCC_APB2ENR_TIM1EN,
	RCC_APB1ENR_TIM2EN,
	RCC_APB1ENR_TIM3EN,
	RCC_APB1ENR_TIM4EN,
	RCC_APB1ENR_TIM5EN,
	RCC_APB1ENR_TIM6EN,
	RCC_APB1ENR_TIM7EN,
	RCC_APB2ENR_TIM8EN,
	RCC_APB2ENR_TIM9EN,
	RCC_APB2ENR_TIM10EN,
	RCC_APB2ENR_TIM11EN,
	RCC_APB1ENR_TIM12EN,
	RCC_APB1ENR_TIM13EN,
	RCC_APB1ENR_TIM14EN,
};

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
	TIM14,
};

uint32_t TIM_CHANNEL_x_MAPPING[TIMER_CHANNEL_MAX] = {
	TIM_CHANNEL_1,
	TIM_CHANNEL_2,
	TIM_CHANNEL_3,
	TIM_CHANNEL_4
};

uint32_t APBx_CLOCK_MAPPING[TIMER_NUM_MAX] = {
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

static int pwm_cleanup(pwm_handle_t handle)
{
	free(handle);

	return 0;
}

pwm_handle_t pwm_init(pwm_config_t *config)
{
	pwm_handle_t handle = calloc(1, sizeof(timer_t));
	if (handle == NULL)
	{
		return -1;
	}

	int err;

	uint32_t RCC_APBxENR_GPIOxEN;
	uint16_t GPIO_PIN_x;
	uint8_t GPIO_AFx_TIMx;
	GPIO_TypeDef *GPIOx;
	TIM_TypeDef *TIMx;
	uint32_t TIM_CHANNEL_x;

	if (config->timer_pins_pack == TIMER_PINS_PACK_1)
	{
		RCC_APBxENR_GPIOxEN = (uint32_t)      TIMER_PARAM_MAPPING_PP1[config->timer_channel][config->timer_num][TIMER_PARAM_MAPPING_RCC_AHB1ENR_GPIOxEN];
		GPIO_PIN_x          = (uint16_t)      TIMER_PARAM_MAPPING_PP1[config->timer_channel][config->timer_num][TIMER_PARAM_MAPPING_GPIO_PIN_x];
		GPIO_AFx_TIMx       = (uint8_t)       TIMER_PARAM_MAPPING_PP1[config->timer_channel][config->timer_num][TIMER_PARAM_MAPPING_GPIO_AFx_TIMx];
		GPIOx               = (GPIO_TypeDef *)TIMER_PARAM_MAPPING_PP1[config->timer_channel][config->timer_num][TIMER_PARAM_MAPPING_GPIOx];
	}
	if (config->timer_pins_pack == TIMER_PINS_PACK_2)
	{
		RCC_APBxENR_GPIOxEN = (uint32_t)      TIMER_PARAM_MAPPING_PP2[config->timer_channel][config->timer_num][TIMER_PARAM_MAPPING_RCC_AHB1ENR_GPIOxEN];
		GPIO_PIN_x          = (uint16_t)      TIMER_PARAM_MAPPING_PP2[config->timer_channel][config->timer_num][TIMER_PARAM_MAPPING_GPIO_PIN_x];
		GPIO_AFx_TIMx       = (uint8_t)       TIMER_PARAM_MAPPING_PP2[config->timer_channel][config->timer_num][TIMER_PARAM_MAPPING_GPIO_AFx_TIMx];
		GPIOx               = (GPIO_TypeDef *)TIMER_PARAM_MAPPING_PP2[config->timer_channel][config->timer_num][TIMER_PARAM_MAPPING_GPIOx];
	}
	if (config->timer_pins_pack == TIMER_PINS_PACK_3)
	{
		RCC_APBxENR_GPIOxEN = (uint32_t)      TIMER_PARAM_MAPPING_PP3[config->timer_channel][config->timer_num][TIMER_PARAM_MAPPING_RCC_AHB1ENR_GPIOxEN];
		GPIO_PIN_x          = (uint16_t)      TIMER_PARAM_MAPPING_PP3[config->timer_channel][config->timer_num][TIMER_PARAM_MAPPING_GPIO_PIN_x];
		GPIO_AFx_TIMx       = (uint8_t)       TIMER_PARAM_MAPPING_PP3[config->timer_channel][config->timer_num][TIMER_PARAM_MAPPING_GPIO_AFx_TIMx];
		GPIOx               = (GPIO_TypeDef *)TIMER_PARAM_MAPPING_PP3[config->timer_channel][config->timer_num][TIMER_PARAM_MAPPING_GPIOx];
	}

	TIMx = TIMx_MAPPING[config->timer_num];
	TIM_CHANNEL_x = TIM_CHANNEL_x_MAPPING[config->timer_channel];

	do {
		__IO uint32_t tmpreg = 0x00U;
		SET_BIT(RCC->AHB1ENR, RCC_APBxENR_GPIOxEN);
		tmpreg = READ_BIT(RCC->AHB1ENR, RCC_APBxENR_GPIOxEN);
		UNUSED(tmpreg);
	} while (0U);


	do {
		__IO uint32_t tmpreg = 0x00U;
		if ((config->timer_num == TIMER_NUM_1) || (config->timer_num == TIMER_NUM_8) || (config->timer_num == TIMER_NUM_9) || (config->timer_num == TIMER_NUM_10) || (config->timer_num == TIMER_NUM_11)) {
			SET_BIT(RCC->APB2ENR, RCC_APBxENR_TIMxEN_MAPPING[config->timer_num]);
			tmpreg = READ_BIT(RCC->APB2ENR, RCC_APBxENR_TIMxEN_MAPPING[config->timer_num]);
		}
		else {
			SET_BIT(RCC->APB1ENR, RCC_APBxENR_TIMxEN_MAPPING[config->timer_num]);
			tmpreg = READ_BIT(RCC->APB1ENR, RCC_APBxENR_TIMxEN_MAPPING[config->timer_num]);
		}
		UNUSED(tmpreg);
	} while (0U);

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_x;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AFx_TIMx;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

	handle->hal_handle.Instance 				= TIMx;
	handle->hal_handle.Init.Prescaler 			= 0;
	handle->hal_handle.Init.CounterMode 		= PWM_COUNTERMODE_DEFAULT;
	handle->hal_handle.Init.Period 				= 0;
	handle->hal_handle.Init.ClockDivision 		= PWM_TIM_CLOCK_DIV_DEFAULT;
	handle->hal_handle.Init.AutoReloadPreload 	= TIM_AUTORELOAD_PRELOAD_DISABLE;
	err = HAL_TIM_Base_Init(&handle->hal_handle);
	if(err != HAL_OK)
	{
		pwm_cleanup(handle);
		return -1;
	}
	err = HAL_TIM_PWM_Init(&handle->hal_handle);
	if(err != HAL_OK)
	{
		pwm_cleanup(handle);
		return -1;
	}

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	err = HAL_TIM_ConfigClockSource(&handle->hal_handle, &sClockSourceConfig);
	if(err != HAL_OK)
	{
		pwm_cleanup(handle);
		return -1;
	}

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	err = HAL_TIMEx_MasterConfigSynchronization(&handle->hal_handle, &sMasterConfig);
	if(err != HAL_OK)
	{
		pwm_cleanup(handle);
		return -1;
	}

	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	err = HAL_TIM_PWM_ConfigChannel(&handle->hal_handle, &sConfigOC, TIM_CHANNEL_x);
	if(err != HAL_OK)
	{
		pwm_cleanup(handle);
		return -1;
	}

	handle->timer_channel 	 = config->timer_channel;
	handle->timer_num 		 = config->timer_num;
	handle->timer_pins_pack  = config->timer_pins_pack;
	handle->pwm_freq_hz 	 = config->pwm_freq_hz;
	handle->pwm_duty_percent = config->pwm_duty_percent;

	return handle;
}

int pwm_deinit(pwm_handle_t handle)
{
	pwm_cleanup(handle);

	return 0;
}

int pwm_start(pwm_handle_t handle)
{
	HAL_TIM_PWM_Start(&handle->hal_handle, TIM_CHANNEL_x_MAPPING[handle->timer_channel]);

	return 0;
}

int pwm_stop(pwm_handle_t handle)
{
	HAL_TIM_PWM_Stop(&handle->hal_handle, TIM_CHANNEL_x_MAPPING[handle->timer_channel]);

	return 0;
}

int pwm_set_freq(pwm_handle_t handle, uint32_t freq_hz)
{
	uint32_t conduct = (uint32_t) (APBx_CLOCK_MAPPING[handle->timer_num]/freq_hz);
	uint16_t timer_prescaler = conduct / TIMER_MAX_RELOAD + 1;
	uint16_t timer_period = (uint16_t)(conduct /(timer_prescaler+1)) -1;
	uint32_t timer_compare_value = handle->pwm_duty_percent*timer_period/100;

	__HAL_TIM_SET_AUTORELOAD(&handle->hal_handle,timer_period);
	__HAL_TIM_SET_PRESCALER(&handle->hal_handle,timer_prescaler);
	__HAL_TIM_SET_COMPARE(&handle->hal_handle,TIM_CHANNEL_x_MAPPING[handle->timer_channel], timer_compare_value);
	
	handle->pwm_freq_hz = freq_hz;
	return 0;
}
int pwm_set_duty(pwm_handle_t handle, uint8_t duty_percent)
{
	uint32_t compare_value;
	compare_value = duty_percent*(handle->hal_handle.Instance->ARR) /100;
	__HAL_TIM_SET_COMPARE(&handle->hal_handle, TIM_CHANNEL_x_MAPPING[handle->timer_channel], compare_value);

	handle->pwm_duty_percent = duty_percent;
	return 0;
}




