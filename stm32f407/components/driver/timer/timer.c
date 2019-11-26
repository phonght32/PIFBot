#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"

#include "include/timer.h"

#include "stdint.h"

#define GPIO_SPEED_FREQ	GPIO_Speed_100MHz
#define GPIO_PUPD 		GPIO_PuPd_UP


static uint32_t RCC_APBxPeriph_TIM_MAPPING[14] = {
	RCC_APB2Periph_TIM1,
	RCC_APB1Periph_TIM2,
	RCC_APB1Periph_TIM3,
	RCC_APB1Periph_TIM4,
	RCC_APB1Periph_TIM5,
	RCC_APB1Periph_TIM6,
	RCC_APB1Periph_TIM7,
	RCC_APB2Periph_TIM8,
	RCC_APB2Periph_TIM9,
	RCC_APB2Periph_TIM10,
	RCC_APB2Periph_TIM11,
	RCC_APB1Periph_TIM12,
	RCC_APB1Periph_TIM13,
	RCC_APB1Periph_TIM14
};


typedef struct
{
	TIM_TypeDef *TIMx;
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_PIN_x;
} pwm_config_t;

int pwm_init(pwm_config_t *config)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Timer clock enable */
	if ((config->TIMx = TIM1) || (config->TIMx == TIM8) || (config->TIMx == TIM9) || (config->TIMx == TIM10) || (config->TIMx == TIM11))
	{
		RCC_APB2PeriphClockCmd(config->TIMx, ENABLE);
	}
	else
	{
		RCC_APP1PeriphClockCmd(config->TIMx, ENABLE);
	}


	/* GPIO clock enable */
	RCC_AHB1PeriphClockCmd(config->GPIOx, ENABLE);

	/* GPIO Configuration */
	GPIO_InitStructure.GPIO_Pin = config->GPIO_PIN_x;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_FREQ;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD ;
	GPIO_Init(config->GPIOx, &GPIO_InitStructure); 

	GPIO_PinAFConfig(config->GPIOx, GPIO_PinSource6, GPIO_AF_TIM3);
	return 0;
}
