#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_tim.h"

#include "stdlib.h"

#include "include/timer.h"

#define SYSTEM_CLOCK					168000000
#define APB1_CLOCK						(SYSTEM_CLOCK/2)
#define APB2_CLOCK 						(SYSTEM_CLOCK)

#define TIMER_MAX_RELOAD				0xFFFF

#define PWM_COUNTERMODE_DEFAULT			TIM_COUNTERMODE_UP
#define PWM_TIM_CLOCK_DIV_DEFAULT 		TIM_CLOCKDIVISION_DIV1

#define TIM1_PWM_CH1_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM1EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM1, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_8, 						\
								 	 .alternate_func = GPIO_AF1_TIM1}

#define TIM1_PWM_CH1_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM1EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOEEN, 	\
								 	 .timer = TIM1, 							\
								 	 .port = GPIOE, 							\
								 	 .pin = GPIO_PIN_9, 						\
								 	 .alternate_func = GPIO_AF1_TIM1}

#define TIM1_PWM_CH2_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM1EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM1, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_9, 						\
								 	 .alternate_func = GPIO_AF1_TIM1}

#define TIM1_PWM_CH2_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM1EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOEEN, 	\
								 	 .timer = TIM1, 							\
								 	 .port = GPIOE, 							\
								 	 .pin = GPIO_PIN_10, 						\
								 	 .alternate_func = GPIO_AF1_TIM1}

#define TIM1_PWM_CH3_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM1EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM1, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_10, 						\
								 	 .alternate_func = GPIO_AF1_TIM1}

#define TIM1_PWM_CH3_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM1EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOEEN, 	\
								 	 .timer = TIM1, 							\
								 	 .port = GPIOE, 							\
								 	 .pin = GPIO_PIN_13, 						\
								 	 .alternate_func = GPIO_AF1_TIM1}

#define TIM1_PWM_CH4_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM1EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM1, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_11, 						\
								 	 .alternate_func = GPIO_AF1_TIM1}

#define TIM1_PWM_CH4_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM1EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOEEN, 	\
								 	 .timer = TIM1, 							\
								 	 .port = GPIOE, 							\
								 	 .pin = GPIO_PIN_14, 						\
								 	 .alternate_func = GPIO_AF1_TIM1}

#define TIM2_PWM_CH1_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM2, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_0, 						\
								 	 .alternate_func = GPIO_AF1_TIM2}

#define TIM2_PWM_CH1_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM2, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_5, 						\
								 	 .alternate_func = GPIO_AF1_TIM2}
							 
#define TIM2_PWM_CH1_PP3_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM2, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_15, 						\
								 	 .alternate_func = GPIO_AF1_TIM2}

#define TIM2_PWM_CH2_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM2, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_1, 						\
								 	 .alternate_func = GPIO_AF1_TIM2}

#define TIM2_PWM_CH2_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN, 	\
								 	 .timer = TIM2, 							\
								 	 .port = GPIOB, 							\
								 	 .pin = GPIO_PIN_3, 						\
								 	 .alternate_func = GPIO_AF1_TIM2}

#define TIM2_PWM_CH3_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM2, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_2, 						\
								 	 .alternate_func = GPIO_AF1_TIM2}

#define TIM2_PWM_CH3_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN, 	\
								 	 .timer = TIM2, 							\
								 	 .port = GPIOB, 							\
								 	 .pin = GPIO_PIN_10, 						\
								 	 .alternate_func = GPIO_AF1_TIM2}

#define TIM2_PWM_CH4_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM2, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_3, 						\
								 	 .alternate_func = GPIO_AF1_TIM2}

#define TIM2_PWM_CH4_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN, 	\
								 	 .timer = TIM2, 							\
								 	 .port = GPIOB, 							\
								 	 .pin = GPIO_PIN_11, 						\
								 	 .alternate_func = GPIO_AF1_TIM2}

#define TIM3_PWM_CH1_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM3, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_6, 						\
								 	 .alternate_func = GPIO_AF2_TIM3}

#define TIM3_PWM_CH1_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN, 	\
								 	 .timer = TIM3, 							\
								 	 .port = GPIOB, 							\
								 	 .pin = GPIO_PIN_4, 						\
								 	 .alternate_func = GPIO_AF2_TIM3}

#define TIM3_PWM_CH1_PP3_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOCEN, 	\
								 	 .timer = TIM3, 							\
								 	 .port = GPIOC, 							\
								 	 .pin = GPIO_PIN_6, 						\
								 	 .alternate_func = GPIO_AF2_TIM3}

#define TIM3_PWM_CH2_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM3, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_7, 						\
								 	 .alternate_func = GPIO_AF2_TIM3}

#define TIM3_PWM_CH2_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN, 	\
								 	 .timer = TIM3, 							\
								 	 .port = GPIOB, 							\
								 	 .pin = GPIO_PIN_5, 						\
								 	 .alternate_func = GPIO_AF2_TIM3}

#define TIM3_PWM_CH2_PP3_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOCEN, 	\
								 	 .timer = TIM3, 							\
								 	 .port = GPIOC, 							\
								 	 .pin = GPIO_PIN_7, 						\
								 	 .alternate_func = GPIO_AF2_TIM3}

#define TIM3_PWM_CH3_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN, 	\
								 	 .timer = TIM3, 							\
								 	 .port = GPIOB, 							\
								 	 .pin = GPIO_PIN_0, 						\
								 	 .alternate_func = GPIO_AF2_TIM3}

#define TIM3_PWM_CH3_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOCEN, 	\
								 	 .timer = TIM3, 							\
								 	 .port = GPIOC, 							\
								 	 .pin = GPIO_PIN_8, 						\
								 	 .alternate_func = GPIO_AF2_TIM3}

#define TIM3_PWM_CH4_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN, 	\
								 	 .timer = TIM3, 							\
								 	 .port = GPIOB, 							\
								 	 .pin = GPIO_PIN_1, 						\
								 	 .alternate_func = GPIO_AF2_TIM3}

#define TIM3_PWM_CH4_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOCEN, 	\
								 	 .timer = TIM3, 							\
								 	 .port = GPIOC, 							\
								 	 .pin = GPIO_PIN_9, 						\
								 	 .alternate_func = GPIO_AF2_TIM3}


#define TIM4_PWM_CH1_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM4EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN, 	\
								 	 .timer = TIM4, 							\
								 	 .port = GPIOB, 							\
								 	 .pin = GPIO_PIN_6, 						\
								 	 .alternate_func = GPIO_AF2_TIM4}

#define TIM4_PWM_CH1_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM4EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIODEN, 	\
								 	 .timer = TIM4, 							\
								 	 .port = GPIOD, 							\
								 	 .pin = GPIO_PIN_12, 						\
								 	 .alternate_func = GPIO_AF2_TIM4}

#define TIM4_PWM_CH2_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM4EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN, 	\
								 	 .timer = TIM4, 							\
								 	 .port = GPIOB, 							\
								 	 .pin = GPIO_PIN_7, 						\
								 	 .alternate_func = GPIO_AF2_TIM4}

#define TIM4_PWM_CH2_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM4EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIODEN, 	\
								 	 .timer = TIM4, 							\
								 	 .port = GPIOD, 							\
								 	 .pin = GPIO_PIN_13, 						\
								 	 .alternate_func = GPIO_AF2_TIM4}

#define TIM4_PWM_CH3_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM4EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN, 	\
								 	 .timer = TIM4, 							\
								 	 .port = GPIOB, 							\
								 	 .pin = GPIO_PIN_8, 						\
								 	 .alternate_func = GPIO_AF2_TIM4}

#define TIM4_PWM_CH3_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM4EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIODEN, 	\
								 	 .timer = TIM4, 							\
								 	 .port = GPIOD, 							\
								 	 .pin = GPIO_PIN_14, 						\
								 	 .alternate_func = GPIO_AF2_TIM4}

#define TIM4_PWM_CH4_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM4EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN, 	\
								 	 .timer = TIM4, 							\
								 	 .port = GPIOB, 							\
								 	 .pin = GPIO_PIN_9, 						\
								 	 .alternate_func = GPIO_AF2_TIM4}

#define TIM4_PWM_CH4_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM4EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIODEN, 	\
								 	 .timer = TIM4, 							\
								 	 .port = GPIOD, 							\
								 	 .pin = GPIO_PIN_15, 						\
								 	 .alternate_func = GPIO_AF2_TIM4}

#define TIM5_PWM_CH1_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM5EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM5, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_0, 						\
								 	 .alternate_func = GPIO_AF2_TIM5}

#define TIM5_PWM_CH1_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM5EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOHEN, 	\
								 	 .timer = TIM5, 							\
								 	 .port = GPIOH, 							\
								 	 .pin = GPIO_PIN_10, 						\
								 	 .alternate_func = GPIO_AF2_TIM5}

#define TIM5_PWM_CH2_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM5EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM5, 							\
								 	 .port = GPIOA,						\
								 	 .pin = GPIO_PIN_1, 						\
								 	 .alternate_func = GPIO_AF2_TIM5}

#define TIM5_PWM_CH2_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM5EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOHEN, 	\
								 	 .timer = TIM5, 							\
								 	 .port = GPIOH, 							\
								 	 .pin = GPIO_PIN_11, 						\
								 	 .alternate_func = GPIO_AF2_TIM5}

#define TIM5_PWM_CH3_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM5EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM5, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_2, 						\
								 	 .alternate_func = GPIO_AF2_TIM5}

#define TIM5_PWM_CH3_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM5EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOHEN, 	\
								 	 .timer = TIM5, 							\
								 	 .port = GPIOH, 							\
								 	 .pin = GPIO_PIN_12, 						\
								 	 .alternate_func = GPIO_AF2_TIM5}

#define TIM5_PWM_CH4_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM5EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM5, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_3, 						\
								 	 .alternate_func = GPIO_AF2_TIM5}

#define TIM5_PWM_CH4_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM5EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOIEN, 	\
								 	 .timer = TIM5, 							\
								 	 .port = GPIOI, 							\
								 	 .pin = GPIO_PIN_0, 						\
								 	 .alternate_func = GPIO_AF2_TIM5}

#define TIM8_PWM_CH1_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM8EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOCEN, 	\
								 	 .timer = TIM8, 							\
								 	 .port = GPIOC, 							\
								 	 .pin = GPIO_PIN_6, 						\
								 	 .alternate_func = GPIO_AF3_TIM8}

#define TIM8_PWM_CH1_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM8EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOIEN, 	\
								 	 .timer = TIM8, 							\
								 	 .port = GPIOI, 							\
								 	 .pin = GPIO_PIN_15, 						\
								 	 .alternate_func = GPIO_AF3_TIM8}

#define TIM8_PWM_CH2_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM8EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOCEN, 	\
								 	 .timer = TIM8, 							\
								 	 .port = GPIOC, 							\
								 	 .pin = GPIO_PIN_7, 						\
								 	 .alternate_func = GPIO_AF3_TIM8}

#define TIM8_PWM_CH2_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM8EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOIEN, 	\
								 	 .timer = TIM8, 							\
								 	 .port = GPIOI, 							\
								 	 .pin = GPIO_PIN_6, 						\
								 	 .alternate_func = GPIO_AF3_TIM8}

#define TIM8_PWM_CH3_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM8EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOCEN, 	\
								 	 .timer = TIM8, 							\
								 	 .port = GPIOC, 							\
								 	 .pin = GPIO_PIN_8, 						\
								 	 .alternate_func = GPIO_AF3_TIM8}

#define TIM8_PWM_CH3_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM8EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOIEN, 	\
								 	 .timer = TIM8, 							\
								 	 .port = GPIOI, 							\
								 	 .pin = GPIO_PIN_7, 						\
								 	 .alternate_func = GPIO_AF3_TIM8}

#define TIM8_PWM_CH4_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM8EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOCEN, 	\
								 	 .timer = TIM8, 							\
								 	 .port = GPIOC, 							\
								 	 .pin = GPIO_PIN_9, 						\
								 	 .alternate_func = GPIO_AF3_TIM8}

#define TIM8_PWM_CH4_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM8EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOIEN, 	\
								 	 .timer = TIM8, 							\
								 	 .port = GPIOI, 							\
								 	 .pin = GPIO_PIN_2, 						\
								 	 .alternate_func = GPIO_AF3_TIM8}

#define TIM9_PWM_CH1_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM9EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM9, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_2, 						\
								 	 .alternate_func = GPIO_AF3_TIM9}

#define TIM9_PWM_CH1_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM9EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOEEN, 	\
								 	 .timer = TIM9, 							\
								 	 .port = GPIOE, 							\
								 	 .pin = GPIO_PIN_5, 						\
								 	 .alternate_func = GPIO_AF3_TIM9}

#define TIM9_PWM_CH2_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM9EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM9, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_3, 						\
								 	 .alternate_func = GPIO_AF3_TIM9}

#define TIM9_PWM_CH2_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM9EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOEEN, 	\
								 	 .timer = TIM9, 							\
								 	 .port = GPIOE, 							\
								 	 .pin = GPIO_PIN_6, 						\
								 	 .alternate_func = GPIO_AF3_TIM9}

#define TIM10_PWM_CH1_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM10EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN, 	\
								 	 .timer = TIM10, 							\
								 	 .port = GPIOB, 							\
								 	 .pin = GPIO_PIN_8, 						\
								 	 .alternate_func = GPIO_AF3_TIM10}

#define TIM10_PWM_CH1_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM10EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOFEN, 	\
								 	 .timer = TIM10, 							\
								 	 .port = GPIOF, 							\
								 	 .pin = GPIO_PIN_6, 						\
								 	 .alternate_func = GPIO_AF3_TIM10}

#define TIM11_PWM_CH1_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM11EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN, 	\
								 	 .timer = TIM11, 							\
								 	 .port = GPIOB, 							\
								 	 .pin = GPIO_PIN_9, 						\
								 	 .alternate_func = GPIO_AF3_TIM11}

#define TIM11_PWM_CH1_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB2ENR_TIM11EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOFEN, 	\
								 	 .timer = TIM11, 							\
								 	 .port = GPIOF, 							\
								 	 .pin = GPIO_PIN_7, 						\
								 	 .alternate_func = GPIO_AF3_TIM11}

#define TIM12_PWM_CH1_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM12EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN, 	\
								 	 .timer = TIM12, 							\
								 	 .port = GPIOB, 							\
								 	 .pin = GPIO_PIN_14, 						\
								 	 .alternate_func = GPIO_AF9_TIM12}

#define TIM12_PWM_CH1_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM12EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOHEN, 	\
								 	 .timer = TIM12, 							\
								 	 .port = GPIOH, 							\
								 	 .pin = GPIO_PIN_6, 						\
								 	 .alternate_func = GPIO_AF9_TIM12}

#define TIM12_PWM_CH2_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM12EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN, 	\
								 	 .timer = TIM12, 							\
								 	 .port = GPIOB, 							\
								 	 .pin = GPIO_PIN_15, 						\
								 	 .alternate_func = GPIO_AF9_TIM12}

#define TIM12_PWM_CH2_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM12EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOHEN, 	\
								 	 .timer = TIM12, 							\
								 	 .port = GPIOH, 							\
								 	 .pin = GPIO_PIN_9, 						\
								 	 .alternate_func = GPIO_AF9_TIM12}

#define TIM13_PWM_CH1_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM13EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM13, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_6, 						\
								 	 .alternate_func = GPIO_AF9_TIM13}

#define TIM13_PWM_CH1_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM13EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOFEN, 	\
								 	 .timer = TIM13, 							\
								 	 .port = GPIOF, 							\
								 	 .pin = GPIO_PIN_8, 						\
								 	 .alternate_func = GPIO_AF9_TIM13}

#define TIM14_PWM_CH1_PP1_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM14EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN, 	\
								 	 .timer = TIM14, 							\
								 	 .port = GPIOA, 							\
								 	 .pin = GPIO_PIN_7, 						\
								 	 .alternate_func = GPIO_AF9_TIM14}

#define TIM14_PWM_CH1_PP2_HW_INFO	{.rcc_apbenr_timen = RCC_APB1ENR_TIM14EN, 	\
								 	 .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOFEN, 	\
								 	 .timer = TIM14, 							\
								 	 .port = GPIOF, 							\
								 	 .pin = GPIO_PIN_9, 						\
								 	 .alternate_func = GPIO_AF9_TIM14}


typedef struct {
	uint32_t 		rcc_apbenr_timen;
	uint32_t 		rcc_ahbenr_gpioen;
	TIM_TypeDef 	*timer;
	GPIO_TypeDef 	*port;
	uint16_t 		pin;
	uint8_t 		alternate_func;
} tim_pwm_hw_info_t;

typedef struct timer {
	timer_num_t 		timer_num;
	timer_channel_t 	timer_channel;
	timer_pins_pack_t 	timer_pins_pack;
	uint32_t 			pwm_freq_hz;
	uint8_t				pwm_duty_percent;
	TIM_HandleTypeDef 	hal_handle;
	tim_pwm_hw_info_t 	hw_info;
} timer_t;

typedef enum {
	TIMER_PARAM_MAPPING_RCC_AHB1ENR_GPIOxEN = 0,
	TIMER_PARAM_MAPPING_GPIO_PIN_x,
	TIMER_PARAM_MAPPING_GPIO_AFx_TIMx,
	TIMER_PARAM_MAPPING_GPIOx,
	TIMER_PARAM_INDEX_MAX
} timer_param_mapping_t;


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

tim_pwm_hw_info_t TIM_PWM_HW_INFO_MAPPING[TIMER_NUM_MAX][TIMER_CHANNEL_MAX][TIMER_PINS_PACK_MAX] = {
	{	{TIM1_PWM_CH1_PP1_HW_INFO , TIM1_PWM_CH1_PP2_HW_INFO ,                        0},
		{TIM1_PWM_CH2_PP1_HW_INFO , TIM1_PWM_CH2_PP2_HW_INFO ,                        0},
		{TIM1_PWM_CH3_PP1_HW_INFO , TIM1_PWM_CH3_PP2_HW_INFO ,                        0},
		{TIM1_PWM_CH4_PP1_HW_INFO , TIM1_PWM_CH4_PP2_HW_INFO ,                        0}
	},

	{	{TIM2_PWM_CH1_PP1_HW_INFO , TIM2_PWM_CH1_PP2_HW_INFO , TIM2_PWM_CH1_PP3_HW_INFO},
		{TIM2_PWM_CH2_PP1_HW_INFO , TIM2_PWM_CH2_PP2_HW_INFO ,                        0},
		{TIM2_PWM_CH3_PP1_HW_INFO , TIM2_PWM_CH3_PP2_HW_INFO ,                        0},
		{TIM2_PWM_CH4_PP1_HW_INFO , TIM2_PWM_CH4_PP2_HW_INFO ,                        0}
	},

	{	{TIM3_PWM_CH1_PP1_HW_INFO , TIM3_PWM_CH1_PP2_HW_INFO , TIM3_PWM_CH1_PP3_HW_INFO},
		{TIM3_PWM_CH2_PP1_HW_INFO , TIM3_PWM_CH2_PP2_HW_INFO , TIM3_PWM_CH2_PP3_HW_INFO},
		{TIM3_PWM_CH3_PP1_HW_INFO , TIM3_PWM_CH3_PP2_HW_INFO ,                        0},
		{TIM3_PWM_CH4_PP1_HW_INFO , TIM3_PWM_CH4_PP2_HW_INFO ,                        0}
	},

	{	{TIM4_PWM_CH1_PP1_HW_INFO , TIM4_PWM_CH1_PP2_HW_INFO ,                        0},
		{TIM4_PWM_CH2_PP1_HW_INFO , TIM4_PWM_CH2_PP2_HW_INFO ,                        0 },
		{TIM4_PWM_CH3_PP1_HW_INFO , TIM4_PWM_CH3_PP2_HW_INFO ,                        0},
		{TIM4_PWM_CH4_PP1_HW_INFO , TIM4_PWM_CH4_PP2_HW_INFO ,                        0}
	},

	{	{TIM5_PWM_CH1_PP1_HW_INFO , TIM5_PWM_CH1_PP2_HW_INFO ,                        0},
		{TIM5_PWM_CH2_PP1_HW_INFO , TIM5_PWM_CH2_PP2_HW_INFO ,                        0},
		{TIM5_PWM_CH3_PP1_HW_INFO , TIM5_PWM_CH3_PP2_HW_INFO ,                        0},
		{TIM5_PWM_CH4_PP1_HW_INFO , TIM5_PWM_CH4_PP2_HW_INFO ,                        0}
	},

	{	{                        0,                         0,                        0},
		{                        0,                         0,                        0},
		{                        0,                         0,                        0},
		{                        0,                         0,                        0}
	},

	{	{                        0,                         0,                        0},
		{                        0,                         0,                        0},
		{                        0,                         0,                        0},
		{                        0,                         0,                        0}
	},

	{	{TIM8_PWM_CH1_PP1_HW_INFO , TIM8_PWM_CH1_PP2_HW_INFO ,                        0},
		{TIM8_PWM_CH2_PP1_HW_INFO , TIM8_PWM_CH2_PP2_HW_INFO ,                        0},
		{TIM8_PWM_CH3_PP1_HW_INFO , TIM8_PWM_CH3_PP2_HW_INFO ,                        0},
		{TIM8_PWM_CH4_PP1_HW_INFO , TIM8_PWM_CH4_PP2_HW_INFO ,                        0}
	},

	{	{TIM9_PWM_CH1_PP1_HW_INFO , TIM9_PWM_CH1_PP2_HW_INFO ,                        0},
		{TIM9_PWM_CH2_PP1_HW_INFO , TIM9_PWM_CH2_PP2_HW_INFO ,                         },
		{                        0,                         0,                        0},
		{                        0,                         0,                        0}
	},

	{	{TIM10_PWM_CH1_PP1_HW_INFO, TIM10_PWM_CH1_PP2_HW_INFO,                         },
		{                        0,                         0,                        0},
		{                        0,                         0,                        0},
		{                        0,                         0,                        0}
	},

	{	{TIM11_PWM_CH1_PP1_HW_INFO, TIM11_PWM_CH1_PP2_HW_INFO,                         },
		{                        0,                         0,                        0},
		{                        0,                         0,                        0},
		{                        0,                         0,                        0}
	},

	{	{TIM12_PWM_CH1_PP1_HW_INFO, TIM12_PWM_CH1_PP2_HW_INFO,                         },
		{TIM12_PWM_CH2_PP1_HW_INFO, TIM12_PWM_CH2_PP2_HW_INFO,                         },
		{                        0,                         0,                        0},
		{                        0,                         0,                        0}
	},

	{	{TIM13_PWM_CH1_PP1_HW_INFO, TIM13_PWM_CH1_PP2_HW_INFO,                         },
		{                        0,                         0,                        0},
		{                        0,                         0,                        0},
		{                        0,                         0,                        0}
	},

	{	{TIM14_PWM_CH1_PP1_HW_INFO, TIM14_PWM_CH1_PP2_HW_INFO,                         },
		{                        0,                         0,                        0},
		{                        0,                         0,                        0},
		{                        0,                         0,                        0}
	},


};


static tim_pwm_hw_info_t tim_pwm_get_hw_info(timer_num_t timer_num, timer_channel_t timer_channel, timer_pins_pack_t timer_pins_pack)
{
	tim_pwm_hw_info_t hw_info;
	hw_info = TIM_PWM_HW_INFO_MAPPING[timer_num][timer_channel][timer_pins_pack];
	return hw_info;
}

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
	handle->hw_info = tim_pwm_get_hw_info(config->timer_num, config->timer_channel, config->timer_pins_pack);

	int err;

	do {
		__IO uint32_t tmpreg = 0x00U;
		SET_BIT(RCC->AHB1ENR, handle->hw_info.rcc_ahbenr_gpioen);
		tmpreg = READ_BIT(RCC->AHB1ENR, handle->hw_info.rcc_ahbenr_gpioen);
		UNUSED(tmpreg);
	} while (0U);

	do {
		__IO uint32_t tmpreg = 0x00U;
		if ((config->timer_num == TIMER_NUM_1) || (config->timer_num == TIMER_NUM_8) || (config->timer_num == TIMER_NUM_9) || (config->timer_num == TIMER_NUM_10) || (config->timer_num == TIMER_NUM_11)) {
			SET_BIT(RCC->APB2ENR, handle->hw_info.rcc_apbenr_timen);
			tmpreg = READ_BIT(RCC->APB2ENR, handle->hw_info.rcc_apbenr_timen);
		}
		else {
			SET_BIT(RCC->APB1ENR, handle->hw_info.rcc_apbenr_timen);
			tmpreg = READ_BIT(RCC->APB1ENR, handle->hw_info.rcc_apbenr_timen);
		}
		UNUSED(tmpreg);
	} while (0U);

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = handle->hw_info.pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = handle->hw_info.alternate_func;
	HAL_GPIO_Init(handle->hw_info.port, &GPIO_InitStruct);

	handle->hal_handle.Instance 				= handle->hw_info.timer;
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
	err = HAL_TIM_PWM_ConfigChannel(&handle->hal_handle, &sConfigOC, TIM_CHANNEL_x_MAPPING[config->timer_channel]);
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







