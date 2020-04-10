#include "driver/timer.h"

#define GPIO_MODE_DEFAULT           GPIO_MODE_AF_PP             /*!< Default GPIO mode */
#define GPIO_PULL_REG_DEFAULT       GPIO_NOPULL                 /*!< Default GPIO pull registor */
#define GPIO_SPEED_FREQ_DEFAULT     GPIO_SPEED_FREQ_VERY_HIGH   /*!< Default GPIO speed */

#define PWM_COUNTERMODE_DEFAULT     TIM_COUNTERMODE_UP          /*!< Default PWM counter mode */
#define PWM_TIM_CLOCK_DIV_DEFAULT   TIM_CLOCKDIVISION_DIV1      /*!< Default PWM timer clock divide */

#define EXT_CNT_PRES_DEFAULT        0                           /*!< Default external counter prescaler */

#define SYSTEM_CLOCK                168000000                   /*!< System clock */
#define APB1_CLOCK                  (SYSTEM_CLOCK/2)            /*!< APB1 clock */
#define APB2_CLOCK                  (SYSTEM_CLOCK)              /*!< APB2 clock */

#define TIMER_MAX_RELOAD            0xFFFF                      /*!< User timer max value (16bit value) */

#define TIMER_NUM_ERR_STR           "timer num error"
#define TIMER_PINS_PACK_ERR_STR     "timer pins pack error"
#define TIMER_CHANNEL_ERR_STR       "timer channel error"

#define PWM_INIT_ERR_STR            "pwn init error"
#define PWM_START_ERR_STR           "pwm start error"
#define PWM_STOP_ERR_STR            "pwm stop error"
#define PWM_SET_FREQ_ERR_STR        "pwm set frequency error"
#define PWM_SET_DUTYCYCLE_ERR_STR   "pwm set duty cycle error"
#define PWM_SET_PARAMS_ERR_STR      "pwm set params error"
#define PWM_FREQUENCY_ERR_STR       "pwm frequency error"
#define PWM_DUTYCYCLE_ERR_STR       "pwm duty cycle error"

#define ETR_INIT_ERR_STR            "etr init error"
#define ETR_START_ERR_STR           "etr start error"
#define ETR_STOP_ERR_STR            "etr stop error"
#define ETR_GET_VALUE_ERR_STR       "etr get value error"
#define ETR_SET_VALUE_ERR_STR       "etr set value error"
#define ETR_SET_MODE_ERR_STR        "etr set mode error"

static const char* TIMER_TAG = "DRIVER TIMER";
#define TIMER_CHECK(a, str, ret)  if(!(a)) {                                             \
        STM_LOGE(TIMER_TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);      \
        return (ret);                                                                  \
        }

/*
 * Timer Hardware Information.
 */
#define TIM1_CH1_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM1EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM1,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_8,                         \
                                     .alternate_func = GPIO_AF1_TIM1}

#define TIM1_CH1_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM1EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOEEN,  \
                                     .timer = TIM1,                             \
                                     .port = GPIOE,                             \
                                     .pin = GPIO_PIN_9,                         \
                                     .alternate_func = GPIO_AF1_TIM1}

#define TIM1_CH2_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM1EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM1,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_9,                         \
                                     .alternate_func = GPIO_AF1_TIM1}

#define TIM1_CH2_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM1EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOEEN,  \
                                     .timer = TIM1,                             \
                                     .port = GPIOE,                             \
                                     .pin = GPIO_PIN_10,                        \
                                     .alternate_func = GPIO_AF1_TIM1}

#define TIM1_CH3_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM1EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM1,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_10,                        \
                                     .alternate_func = GPIO_AF1_TIM1}

#define TIM1_CH3_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM1EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOEEN,  \
                                     .timer = TIM1,                             \
                                     .port = GPIOE,                             \
                                     .pin = GPIO_PIN_13,                        \
                                     .alternate_func = GPIO_AF1_TIM1}

#define TIM1_CH4_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM1EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM1,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_11,                        \
                                     .alternate_func = GPIO_AF1_TIM1}

#define TIM1_CH4_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM1EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOEEN,  \
                                     .timer = TIM1,                             \
                                     .port = GPIOE,                             \
                                     .pin = GPIO_PIN_14,                        \
                                     .alternate_func = GPIO_AF1_TIM1}

#define TIM2_CH1_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM2,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_0,                         \
                                     .alternate_func = GPIO_AF1_TIM2}

#define TIM2_CH1_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM2,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_5,                         \
                                     .alternate_func = GPIO_AF1_TIM2}

#define TIM2_CH1_PP3_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM2,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_15,                        \
                                     .alternate_func = GPIO_AF1_TIM2}

#define TIM2_CH2_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM2,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_1,                         \
                                     .alternate_func = GPIO_AF1_TIM2}

#define TIM2_CH2_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN,  \
                                     .timer = TIM2,                             \
                                     .port = GPIOB,                             \
                                     .pin = GPIO_PIN_3,                         \
                                     .alternate_func = GPIO_AF1_TIM2}

#define TIM2_CH3_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM2,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_2,                         \
                                     .alternate_func = GPIO_AF1_TIM2}

#define TIM2_CH3_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN,  \
                                     .timer = TIM2,                             \
                                     .port = GPIOB,                             \
                                     .pin = GPIO_PIN_10,                        \
                                     .alternate_func = GPIO_AF1_TIM2}

#define TIM2_CH4_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM2,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_3,                         \
                                     .alternate_func = GPIO_AF1_TIM2}

#define TIM2_CH4_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN,  \
                                     .timer = TIM2,                             \
                                     .port = GPIOB,                             \
                                     .pin = GPIO_PIN_11,                        \
                                     .alternate_func = GPIO_AF1_TIM2}

#define TIM3_CH1_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM3,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_6,                         \
                                     .alternate_func = GPIO_AF2_TIM3}

#define TIM3_CH1_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN,  \
                                     .timer = TIM3,                             \
                                     .port = GPIOB,                             \
                                     .pin = GPIO_PIN_4,                         \
                                     .alternate_func = GPIO_AF2_TIM3}

#define TIM3_CH1_PP3_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOCEN,  \
                                     .timer = TIM3,                             \
                                     .port = GPIOC,                             \
                                     .pin = GPIO_PIN_6,                         \
                                     .alternate_func = GPIO_AF2_TIM3}

#define TIM3_CH2_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM3,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_7,                         \
                                     .alternate_func = GPIO_AF2_TIM3}

#define TIM3_CH2_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN,  \
                                     .timer = TIM3,                             \
                                     .port = GPIOB,                             \
                                     .pin = GPIO_PIN_5,                         \
                                     .alternate_func = GPIO_AF2_TIM3}

#define TIM3_CH2_PP3_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOCEN,  \
                                     .timer = TIM3,                             \
                                     .port = GPIOC,                             \
                                     .pin = GPIO_PIN_7,                         \
                                     .alternate_func = GPIO_AF2_TIM3}

#define TIM3_CH3_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN,  \
                                     .timer = TIM3,                             \
                                     .port = GPIOB,                             \
                                     .pin = GPIO_PIN_0,                         \
                                     .alternate_func = GPIO_AF2_TIM3}

#define TIM3_CH3_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOCEN,  \
                                     .timer = TIM3,                             \
                                     .port = GPIOC,                             \
                                     .pin = GPIO_PIN_8,                         \
                                     .alternate_func = GPIO_AF2_TIM3}

#define TIM3_CH4_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN,  \
                                     .timer = TIM3,                             \
                                     .port = GPIOB,                             \
                                     .pin = GPIO_PIN_1,                         \
                                     .alternate_func = GPIO_AF2_TIM3}

#define TIM3_CH4_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOCEN,  \
                                     .timer = TIM3,                             \
                                     .port = GPIOC,                             \
                                     .pin = GPIO_PIN_9,                         \
                                     .alternate_func = GPIO_AF2_TIM3}


#define TIM4_CH1_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM4EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN,  \
                                     .timer = TIM4,                             \
                                     .port = GPIOB,                             \
                                     .pin = GPIO_PIN_6,                         \
                                     .alternate_func = GPIO_AF2_TIM4}

#define TIM4_CH1_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM4EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIODEN,  \
                                     .timer = TIM4,                             \
                                     .port = GPIOD,                             \
                                     .pin = GPIO_PIN_12,                        \
                                     .alternate_func = GPIO_AF2_TIM4}

#define TIM4_CH2_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM4EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN,  \
                                     .timer = TIM4,                             \
                                     .port = GPIOB,                             \
                                     .pin = GPIO_PIN_7,                         \
                                     .alternate_func = GPIO_AF2_TIM4}

#define TIM4_CH2_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM4EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIODEN,  \
                                     .timer = TIM4,                             \
                                     .port = GPIOD,                             \
                                     .pin = GPIO_PIN_13,                        \
                                     .alternate_func = GPIO_AF2_TIM4}

#define TIM4_CH3_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM4EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN,  \
                                     .timer = TIM4,                             \
                                     .port = GPIOB,                             \
                                     .pin = GPIO_PIN_8,                         \
                                     .alternate_func = GPIO_AF2_TIM4}

#define TIM4_CH3_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM4EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIODEN,  \
                                     .timer = TIM4,                             \
                                     .port = GPIOD,                             \
                                     .pin = GPIO_PIN_14,                        \
                                     .alternate_func = GPIO_AF2_TIM4}

#define TIM4_CH4_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM4EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN,  \
                                     .timer = TIM4,                             \
                                     .port = GPIOB,                             \
                                     .pin = GPIO_PIN_9,                         \
                                     .alternate_func = GPIO_AF2_TIM4}

#define TIM4_CH4_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM4EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIODEN,  \
                                     .timer = TIM4,                             \
                                     .port = GPIOD,                             \
                                     .pin = GPIO_PIN_15,                        \
                                     .alternate_func = GPIO_AF2_TIM4}

#define TIM5_CH1_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM5EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM5,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_0,                         \
                                     .alternate_func = GPIO_AF2_TIM5}

#define TIM5_CH1_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM5EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOHEN,  \
                                     .timer = TIM5,                             \
                                     .port = GPIOH,                             \
                                     .pin = GPIO_PIN_10,                        \
                                     .alternate_func = GPIO_AF2_TIM5}

#define TIM5_CH2_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM5EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM5,                             \
                                     .port = GPIOA,                     \
                                     .pin = GPIO_PIN_1,                         \
                                     .alternate_func = GPIO_AF2_TIM5}

#define TIM5_CH2_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM5EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOHEN,  \
                                     .timer = TIM5,                             \
                                     .port = GPIOH,                             \
                                     .pin = GPIO_PIN_11,                        \
                                     .alternate_func = GPIO_AF2_TIM5}

#define TIM5_CH3_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM5EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM5,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_2,                         \
                                     .alternate_func = GPIO_AF2_TIM5}

#define TIM5_CH3_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM5EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOHEN,  \
                                     .timer = TIM5,                             \
                                     .port = GPIOH,                             \
                                     .pin = GPIO_PIN_12,                        \
                                     .alternate_func = GPIO_AF2_TIM5}

#define TIM5_CH4_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM5EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM5,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_3,                         \
                                     .alternate_func = GPIO_AF2_TIM5}

#define TIM5_CH4_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM5EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOIEN,  \
                                     .timer = TIM5,                             \
                                     .port = GPIOI,                             \
                                     .pin = GPIO_PIN_0,                         \
                                     .alternate_func = GPIO_AF2_TIM5}

#define TIM8_CH1_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM8EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOCEN,  \
                                     .timer = TIM8,                             \
                                     .port = GPIOC,                             \
                                     .pin = GPIO_PIN_6,                         \
                                     .alternate_func = GPIO_AF3_TIM8}

#define TIM8_CH1_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM8EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOIEN,  \
                                     .timer = TIM8,                             \
                                     .port = GPIOI,                             \
                                     .pin = GPIO_PIN_15,                        \
                                     .alternate_func = GPIO_AF3_TIM8}

#define TIM8_CH2_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM8EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOCEN,  \
                                     .timer = TIM8,                             \
                                     .port = GPIOC,                             \
                                     .pin = GPIO_PIN_7,                         \
                                     .alternate_func = GPIO_AF3_TIM8}

#define TIM8_CH2_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM8EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOIEN,  \
                                     .timer = TIM8,                             \
                                     .port = GPIOI,                             \
                                     .pin = GPIO_PIN_6,                         \
                                     .alternate_func = GPIO_AF3_TIM8}

#define TIM8_CH3_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM8EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOCEN,  \
                                     .timer = TIM8,                             \
                                     .port = GPIOC,                             \
                                     .pin = GPIO_PIN_8,                         \
                                     .alternate_func = GPIO_AF3_TIM8}

#define TIM8_CH3_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM8EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOIEN,  \
                                     .timer = TIM8,                             \
                                     .port = GPIOI,                             \
                                     .pin = GPIO_PIN_7,                         \
                                     .alternate_func = GPIO_AF3_TIM8}

#define TIM8_CH4_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM8EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOCEN,  \
                                     .timer = TIM8,                             \
                                     .port = GPIOC,                             \
                                     .pin = GPIO_PIN_9,                         \
                                     .alternate_func = GPIO_AF3_TIM8}

#define TIM8_CH4_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM8EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOIEN,  \
                                     .timer = TIM8,                             \
                                     .port = GPIOI,                             \
                                     .pin = GPIO_PIN_2,                         \
                                     .alternate_func = GPIO_AF3_TIM8}

#define TIM9_CH1_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM9EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM9,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_2,                         \
                                     .alternate_func = GPIO_AF3_TIM9}

#define TIM9_CH1_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM9EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOEEN,  \
                                     .timer = TIM9,                             \
                                     .port = GPIOE,                             \
                                     .pin = GPIO_PIN_5,                         \
                                     .alternate_func = GPIO_AF3_TIM9}

#define TIM9_CH2_PP1_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM9EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM9,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_3,                         \
                                     .alternate_func = GPIO_AF3_TIM9}

#define TIM9_CH2_PP2_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM9EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOEEN,  \
                                     .timer = TIM9,                             \
                                     .port = GPIOE,                             \
                                     .pin = GPIO_PIN_6,                         \
                                     .alternate_func = GPIO_AF3_TIM9}

#define TIM10_CH1_PP1_HW_INFO       {.rcc_apbenr_timen = RCC_APB2ENR_TIM10EN,   \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN,  \
                                     .timer = TIM10,                            \
                                     .port = GPIOB,                             \
                                     .pin = GPIO_PIN_8,                         \
                                     .alternate_func = GPIO_AF3_TIM10}

#define TIM10_CH1_PP2_HW_INFO       {.rcc_apbenr_timen = RCC_APB2ENR_TIM10EN,   \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOFEN,  \
                                     .timer = TIM10,                            \
                                     .port = GPIOF,                             \
                                     .pin = GPIO_PIN_6,                         \
                                     .alternate_func = GPIO_AF3_TIM10}

#define TIM11_CH1_PP1_HW_INFO       {.rcc_apbenr_timen = RCC_APB2ENR_TIM11EN,   \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN,  \
                                     .timer = TIM11,                            \
                                     .port = GPIOB,                             \
                                     .pin = GPIO_PIN_9,                         \
                                     .alternate_func = GPIO_AF3_TIM11}

#define TIM11_CH1_PP2_HW_INFO       {.rcc_apbenr_timen = RCC_APB2ENR_TIM11EN,   \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOFEN,  \
                                     .timer = TIM11,                            \
                                     .port = GPIOF,                             \
                                     .pin = GPIO_PIN_7,                         \
                                     .alternate_func = GPIO_AF3_TIM11}

#define TIM12_CH1_PP1_HW_INFO       {.rcc_apbenr_timen = RCC_APB1ENR_TIM12EN,   \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN,  \
                                     .timer = TIM12,                            \
                                     .port = GPIOB,                             \
                                     .pin = GPIO_PIN_14,                        \
                                     .alternate_func = GPIO_AF9_TIM12}

#define TIM12_CH1_PP2_HW_INFO       {.rcc_apbenr_timen = RCC_APB1ENR_TIM12EN,   \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOHEN,  \
                                     .timer = TIM12,                            \
                                     .port = GPIOH,                             \
                                     .pin = GPIO_PIN_6,                         \
                                     .alternate_func = GPIO_AF9_TIM12}

#define TIM12_CH2_PP1_HW_INFO       {.rcc_apbenr_timen = RCC_APB1ENR_TIM12EN,   \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOBEN,  \
                                     .timer = TIM12,                            \
                                     .port = GPIOB,                             \
                                     .pin = GPIO_PIN_15,                        \
                                     .alternate_func = GPIO_AF9_TIM12}

#define TIM12_CH2_PP2_HW_INFO       {.rcc_apbenr_timen = RCC_APB1ENR_TIM12EN,   \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOHEN,  \
                                     .timer = TIM12,                            \
                                     .port = GPIOH,                             \
                                     .pin = GPIO_PIN_9,                         \
                                     .alternate_func = GPIO_AF9_TIM12}

#define TIM13_CH1_PP1_HW_INFO       {.rcc_apbenr_timen = RCC_APB1ENR_TIM13EN,   \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM13,                            \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_6,                         \
                                     .alternate_func = GPIO_AF9_TIM13}

#define TIM13_CH1_PP2_HW_INFO       {.rcc_apbenr_timen = RCC_APB1ENR_TIM13EN,   \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOFEN,  \
                                     .timer = TIM13,                            \
                                     .port = GPIOF,                             \
                                     .pin = GPIO_PIN_8,                         \
                                     .alternate_func = GPIO_AF9_TIM13}

#define TIM14_CH1_PP1_HW_INFO       {.rcc_apbenr_timen = RCC_APB1ENR_TIM14EN,   \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM14,                            \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_7,                         \
                                     .alternate_func = GPIO_AF9_TIM14}

#define TIM14_CH1_PP2_HW_INFO       {.rcc_apbenr_timen = RCC_APB1ENR_TIM14EN,   \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOFEN,  \
                                     .timer = TIM14,                            \
                                     .port = GPIOF,                             \
                                     .pin = GPIO_PIN_9,                         \
                                     .alternate_func = GPIO_AF9_TIM14}

/*
 * Timer External Counter Hardware Information.
 */
#define TIM1_PP1_ETR_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM1EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM1,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_12,                        \
                                     .alternate_func = GPIO_AF1_TIM1}

#define TIM1_PP2_ETR_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM1EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOEEN,  \
                                     .timer = TIM1,                             \
                                     .port = GPIOE,                             \
                                     .pin = GPIO_PIN_7,                         \
                                     .alternate_func = GPIO_AF1_TIM1}

#define TIM2_PP1_ETR_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM2,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_0,                         \
                                     .alternate_func = GPIO_AF1_TIM2}

#define TIM2_PP2_ETR_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM2,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_5,                         \
                                     .alternate_func = GPIO_AF1_TIM2}

#define TIM2_PP3_ETR_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM2EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM2,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_15,                        \
                                     .alternate_func = GPIO_AF1_TIM2}

#define TIM3_PP1_ETR_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM3EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIODEN,  \
                                     .timer = TIM3,                             \
                                     .port = GPIOD,                             \
                                     .pin = GPIO_PIN_2,                         \
                                     .alternate_func = GPIO_AF2_TIM3}

#define TIM4_PP1_ETR_HW_INFO        {.rcc_apbenr_timen = RCC_APB1ENR_TIM4EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOEEN,  \
                                     .timer = TIM4,                             \
                                     .port = GPIOE,                             \
                                     .pin = GPIO_PIN_0,                         \
                                     .alternate_func = GPIO_AF2_TIM4}

#define TIM8_PP1_ETR_HW_INFO        {.rcc_apbenr_timen = RCC_APB2ENR_TIM8EN,    \
                                     .rcc_ahbenr_gpioen = RCC_AHB1ENR_GPIOAEN,  \
                                     .timer = TIM8,                             \
                                     .port = GPIOA,                             \
                                     .pin = GPIO_PIN_0,                         \
                                     .alternate_func = GPIO_AF3_TIM8}
/*
 * Timer Hardware Information.
 */
typedef struct {
    uint32_t        rcc_apbenr_timen;       /*!< Timer RCC APBENR register */
    uint32_t        rcc_ahbenr_gpioen;      /*!< GPIO RCC AHPENR register */
    TIM_TypeDef     *timer;                 /*!< TIM */
    GPIO_TypeDef    *port;                  /*!< General Purpose I/O */
    uint16_t        pin;                    /*!< Pin */
    uint8_t         alternate_func;         /*!< Alternative function */
} tim_hw_info_t;

/*
 * Timer Handle.
 */
static TIM_HandleTypeDef timer_handle[TIMER_NUM_MAX];

/*
 * Timer Channel Mapping Table.
 */
uint32_t TIM_CHANNEL_x_MAPPING[TIMER_CHANNEL_MAX] = {
    TIM_CHANNEL_1,      /*!< HAL Timer channel 1 define */
    TIM_CHANNEL_2,      /*!< HAL Timer channel 2 define */
    TIM_CHANNEL_3,      /*!< HAL Timer channel 3 define */
    TIM_CHANNEL_4       /*!< HAL Timer channel 4 define */
};

uint32_t TIMER_COUNTER_MODE_MAPPING[TIMER_COUNTER_MODE_MAX] = {
    TIM_COUNTERMODE_UP,
    TIM_COUNTERMODE_DOWN
};

/*
 * APB Clock Mapping Table.
 */
uint32_t APBx_CLOCK_MAPPING[TIMER_NUM_MAX] = {
    APB2_CLOCK,         /*!< Timer 1 APB clock frequency */
    APB1_CLOCK,         /*!< Timer 2 APB clock frequency */
    APB1_CLOCK,         /*!< Timer 3 APB clock frequency */
    APB1_CLOCK,         /*!< Timer 4 APB clock frequency */
    APB1_CLOCK,         /*!< Timer 5 APB clock frequency */
    APB1_CLOCK,         /*!< Timer 6 APB clock frequency */
    APB1_CLOCK,         /*!< Timer 7 APB clock frequency */
    APB1_CLOCK,         /*!< Timer 8 APB clock frequency */
    APB2_CLOCK,         /*!< Timer 9 APB clock frequency */
    APB2_CLOCK,         /*!< Timer 10 APB clock frequency */
    APB2_CLOCK,         /*!< Timer 11 APB clock frequency */
    APB1_CLOCK,         /*!< Timer 12 APB clock frequency */
    APB1_CLOCK,         /*!< Timer 13 APB clock frequency */
    APB1_CLOCK          /*!< Timer 14 APB clock frequency */
};

/*
 * Timer Hardware Information Mapping Table.
 */
tim_hw_info_t TIM_HW_INFO_MAPPING[TIMER_NUM_MAX][TIMER_CHANNEL_MAX][TIMER_PINS_PACK_MAX] = {
    {   {TIM1_CH1_PP1_HW_INFO , TIM1_CH1_PP2_HW_INFO ,                  {0}},
        {TIM1_CH2_PP1_HW_INFO , TIM1_CH2_PP2_HW_INFO ,                  {0}},
        {TIM1_CH3_PP1_HW_INFO , TIM1_CH3_PP2_HW_INFO ,                  {0}},
        {TIM1_CH4_PP1_HW_INFO , TIM1_CH4_PP2_HW_INFO ,                  {0}}
    },

    {   {TIM2_CH1_PP1_HW_INFO , TIM2_CH1_PP2_HW_INFO , TIM2_CH1_PP3_HW_INFO},
        {TIM2_CH2_PP1_HW_INFO , TIM2_CH2_PP2_HW_INFO ,                  {0}},
        {TIM2_CH3_PP1_HW_INFO , TIM2_CH3_PP2_HW_INFO ,                  {0}},
        {TIM2_CH4_PP1_HW_INFO , TIM2_CH4_PP2_HW_INFO ,                  {0}}
    },

    {   {TIM3_CH1_PP1_HW_INFO , TIM3_CH1_PP2_HW_INFO , TIM3_CH1_PP3_HW_INFO},
        {TIM3_CH2_PP1_HW_INFO , TIM3_CH2_PP2_HW_INFO , TIM3_CH2_PP3_HW_INFO},
        {TIM3_CH3_PP1_HW_INFO , TIM3_CH3_PP2_HW_INFO ,                  {0}},
        {TIM3_CH4_PP1_HW_INFO , TIM3_CH4_PP2_HW_INFO ,                  {0}}
    },

    {   {TIM4_CH1_PP1_HW_INFO , TIM4_CH1_PP2_HW_INFO ,                  {0}},
        {TIM4_CH2_PP1_HW_INFO , TIM4_CH2_PP2_HW_INFO ,                  {0}},
        {TIM4_CH3_PP1_HW_INFO , TIM4_CH3_PP2_HW_INFO ,                  {0}},
        {TIM4_CH4_PP1_HW_INFO , TIM4_CH4_PP2_HW_INFO ,                  {0}}
    },

    {   {TIM5_CH1_PP1_HW_INFO , TIM5_CH1_PP2_HW_INFO ,                  {0}},
        {TIM5_CH2_PP1_HW_INFO , TIM5_CH2_PP2_HW_INFO ,                  {0}},
        {TIM5_CH3_PP1_HW_INFO , TIM5_CH3_PP2_HW_INFO ,                  {0}},
        {TIM5_CH4_PP1_HW_INFO , TIM5_CH4_PP2_HW_INFO ,                  {0}}
    },

    {   {                  {0},                   {0},                  {0}},
        {                  {0},                   {0},                  {0}},
        {                  {0},                   {0},                  {0}},
        {                  {0},                   {0},                  {0}}
    },

    {   {                  {0},                   {0},                  {0}},
        {                  {0},                   {0},                  {0}},
        {                  {0},                   {0},                  {0}},
        {                  {0},                   {0},                  {0}}
    },

    {   {TIM8_CH1_PP1_HW_INFO , TIM8_CH1_PP2_HW_INFO ,                  {0}},
        {TIM8_CH2_PP1_HW_INFO , TIM8_CH2_PP2_HW_INFO ,                  {0}},
        {TIM8_CH3_PP1_HW_INFO , TIM8_CH3_PP2_HW_INFO ,                  {0}},
        {TIM8_CH4_PP1_HW_INFO , TIM8_CH4_PP2_HW_INFO ,                  {0}}
    },

    {   {TIM9_CH1_PP1_HW_INFO , TIM9_CH1_PP2_HW_INFO ,                  {0}},
        {TIM9_CH2_PP1_HW_INFO , TIM9_CH2_PP2_HW_INFO ,                  {0}},
        {                  {0},                   {0},                  {0}},
        {                  {0},                   {0},                  {0}}
    },

    {   {TIM10_CH1_PP1_HW_INFO, TIM10_CH1_PP2_HW_INFO,                  {0}},
        {                  {0},                   {0},                  {0}},
        {                  {0},                   {0},                  {0}},
        {                  {0},                   {0},                  {0}}
    },

    {   {TIM11_CH1_PP1_HW_INFO, TIM11_CH1_PP2_HW_INFO,                  {0}},
        {                  {0},                   {0},                  {0}},
        {                  {0},                   {0},                  {0}},
        {                  {0},                   {0},                  {0}}
    },

    {   {TIM12_CH1_PP1_HW_INFO, TIM12_CH1_PP2_HW_INFO,                  {0}},
        {TIM12_CH2_PP1_HW_INFO, TIM12_CH2_PP2_HW_INFO,                  {0}},
        {                  {0},                   {0},                  {0}},
        {                  {0},                   {0},                  {0}}
    },

    {   {TIM13_CH1_PP1_HW_INFO, TIM13_CH1_PP2_HW_INFO,                  {0}},
        {                  {0},                   {0},                  {0}},
        {                  {0},                   {0},                  {0}},
        {                  {0},                   {0},                  {0}}
    },

    {   {TIM14_CH1_PP1_HW_INFO, TIM14_CH1_PP2_HW_INFO,                     },
        {                  {0},                   {0},                  {0}},
        {                  {0},                   {0},                  {0}},
        {                  {0},                   {0},                  {0}}
    }
};

/*
 * External Counter Hardware Information Mapping Table.
 */
tim_hw_info_t TIM_ETR_HW_INFO_MAPPING[TIMER_NUM_MAX][TIMER_PINS_PACK_MAX] = {
    { TIM1_PP1_ETR_HW_INFO, TIM1_PP2_ETR_HW_INFO,                  {0}},
    { TIM2_PP1_ETR_HW_INFO, TIM2_PP2_ETR_HW_INFO, TIM2_PP3_ETR_HW_INFO},
    { TIM3_PP1_ETR_HW_INFO,                  {0},                  {0}},
    { TIM4_PP1_ETR_HW_INFO,                  {0},                  {0}},
    {                  {0},                  {0},                  {0}},
    {                  {0},                  {0},                  {0}},
    {                  {0},                  {0},                  {0}},
    { TIM8_PP1_ETR_HW_INFO,                  {0},                  {0}},
    {                  {0},                  {0},                  {0}},
    {                  {0},                  {0},                  {0}},
    {                  {0},                  {0},                  {0}},
    {                  {0},                  {0},                  {0}},
    {                  {0},                  {0},                  {0}},
    {                  {0},                  {0},                  {0}},
};

/* 
 * TIM Mapping Table 
 */
static TIM_TypeDef *TIM_MAPPING[TIMER_NUM_MAX] = {
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

static tim_hw_info_t _tim_pwm_get_hw_info(timer_num_t timer_num, timer_channel_t timer_channel, timer_pins_pack_t timer_pins_pack)
{
    tim_hw_info_t hw_info;
    hw_info = TIM_HW_INFO_MAPPING[timer_num][timer_channel][timer_pins_pack];

    return hw_info;
}

static tim_hw_info_t _tim_etr_get_hw_info(timer_num_t timer_num, timer_pins_pack_t timer_pins_pack)
{
    tim_hw_info_t hw_info;
    hw_info = TIM_ETR_HW_INFO_MAPPING[timer_num][timer_pins_pack];

    return hw_info;
}

stm_err_t pwm_config(pwm_config_t *config)
{
    /* Check input condition */
    TIMER_CHECK(config, PWM_INIT_ERR_STR, STM_ERR_INVALID_ARG);
    TIMER_CHECK(config->timer_num < TIMER_NUM_MAX, PWM_INIT_ERR_STR, STM_ERR_INVALID_ARG);
    TIMER_CHECK(config->timer_pins_pack < TIMER_PINS_PACK_MAX, PWM_INIT_ERR_STR, STM_ERR_INVALID_ARG);
    TIMER_CHECK(config->timer_channel < TIMER_CHANNEL_MAX, PWM_INIT_ERR_STR, STM_ERR_INVALID_ARG);

    /* Get hardware information */
    tim_hw_info_t hw_info = _tim_pwm_get_hw_info(config->timer_num, config->timer_channel, config->timer_pins_pack);

    int ret;

    /* Enable GPIO clock */
    uint32_t tmpreg = 0x00;
    SET_BIT(RCC->AHB1ENR, hw_info.rcc_ahbenr_gpioen);
    tmpreg = READ_BIT(RCC->AHB1ENR, hw_info.rcc_ahbenr_gpioen);
    UNUSED(tmpreg);

    /* Enable timer clock */
    tmpreg = 0x00;
    if ((config->timer_num == TIMER_NUM_1) || (config->timer_num == TIMER_NUM_8) || (config->timer_num == TIMER_NUM_9) || (config->timer_num == TIMER_NUM_10) || (config->timer_num == TIMER_NUM_11)) {
        SET_BIT(RCC->APB2ENR, hw_info.rcc_apbenr_timen);
        tmpreg = READ_BIT(RCC->APB2ENR, hw_info.rcc_apbenr_timen);
    }
    else {
        SET_BIT(RCC->APB1ENR, hw_info.rcc_apbenr_timen);
        tmpreg = READ_BIT(RCC->APB1ENR, hw_info.rcc_apbenr_timen);
    }
    UNUSED(tmpreg);

    /* Configure GPIO Pin */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = hw_info.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_DEFAULT;
    GPIO_InitStruct.Pull = GPIO_PULL_REG_DEFAULT;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_DEFAULT;
    GPIO_InitStruct.Alternate = hw_info.alternate_func;
    HAL_GPIO_Init(hw_info.port, &GPIO_InitStruct);

    /* Configure Timer */
    timer_handle[config->timer_num].Instance                 = hw_info.timer;
    timer_handle[config->timer_num].Init.Prescaler           = 0;
    timer_handle[config->timer_num].Init.CounterMode         = PWM_COUNTERMODE_DEFAULT;
    timer_handle[config->timer_num].Init.Period              = 0;
    timer_handle[config->timer_num].Init.ClockDivision       = PWM_TIM_CLOCK_DIV_DEFAULT;
    timer_handle[config->timer_num].Init.AutoReloadPreload   = TIM_AUTORELOAD_PRELOAD_DISABLE;
    ret = HAL_TIM_Base_Init(&timer_handle[config->timer_num]);
    TIMER_CHECK(!ret, PWM_INIT_ERR_STR, STM_FAIL);

    /* Configure PWM */
    ret = HAL_TIM_PWM_Init(&timer_handle[config->timer_num]);
    TIMER_CHECK(!ret, PWM_INIT_ERR_STR, STM_FAIL);

    /* Configure Timer clock source */
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    ret = HAL_TIM_ConfigClockSource(&timer_handle[config->timer_num], &sClockSourceConfig);
    TIMER_CHECK(!ret, PWM_INIT_ERR_STR, STM_FAIL);

    /* Configure Timer in master mode */
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    ret = HAL_TIMEx_MasterConfigSynchronization(&timer_handle[config->timer_num], &sMasterConfig);
    TIMER_CHECK(!ret, PWM_INIT_ERR_STR, STM_FAIL);

    /* Configure Timer PWM channel */
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    ret = HAL_TIM_PWM_ConfigChannel(&timer_handle[config->timer_num], &sConfigOC, TIM_CHANNEL_x_MAPPING[config->timer_channel]);
    TIMER_CHECK(!ret, PWM_INIT_ERR_STR, STM_FAIL);

    HAL_TIM_Base_Start(&timer_handle[config->timer_num]);
    return STM_OK;
}

stm_err_t pwm_start(timer_num_t timer_num, timer_channel_t timer_channel)
{
    TIMER_CHECK(timer_num < TIMER_NUM_MAX, PWM_START_ERR_STR, STM_ERR_INVALID_ARG);
    TIMER_CHECK(timer_channel < TIMER_CHANNEL_MAX, PWM_START_ERR_STR, STM_ERR_INVALID_ARG);

    HAL_TIM_PWM_Start(&timer_handle[timer_num], TIM_CHANNEL_x_MAPPING[timer_channel]);
    return STM_OK;
}

stm_err_t pwm_stop(timer_num_t timer_num, timer_channel_t timer_channel)
{
    TIMER_CHECK(timer_num < TIMER_NUM_MAX, PWM_STOP_ERR_STR, STM_ERR_INVALID_ARG);
    TIMER_CHECK(timer_channel < TIMER_CHANNEL_MAX, PWM_STOP_ERR_STR, STM_ERR_INVALID_ARG);

    HAL_TIM_PWM_Stop(&timer_handle[timer_num], TIM_CHANNEL_x_MAPPING[timer_channel]);
    return STM_OK;
}

stm_err_t pwm_set_frequency(timer_num_t timer_num, timer_channel_t timer_channel, uint32_t freq_hz)
{
    TIMER_CHECK(timer_num < TIMER_NUM_MAX, PWM_SET_FREQ_ERR_STR, STM_ERR_INVALID_ARG);
    TIMER_CHECK(timer_channel < TIMER_CHANNEL_MAX, PWM_SET_FREQ_ERR_STR, STM_ERR_INVALID_ARG);

    uint16_t cur_period = __HAL_TIM_GET_AUTORELOAD(&timer_handle[timer_num]);
    uint32_t cur_compare  = __HAL_TIM_GET_COMPARE(&timer_handle[timer_num], TIM_CHANNEL_x_MAPPING[timer_channel]);
    uint8_t cur_duty = (uint8_t)(cur_compare * 100 / cur_period);

    /* Calculate Timer PWM parameters. When change timer period you also
     * need to update timer compare value to keep duty cycle stable */
    uint32_t conduct = (uint32_t) (APBx_CLOCK_MAPPING[timer_num] / freq_hz);
    uint16_t timer_prescaler = conduct / TIMER_MAX_RELOAD + 1;
    uint16_t timer_period = (uint16_t)(conduct / (timer_prescaler + 1)) - 1;
    uint32_t timer_compare_value = cur_duty * timer_period / 100;

    /* Configure Timer PWM parameters */
    __HAL_TIM_SET_AUTORELOAD(&timer_handle[timer_num], timer_period);
    __HAL_TIM_SET_PRESCALER(&timer_handle[timer_num], timer_prescaler);
    __HAL_TIM_SET_COMPARE(&timer_handle[timer_num], TIM_CHANNEL_x_MAPPING[timer_channel], timer_compare_value);
    return STM_OK;
}

stm_err_t pwm_set_params(timer_num_t timer_num, timer_channel_t timer_channel, uint32_t freq_hz, uint8_t duty_percent)
{
    TIMER_CHECK(timer_num < TIMER_NUM_MAX, PWM_SET_PARAMS_ERR_STR, STM_ERR_INVALID_ARG);
    TIMER_CHECK(timer_channel < TIMER_CHANNEL_MAX, PWM_SET_PARAMS_ERR_STR, STM_ERR_INVALID_ARG);

    /* Calculate Timer PWM parameters. When change timer period you also
     * need to update timer compare value to keep duty cycle stable */
    uint32_t conduct = (uint32_t) (APBx_CLOCK_MAPPING[timer_num] / freq_hz);
    uint16_t timer_prescaler = conduct / TIMER_MAX_RELOAD + 1;
    uint16_t timer_period = (uint16_t)(conduct / (timer_prescaler + 1)) - 1;
    uint32_t timer_compare_value = duty_percent * timer_period / 100;

    /* Configure Timer PWM parameters */
    __HAL_TIM_SET_AUTORELOAD(&timer_handle[timer_num], timer_period);
    __HAL_TIM_SET_PRESCALER(&timer_handle[timer_num], timer_prescaler);
    __HAL_TIM_SET_COMPARE(&timer_handle[timer_num], TIM_CHANNEL_x_MAPPING[timer_channel], timer_compare_value);
    return STM_OK;
}

stm_err_t pwm_set_duty(timer_num_t timer_num, timer_channel_t timer_channel, uint8_t duty_percent)
{
    TIMER_CHECK(timer_num < TIMER_NUM_MAX, PWM_SET_DUTYCYCLE_ERR_STR, STM_ERR_INVALID_ARG);
    TIMER_CHECK(timer_channel < TIMER_CHANNEL_MAX, PWM_SET_DUTYCYCLE_ERR_STR, STM_ERR_INVALID_ARG);

    /* Calculate PWM compare value */
    uint32_t compare_value;
    compare_value = duty_percent * (timer_handle[timer_num].Instance->ARR) / 100;

    /* Configure PWM compare value */
    __HAL_TIM_SET_COMPARE(&timer_handle[timer_num], TIM_CHANNEL_x_MAPPING[timer_channel], compare_value);
    return STM_OK;
}

stm_err_t etr_config(etr_config_t *config)
{
    /* Check input condition */
    TIMER_CHECK(config, ETR_INIT_ERR_STR, STM_ERR_INVALID_ARG);
    TIMER_CHECK(config->timer_num < TIMER_NUM_MAX, ETR_INIT_ERR_STR, STM_ERR_INVALID_ARG);
    TIMER_CHECK(config->timer_pins_pack < TIMER_PINS_PACK_MAX, ETR_INIT_ERR_STR, STM_ERR_INVALID_ARG);
    TIMER_CHECK(config->counter_mode < TIMER_COUNTER_MODE_MAX, ETR_INIT_ERR_STR, STM_ERR_INVALID_ARG);

    /* Get hardware information */
    tim_hw_info_t hw_info = _tim_etr_get_hw_info(config->timer_num, config->timer_pins_pack);

    int ret;

    /* Enable GPIO clock */
    uint32_t tmpreg = 0x00;
    SET_BIT(RCC->AHB1ENR, hw_info.rcc_ahbenr_gpioen);
    tmpreg = READ_BIT(RCC->AHB1ENR, hw_info.rcc_ahbenr_gpioen);
    UNUSED(tmpreg);

    /* Enable timer clock */
    tmpreg = 0x00;
    if ((config->timer_num == TIMER_NUM_1) || (config->timer_num == TIMER_NUM_8) || (config->timer_num == TIMER_NUM_9) || (config->timer_num == TIMER_NUM_10) || (config->timer_num == TIMER_NUM_11)) {
        SET_BIT(RCC->APB2ENR, hw_info.rcc_apbenr_timen);
        tmpreg = READ_BIT(RCC->APB2ENR, hw_info.rcc_apbenr_timen);
    }
    else {
        SET_BIT(RCC->APB1ENR, hw_info.rcc_apbenr_timen);
        tmpreg = READ_BIT(RCC->APB1ENR, hw_info.rcc_apbenr_timen);
    }
    UNUSED(tmpreg);

    /* Configure GPIO pin */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = hw_info.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_DEFAULT;
    GPIO_InitStruct.Pull = GPIO_PULL_REG_DEFAULT;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_DEFAULT;
    GPIO_InitStruct.Alternate = hw_info.alternate_func;
    HAL_GPIO_Init(hw_info.port, &GPIO_InitStruct);

    /* Configure Timer */
    timer_handle[config->timer_num].Instance                 = hw_info.timer;
    timer_handle[config->timer_num].Init.Prescaler           = EXT_CNT_PRES_DEFAULT;
    timer_handle[config->timer_num].Init.CounterMode         = TIMER_COUNTER_MODE_MAPPING[config->counter_mode];
    timer_handle[config->timer_num].Init.Period              = config->max_reload;
    timer_handle[config->timer_num].Init.ClockDivision       = PWM_TIM_CLOCK_DIV_DEFAULT;
    timer_handle[config->timer_num].Init.AutoReloadPreload   = TIM_AUTORELOAD_PRELOAD_DISABLE;
    ret = HAL_TIM_Base_Init(&timer_handle[config->timer_num]);
    TIMER_CHECK(!ret, ETR_INIT_ERR_STR, STM_FAIL);

    /* Configure Timer clock source */
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
    sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
    sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
    sClockSourceConfig.ClockFilter = 0;
    ret = HAL_TIM_ConfigClockSource(&timer_handle[config->timer_num], &sClockSourceConfig);
    TIMER_CHECK(!ret, ETR_INIT_ERR_STR, STM_FAIL);

    /* Configure Timer in master mode */
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    ret = HAL_TIMEx_MasterConfigSynchronization(&timer_handle[config->timer_num], &sMasterConfig);
    TIMER_CHECK(!ret, ETR_INIT_ERR_STR, STM_FAIL);

    return STM_OK;
}

stm_err_t etr_start(timer_num_t timer_num)
{
    TIMER_CHECK(timer_num < TIMER_NUM_MAX, ETR_START_ERR_STR, STM_ERR_INVALID_ARG);

    /* Start timer base */
    HAL_TIM_Base_Start(&timer_handle[timer_num]);
    return STM_OK;
}

stm_err_t etr_stop(timer_num_t timer_num)
{
    TIMER_CHECK(timer_num < TIMER_NUM_MAX, ETR_STOP_ERR_STR, STM_ERR_INVALID_ARG);

    /* Stop time base */
    HAL_TIM_Base_Stop(&timer_handle[timer_num]);
    return STM_OK;
}

stm_err_t etr_get_value(timer_num_t timer_num, uint32_t *value)
{
    TIMER_CHECK(timer_num < TIMER_NUM_MAX, ETR_GET_VALUE_ERR_STR, STM_ERR_INVALID_ARG);

    /* Get counter value */
    *value = __HAL_TIM_GET_COUNTER(&timer_handle[timer_num]);
    return STM_OK;
}

stm_err_t etr_set_value(timer_num_t timer_num, uint32_t value)
{
    TIMER_CHECK(timer_num < TIMER_NUM_MAX, ETR_SET_VALUE_ERR_STR, STM_ERR_INVALID_ARG);

    /* Set counter value */
    __HAL_TIM_SET_COUNTER(&timer_handle[timer_num], value);
    return STM_OK;
}

stm_err_t etr_set_mode(timer_num_t timer_num, timer_counter_mode_t counter_mode)
{
    TIMER_CHECK(timer_num < TIMER_NUM_MAX, ETR_SET_MODE_ERR_STR, STM_ERR_INVALID_ARG);
    TIMER_CHECK(counter_mode < TIMER_COUNTER_MODE_MAX, ETR_SET_MODE_ERR_STR, STM_ERR_INVALID_ARG);

    /* Reconfigure timer init parameters */
    timer_handle[timer_num].Instance                 = TIM_MAPPING[timer_num];
    timer_handle[timer_num].Init.Prescaler           = EXT_CNT_PRES_DEFAULT;
    timer_handle[timer_num].Init.CounterMode         = TIMER_COUNTER_MODE_MAPPING[counter_mode];
    timer_handle[timer_num].Init.Period              = __HAL_TIM_GET_AUTORELOAD(&timer_handle[timer_num]);
    timer_handle[timer_num].Init.ClockDivision       = PWM_TIM_CLOCK_DIV_DEFAULT;
    timer_handle[timer_num].Init.AutoReloadPreload   = TIM_AUTORELOAD_PRELOAD_DISABLE;

    /* Keep last counter value */
    uint32_t last_counter_val = __HAL_TIM_GET_COUNTER(&timer_handle[timer_num]);

    /* Set timer counter mode */
    int ret = HAL_TIM_Base_Init(&timer_handle[timer_num]);
    TIMER_CHECK(!ret, ETR_SET_MODE_ERR_STR, STM_FAIL);

    /* Set timer last counter value */
    __HAL_TIM_SET_COUNTER(&timer_handle[timer_num], last_counter_val);

    return STM_OK;
}







