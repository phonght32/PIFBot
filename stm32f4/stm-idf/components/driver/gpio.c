#include "driver/gpio.h"

#define GPIO_SPEED_FREQ_DEFAULT     GPIO_SPEED_FREQ_VERY_HIGH   /*!< GPIO speed frequency default */
#define GPIO_LEVEL_DEFAULT          0                           /*!< GPIO level default */

#define GPIO_INIT_ERR_STR           "gpio init error"
#define GPIO_SET_LEVEL_ERR_STR      "gpio set level error"
#define GPIO_TOGGLE_LEVEL_ERR_STR   "gpio toggle level error"

static const char* GPIO_TAG = "DRIVER GPIO";
#define GPIO_CHECK(a, str, ret)  if(!(a)) {                                             \
        STM_LOGE(GPIO_TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);      \
        return (ret);                                                                  \
        }

/*
 * GPIO Port Mapping Table.
 */
static GPIO_TypeDef *GPIOx_MAPPING[GPIO_PORT_MAX] = {
    GPIOA,          /*!< HAL GPIO Port A define */
    GPIOB,          /*!< HAL GPIO Port B define */
    GPIOC,          /*!< HAL GPIO Port C define */
    GPIOD,          /*!< HAL GPIO Port D define */
    GPIOE,          /*!< HAL GPIO Port E define */
    GPIOF,          /*!< HAL GPIO Port F define */
    GPIOG,          /*!< HAL GPIO Port G define */
    GPIOH,          /*!< HAL GPIO Port H define */
    GPIOI           /*!< HAL GPIO Port I define */
};

/*
 * GPIO Pin Mapping Table.
 */
static uint16_t GPIO_PIN_x_MAPPING[GPIO_NUM_MAX] = {
    GPIO_PIN_0,     /*!< HAL GPIO Pin 0 define */
    GPIO_PIN_1,     /*!< HAL GPIO Pin 1 define */
    GPIO_PIN_2,     /*!< HAL GPIO Pin 2 define */
    GPIO_PIN_3,     /*!< HAL GPIO Pin 3 define */
    GPIO_PIN_4,     /*!< HAL GPIO Pin 4 define */
    GPIO_PIN_5,     /*!< HAL GPIO Pin 5 define */
    GPIO_PIN_6,     /*!< HAL GPIO Pin 6 define */
    GPIO_PIN_7,     /*!< HAL GPIO Pin 7 define */
    GPIO_PIN_8,     /*!< HAL GPIO Pin 8 define */
    GPIO_PIN_9,     /*!< HAL GPIO Pin 9 define */
    GPIO_PIN_10,    /*!< HAL GPIO Pin 10 define */
    GPIO_PIN_11,    /*!< HAL GPIO Pin 11 define */
    GPIO_PIN_12,    /*!< HAL GPIO Pin 12 define */
    GPIO_PIN_13,    /*!< HAL GPIO Pin 13 define */
    GPIO_PIN_14,    /*!< HAL GPIO Pin 14 define */
    GPIO_PIN_15     /*!< HAL GPIO Pin 15 define */
};

/*
 * GPIO Pull Register Mapping Table.
 */
static uint32_t GPIO_REG_PULL_MAPPING[GPIO_REG_PULL_MAX] = {
    GPIO_NOPULL,    /*!< HAL GPIO no pull registor define */
    GPIO_PULLUP,    /*!< HAL GPIO pull up registor define */
    GPIO_PULLDOWN   /*!< HAL GPIO pull down registor define */
};

/*
 * GPIO RCC_APBENR Register Mapping Table.
 */
static uint32_t RCC_AHB1ENR_GPIOxEN_MAPPING[GPIO_PORT_MAX] = {
    RCC_AHB1ENR_GPIOAEN,    /*!< HAL GPIO Port A RCC AHBENR Register define */
    RCC_AHB1ENR_GPIOBEN,    /*!< HAL GPIO Port B RCC AHBENR Register define */
    RCC_AHB1ENR_GPIOCEN,    /*!< HAL GPIO Port C RCC AHBENR Register define */
    RCC_AHB1ENR_GPIODEN,    /*!< HAL GPIO Port D RCC AHBENR Register define */
    RCC_AHB1ENR_GPIOEEN,    /*!< HAL GPIO Port E RCC AHBENR Register define */
    RCC_AHB1ENR_GPIOFEN,    /*!< HAL GPIO Port F RCC AHBENR Register define */
    RCC_AHB1ENR_GPIOGEN,    /*!< HAL GPIO Port G RCC AHBENR Register define */
    RCC_AHB1ENR_GPIOHEN,    /*!< HAL GPIO Port H RCC AHBENR Register define */
    RCC_AHB1ENR_GPIOIEN,    /*!< HAL GPIO Port I RCC AHBENR Register define */
};

stm_err_t gpio_config(gpio_config_t *config)
{
    /* Check input parameters */
    GPIO_CHECK(config, GPIO_INIT_ERR_STR, STM_ERR_INVALID_ARG);
    GPIO_CHECK(config->gpio_port < GPIO_PORT_MAX, GPIO_INIT_ERR_STR, STM_ERR_INVALID_ARG);
    GPIO_CHECK(config->gpio_num < GPIO_NUM_MAX, GPIO_INIT_ERR_STR, STM_ERR_INVALID_ARG);
    GPIO_CHECK(config->mode < GPIO_MODE_MAX, GPIO_INIT_ERR_STR, STM_ERR_INVALID_ARG);
    GPIO_CHECK(config->pull_mode < GPIO_REG_PULL_MAX, GPIO_INIT_ERR_STR, STM_ERR_INVALID_ARG);

    /* Mapping GPIO Parameters */
    uint32_t RCC_AHB1ENR_GPIOxEN;
    uint16_t GPIO_PIN_x;
    GPIO_TypeDef *GPIOx;
    uint32_t GPIO_REG_PULL_TYPE;

    RCC_AHB1ENR_GPIOxEN = RCC_AHB1ENR_GPIOxEN_MAPPING[config->gpio_port];
    GPIO_PIN_x = GPIO_PIN_x_MAPPING[config->gpio_num];
    GPIOx = GPIOx_MAPPING[config->gpio_port];
    GPIO_REG_PULL_TYPE = GPIO_REG_PULL_MAPPING[config->pull_mode];

    /* Enable GPIO Port clock */
    uint32_t tmpreg = 0x00;
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOxEN);
    tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOxEN);
    UNUSED(tmpreg);

    /* Initialize GPIO function */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (config->mode == GPIO_INPUT)        /*!< Input mode configuration */
    {
        GPIO_InitStruct.Pin = GPIO_PIN_x;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_REG_PULL_TYPE;
        HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
    }
    if (config->mode == GPIO_OUTPUT)       /*!< Output mode configuration */
    {
        HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);
        GPIO_InitStruct.Pin = GPIO_PIN_x;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_REG_PULL_TYPE;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_DEFAULT;
        HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

        /* Set GPIO default level */
        HAL_GPIO_WritePin(GPIOx_MAPPING[config->gpio_port], GPIO_PIN_x_MAPPING[config->gpio_num], 0);
    }

    return STM_OK;
}

stm_err_t gpio_set_level(gpio_port_t gpio_port, gpio_num_t gpio_num, bool state)
{
    GPIO_CHECK(gpio_port < GPIO_PORT_MAX, GPIO_SET_LEVEL_ERR_STR, STM_ERR_INVALID_ARG);
    GPIO_CHECK(gpio_num < GPIO_NUM_MAX, GPIO_SET_LEVEL_ERR_STR, STM_ERR_INVALID_ARG);

    HAL_GPIO_WritePin(GPIOx_MAPPING[gpio_port], GPIO_PIN_x_MAPPING[gpio_num], state);
    return STM_OK;
}

stm_err_t gpio_toggle_level(gpio_port_t gpio_port, gpio_num_t gpio_num)
{
    GPIO_CHECK(gpio_port < GPIO_PORT_MAX, GPIO_TOGGLE_LEVEL_ERR_STR, STM_ERR_INVALID_ARG);
    GPIO_CHECK(gpio_num < GPIO_NUM_MAX, GPIO_TOGGLE_LEVEL_ERR_STR, STM_ERR_INVALID_ARG);

    HAL_GPIO_TogglePin(GPIOx_MAPPING[gpio_port], GPIO_PIN_x_MAPPING[gpio_num]);
    return STM_OK;
}



