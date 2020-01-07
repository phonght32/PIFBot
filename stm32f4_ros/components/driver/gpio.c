#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"

#include "stdlib.h"

#include "include/gpio.h"

#define GPIO_SPEED_FREQ_DEFAULT  GPIO_SPEED_FREQ_VERY_HIGH

typedef struct gpio {
    gpio_port_t     gpio_port;
    gpio_num_t      gpio_num;
    gpio_mode_t     gpio_mode;
    gpio_reg_pull_t gpio_reg_pull;
} gpio_t;

GPIO_TypeDef *GPIOx_MAPPING[GPIO_PORT_MAX] = {
    GPIOA,
    GPIOB,
    GPIOC,
    GPIOD,
    GPIOE,
    GPIOF,
    GPIOG,
    GPIOH,
    GPIOI
};

uint16_t GPIO_PIN_x_MAPPING[GPIO_NUM_MAX] = {
    GPIO_PIN_0,
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_5,
    GPIO_PIN_6,
    GPIO_PIN_7,
    GPIO_PIN_8,
    GPIO_PIN_9,
    GPIO_PIN_10,
    GPIO_PIN_11,
    GPIO_PIN_12,
    GPIO_PIN_13,
    GPIO_PIN_14,
    GPIO_PIN_15
};

uint32_t GPIO_REG_PULL_MAPPING[GPIO_REG_PULL_MAX] = {
    GPIO_NOPULL,
    GPIO_PULLUP,
    GPIO_PULLDOWN
};

uint32_t RCC_AHB1ENR_GPIOxEN_MAPPING[GPIO_PORT_MAX] = {
    RCC_AHB1ENR_GPIOAEN,
    RCC_AHB1ENR_GPIOBEN,
    RCC_AHB1ENR_GPIOCEN,
    RCC_AHB1ENR_GPIODEN,
    RCC_AHB1ENR_GPIOEEN,
    RCC_AHB1ENR_GPIOFEN,
    RCC_AHB1ENR_GPIOGEN,
    RCC_AHB1ENR_GPIOHEN,
    RCC_AHB1ENR_GPIOIEN,
};

gpio_handle_t gpio_init(gpio_config_t *config)
{
    gpio_handle_t handle;
    handle = calloc(1, sizeof(gpio_t));
    if (handle == NULL)
    {
        return -1;
    }

    uint32_t RCC_AHB1ENR_GPIOxEN;
    uint16_t GPIO_PIN_x;
    GPIO_TypeDef *GPIOx;
    uint32_t GPIO_REG_PULL_TYPE;

    RCC_AHB1ENR_GPIOxEN = RCC_AHB1ENR_GPIOxEN_MAPPING[config->gpio_port];
    GPIO_PIN_x = GPIO_PIN_x_MAPPING[config->gpio_num];
    GPIOx = GPIOx_MAPPING[config->gpio_port];
    GPIO_REG_PULL_TYPE = GPIO_REG_PULL_MAPPING[config->gpio_reg_pull];

    do {
        __IO uint32_t tmpreg = 0x00U;
        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOxEN);
        tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOxEN);
        UNUSED(tmpreg);
    } while (0U);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (config->gpio_mode == GPIO_INPUT)
    {
        GPIO_InitStruct.Pin = GPIO_PIN_x;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_REG_PULL_TYPE;
        HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
    }
    if (config->gpio_mode == GPIO_OUTPUT)
    {
        HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);
        GPIO_InitStruct.Pin = GPIO_PIN_x;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_REG_PULL_TYPE;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_DEFAULT;
        HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
    }

    handle->gpio_port     = config->gpio_port;
    handle->gpio_num      = config->gpio_num;
    handle->gpio_mode     = config->gpio_mode;
    handle->gpio_reg_pull = config->gpio_reg_pull;

    return handle;
}

int gpio_set_level(gpio_handle_t handle, uint8_t state)
{
    HAL_GPIO_WritePin(GPIOx_MAPPING[handle->gpio_port], GPIO_PIN_x_MAPPING[handle->gpio_num], state);

    return 0;
}

int gpio_toggle_level(gpio_handle_t handle)
{
    HAL_GPIO_TogglePin(GPIOx_MAPPING[handle->gpio_port], GPIO_PIN_x_MAPPING[handle->gpio_num]);

    return 0;
}

