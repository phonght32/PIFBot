/* Includes ------------------------------------------------------------------*/
#include "include/gpio.h"

#include "stdlib.h"

/* Internal define -----------------------------------------------------------*/


/* Internal typedef ----------------------------------------------------------*/
typedef struct gpio {
    GPIO_TypeDef *GPIOx;
    uint32_t GPIO_Pin;
} gpio_t;

/* Internal variable ---------------------------------------------------------*/


/* Internal function ---------------------------------------------------------*/


/* External function ---------------------------------------------------------*/
gpio_handle_t gpio_output_init(gpio_config_t *config)
{
    switch ((uint32_t)config->GPIOx)
    {
        case (uint32_t)GPIOA: {
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
            break;
        }
        case (uint32_t)GPIOB: {
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
            break;
        }
        case (uint32_t)GPIOC: {
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
            break;
        }
        case (uint32_t)GPIOD: {
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
            break;
        }
        case (uint32_t)GPIOE: {
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
            break;
        }
        case (uint32_t)GPIOF: {
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
            break;
        }
        case (uint32_t)GPIOG: {
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
            break;
        }
        case (uint32_t)GPIOH: {
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
            break;
        }
        case (uint32_t)GPIOI: {
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
            break;
        }
        case (uint32_t)GPIOJ: {
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOJ, ENABLE);
            break;
        }
        case (uint32_t)GPIOK: {
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOK, ENABLE);
            break;
        }
        default:
            break;
    }

    GPIO_InitTypeDef GPIO_InitDef;
    GPIO_InitDef.GPIO_Pin = config->GPIO_Pin;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
    GPIO_InitDef.GPIO_PuPd = config->pull_reg;
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(config->GPIOx, &GPIO_InitDef);

    gpio_handle_t handle = calloc(1,sizeof(gpio_t));
    if(!handle)
    {
        return -1;
    }
    handle->GPIOx = config->GPIOx;
    handle->GPIO_Pin = config->GPIO_Pin;

    return handle;
}

gpio_handle_t gpio_input_init(gpio_config_t *config)
{
    switch ((uint32_t)config->GPIOx)
    {
    case (uint32_t)GPIOA: {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
        break;
    }
    case (uint32_t)GPIOB: {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
        break;
    }
    case (uint32_t)GPIOC: {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
        break;
    }
    case (uint32_t)GPIOD: {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
        break;
    }
    case (uint32_t)GPIOE: {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
        break;
    }
    case (uint32_t)GPIOF: {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
        break;
    }
    case (uint32_t)GPIOG: {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
        break;
    }
    case (uint32_t)GPIOH: {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
        break;
    }
    case (uint32_t)GPIOI: {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
        break;
    }
    case (uint32_t)GPIOJ: {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOJ, ENABLE);
        break;
    }
    case (uint32_t)GPIOK: {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOK, ENABLE);
        break;
    }
    default:
        break;
    }

    GPIO_InitTypeDef GPIO_InitDef;
    GPIO_InitDef.GPIO_Pin = config->GPIO_Pin;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
    GPIO_InitDef.GPIO_PuPd = config->pull_reg;
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(config->GPIOx, &GPIO_InitDef);

    gpio_handle_t handle = calloc(1,sizeof(gpio_t));
    handle->GPIOx = config->GPIOx;
    handle->GPIO_Pin = config->GPIO_Pin;

    return handle;
}

int gpio_set_level(gpio_handle_t handle, uint8_t level)
{
    if(level)
    {
        GPIO_SetBits(handle->GPIOx, handle->GPIO_Pin);
    }
    else 
    {
        GPIO_ResetBits(handle->GPIOx, handle->GPIO_Pin);
    }
    return 0;
}

int gpio_toggle(gpio_handle_t handle)
{
	GPIO_ToggleBits(handle->GPIOx, handle->GPIO_Pin);

	return 0;
}
