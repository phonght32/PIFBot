#ifndef GPIO_H_
#define GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#include "stdint.h"

typedef struct gpio *gpio_handle_t;

typedef enum {
    GPIO_PULL_REG_DISABLE = 0,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN
} gpio_pull_regs_t;

typedef struct {
    GPIO_TypeDef *GPIOx;
    uint32_t GPIO_Pin;
    gpio_pull_regs_t pull_reg;
} gpio_config_t;

gpio_handle_t gpio_output_init(gpio_config_t *config);
gpio_handle_t gpio_input_init(gpio_config_t *config);
int gpio_set_level(gpio_handle_t handle, uint8_t level);
int gpio_toggle(gpio_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_H_ */
