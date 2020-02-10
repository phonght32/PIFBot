#ifndef _GPIO_H_
#define _GPIO_H_

#ifdef __cplusplus 
extern "C" {
#endif

#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"

typedef struct gpio *gpio_handle_t;

typedef enum {
    GPIO_PORT_A = 0,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_F,
    GPIO_PORT_G,
    GPIO_PORT_H,
    GPIO_PORT_I,
    GPIO_PORT_MAX
} gpio_port_t;

typedef enum {
    GPIO_NUM_0 = 0,
    GPIO_NUM_1,
    GPIO_NUM_2,
    GPIO_NUM_3,
    GPIO_NUM_4,
    GPIO_NUM_5,
    GPIO_NUM_6,
    GPIO_NUM_7,
    GPIO_NUM_8,
    GPIO_NUM_9,
    GPIO_NUM_10,
    GPIO_NUM_11,
    GPIO_NUM_12,
    GPIO_NUM_13,
    GPIO_NUM_14,
    GPIO_NUM_15,
    GPIO_NUM_MAX
} gpio_num_t;

typedef enum {
    GPIO_INPUT = 0, 
    GPIO_OUTPUT
} gpio_mode_t;

typedef enum {
    GPIO_REG_PULL_NONE = 0,
    GPIO_REG_PULL_UP,
    GPIO_REG_PULL_DOWN,
    GPIO_REG_PULL_MAX
} gpio_reg_pull_t;

typedef struct {
    gpio_port_t     gpio_port;
    gpio_num_t      gpio_num;
    gpio_mode_t     gpio_mode;
    gpio_reg_pull_t gpio_reg_pull;
} gpio_config_t;

gpio_handle_t gpio_init(gpio_config_t *config);
int gpio_set_level(gpio_handle_t handle, uint8_t state);
int gpio_toggle_level(gpio_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* _GPIO_H_ */
