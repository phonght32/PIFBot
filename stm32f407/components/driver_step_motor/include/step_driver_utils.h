#ifndef STEP_DRIVER_UTILS_H_
#define STEP_DRIVER_UTILS_H_

#ifdef __cplusplus 
extern "C" {
#endif

#include "stdint.h"

#include "../../driver/include/timer.h"
#include "../../driver/include/gpio.h"

typedef gpio_config_t pin_dir_t;
typedef pwm_config_t pin_clk_t;

typedef enum {
    MICRO_STEP_DIV1 = 0,
    MICRO_STEP_DIV2,
    MICRO_STEP_DIV8,
    MICRO_STEP_DIV16
} micro_step_div_t;


#ifdef __cplusplus
}
#endif

#endif /* STEP_DRIVER_UTILS_H_ */
