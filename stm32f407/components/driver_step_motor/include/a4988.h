#ifndef A4988_H_
#define A4988_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#include "../../driver/include/timer.h"
#include "../../driver/include/gpio.h"

typedef gpio_config_t pin_dir_t;
typedef pwm_config_t pin_clk_t;

typedef struct {
    pin_clk_t pin_clk;
    pin_dir_t pin_dir;
} a4988_config_t;

#ifdef __cplusplus
}
#endif

#endif /* A4988_H_ */