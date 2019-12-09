#ifndef A4988_H_
#define A4988_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#include "../../driver/include/timer.h"
#include "../../driver/include/gpio.h"
#include "step_driver_utils.h"

typedef struct a4988 *a4988_handle_t;

typedef struct {
    pin_clk_t pin_clk;
    pin_dir_t pin_dir;
    micro_step_div_t micro_step_div;
} a4988_config_t;



#ifdef __cplusplus
}
#endif

#endif /* A4988_H_ */