#ifndef _TB6560_H
#define _TB6560_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../../driver/include/timer.h"
#include "../../driver/include/gpio.h"

typedef struct tb6560 *tb6560_handle_t;
typedef gpio_config_t pin_dir_t;
typedef pwm_config_t pin_clk_t;

typedef enum {
    MICRO_STEP_DIV1 = 0,
    MICRO_STEP_DIV2,
    MICRO_STEP_DIV8,
    MICRO_STEP_DIV16
} tb6560_micro_step_t;

typedef struct {
    pin_clk_t pin_clk;
    pin_dir_t pin_dir;
    tb6560_micro_step_t micro_step_div;
} tb6560_config_t;

tb6560_handle_t tb6560_init(tb6560_config_t *config);
int tb6560_start(tb6560_handle_t handle);
int tb6560_stop(tb6560_handle_t handle);
int tb6560_deinit(tb6560_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* _TB6560_H */
