#ifndef _STEP_DRIVER_H_
#define _STEP_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stdbool.h"

#include "../../driver/include/timer.h"
#include "../../driver/include/gpio.h"

typedef struct step_driver *step_driver_handle_t;

typedef struct {
    pwm_config_t pin_clk;
    gpio_config_t pin_dir;
} step_driver_config_t;

step_driver_handle_t step_driver_init(step_driver_config_t *config);
int step_driver_start(step_driver_handle_t handle);
int step_driver_stop(step_driver_handle_t handle);
int step_driver_set_freq(step_driver_handle_t handle, uint32_t freq_hz);
int step_driver_set_dir(step_driver_handle_t handle, bool dir);


#ifdef __cplusplus
}
#endif

#endif /* _STEP_DRIVER_H_ */
