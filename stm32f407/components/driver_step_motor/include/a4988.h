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
    uint8_t dir;
    uint16_t speed;
} a4988_config_t;

a4988_handle_t a4988_init(a4988_config_t *config);
int a4988_start(a4988_handle_t handle);
int a4988_stop(a4988_handle_t handle);
int a4988_set_dir(a4988_handle_t handle, uint8_t dir);
int a4988_toggle_dir(a4988_handle_t handle);
int a4988_deinit(a4988_handle_t handle);


#ifdef __cplusplus
}
#endif

#endif /* A4988_H_ */
