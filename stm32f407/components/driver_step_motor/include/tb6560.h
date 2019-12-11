#ifndef _TB6560_H
#define _TB6560_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../../driver/include/timer.h"
#include "../../driver/include/gpio.h"
#include "step_driver_utils.h"

typedef struct tb6560 *tb6560_handle_t;

typedef struct {
    pin_clk_t           pin_clk;
    pin_dir_t           pin_dir;
    micro_step_div_t    micro_step_div;
    uint8_t             dir;
    uint16_t            speed;
} tb6560_config_t;

tb6560_handle_t tb6560_init(tb6560_config_t *config);
int tb6560_start(tb6560_handle_t handle);
int tb6560_stop(tb6560_handle_t handle);
int tb6560_set_dir(tb6560_handle_t handle, uint8_t dir);
int tb6560_deinit(tb6560_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* _TB6560_H */
