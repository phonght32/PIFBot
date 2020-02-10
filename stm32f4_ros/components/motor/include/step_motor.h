#ifndef _STEP_motor_H_
#define _STEP_motor_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stdbool.h"

#include "../../driver/include/timer.h"
#include "../../driver/include/gpio.h"

typedef struct step_motor *step_motor_handle_t;

typedef struct {
    pwm_config_t pin_clk;
    gpio_config_t pin_dir;
} step_motor_config_t;

step_motor_handle_t step_motor_init(step_motor_config_t *config);
int step_motor_start(step_motor_handle_t handle);
int step_motor_stop(step_motor_handle_t handle);
int step_motor_set_freq(step_motor_handle_t handle, uint32_t freq_hz);
int step_motor_set_dir(step_motor_handle_t handle, bool dir);


#ifdef __cplusplus
}
#endif

#endif /* _STEP_motor_H_ */
