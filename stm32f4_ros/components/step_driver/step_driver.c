#include "stm32f4xx_hal.h"

#include "stdlib.h"

#include "include/step_driver.h"

#define PWM_DUTYCYCLE_DEFAULT	30

typedef struct step_driver {
    pwm_handle_t clk_handle;
    gpio_handle_t dir_handle;
} step_driver_t;

static void step_driver_cleanup(step_driver_handle_t handle)
{
	free(handle->dir_handle);
	free(handle->clk_handle);
	free(handle);
}

step_driver_handle_t step_driver_init(step_driver_config_t *config)
{
    step_driver_handle_t handle;
    if( (handle             = calloc(1, sizeof(step_driver_t))) &&
        (handle->clk_handle = calloc(1, sizeof(pwm_handle_t)))  &&
        (handle->dir_handle = calloc(1, sizeof(gpio_handle_t))) == NULL )
    {
    	step_driver_cleanup(handle);
        return -1;
    }

    handle->clk_handle = pwm_init(&config->pin_clk);
    handle->dir_handle = gpio_init(&config->pin_dir);
    pwm_set_duty(handle->clk_handle, PWM_DUTYCYCLE_DEFAULT);

    return handle;
}

int step_driver_start(step_driver_handle_t handle)
{
	pwm_start(handle->clk_handle);

	return 0;
}

int step_driver_stop(step_driver_handle_t handle)
{
	pwm_stop(handle->clk_handle);

	return 0;
}

int step_driver_set_freq(step_driver_handle_t handle, uint32_t freq_hz)
{
	pwm_set_freq(handle->clk_handle,freq_hz);

	return 0;
}

int step_driver_set_dir(step_driver_handle_t handle, bool dir)
{
	gpio_set_level(handle->dir_handle, dir);

	return 0;
}
