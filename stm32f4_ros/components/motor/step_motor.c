#include "include/step_motor.h"

#define PWM_DUTYCYCLE_DEFAULT	30

typedef struct step_motor {
    pwm_handle_t clk_handle;
    gpio_handle_t dir_handle;
} step_motor_t;

static void step_motor_cleanup(step_motor_handle_t handle)
{
	free(handle->dir_handle);
	free(handle->clk_handle);
	free(handle);
}

step_motor_handle_t step_motor_init(step_motor_config_t *config)
{
    step_motor_handle_t handle;
    if( (handle             = calloc(1, sizeof(step_motor_t))) &&
        (handle->clk_handle = calloc(1, sizeof(pwm_handle_t)))  &&
        (handle->dir_handle = calloc(1, sizeof(gpio_handle_t))) == NULL )
    {
    	step_motor_cleanup(handle);
        return -1;
    }

    handle->clk_handle = pwm_init(&config->pin_clk);
    handle->dir_handle = gpio_init(&config->pin_dir);
    pwm_set_duty(handle->clk_handle, PWM_DUTYCYCLE_DEFAULT);

    return handle;
}

int step_motor_start(step_motor_handle_t handle)
{
	pwm_start(handle->clk_handle);

	return 0;
}

int step_motor_stop(step_motor_handle_t handle)
{
	pwm_stop(handle->clk_handle);

	return 0;
}

int step_motor_set_freq(step_motor_handle_t handle, uint32_t freq_hz)
{
	pwm_set_freq(handle->clk_handle,freq_hz);

	return 0;
}

int step_motor_set_dir(step_motor_handle_t handle, bool dir)
{
	gpio_set_level(handle->dir_handle, dir);

	return 0;
}
