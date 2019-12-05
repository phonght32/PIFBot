/* Includes ------------------------------------------------------------------*/
#include "stdlib.h"

#include "include/tb6560.h"

/* Internal define -----------------------------------------------------------*/


/* Internal typedef ----------------------------------------------------------*/


/* Internal variable ---------------------------------------------------------*/
typedef struct tb6560 {
    pin_clk_t pin_clk;
    pin_dir_t pin_dir;
    tb6560_micro_step_t micro_step_div;
} tb6560_t;

/* Internal function ---------------------------------------------------------*/


/* External function ---------------------------------------------------------*/
tb6560_handle_t tb6560_init(tb6560_config_t *config)
{
	pwm_init(&config->pin_clk);
	gpio_output_init(&config->pin_dir);

	tb6560_handle_t handle = calloc(1,sizeof(tb6560_t));
	handle->pin_clk.pwm_channel     = config->pin_clk.pwm_channel;
	handle->pin_clk.pwm_pins_pack   = config->pin_clk.pwm_pins_pack;
	handle->pin_clk.pwm_duty        = config->pin_clk.pwm_duty;
	handle->pin_clk.timer           = config->pin_clk.timer;
	handle->pin_clk.timer_period    = config->pin_clk.timer_period;
	handle->pin_clk.timer_prescaler = config->pin_clk.timer_prescaler;
	handle->pin_dir.GPIOx           = config->pin_dir.GPIOx;
	handle->pin_dir.GPIO_Pin        = config->pin_dir.GPIO_Pin;
	handle->micro_step_div          = config->micro_step_div;

	return handle;
}
