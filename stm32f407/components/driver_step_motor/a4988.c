/* Includes ------------------------------------------------------------------*/
#include "stdlib.h"

#include "include/a4988.h"

/* Internal define -----------------------------------------------------------*/


/* Internal typedef ----------------------------------------------------------*/
typedef struct a4988 {
    pin_clk_t *pin_clk;
    pin_dir_t *pin_dir;
    micro_step_div_t micro_step_div;
    uint8_t dir;
    uint16_t speed;
} a4988_t;

/* Internal function ---------------------------------------------------------*/
static int a4988_cleanup(a4988_handle_t handle);

/* External function ---------------------------------------------------------*/
a4988_handle_t a4988_init(a4988_config_t *config)
{
    a4988_handle_t handle;
    if ( (handle          = calloc(1, sizeof(a4988_t)))   &&
         (handle->pin_clk = calloc(1, sizeof(pin_clk_t)))  &&
         (handle->pin_dir = calloc(1, sizeof(pin_dir_t)))  == NULL)
    {
        a4988_cleanup(handle);
        return -1;
    }

    handle->pin_clk = (pin_clk_t *)pwm_init(&config->pin_clk);
    handle->pin_dir = (pin_dir_t *)gpio_output_init(&config->pin_dir);
    handle->micro_step_div = config->micro_step_div;
    handle->dir = config->dir;
    handle->speed = config->speed;

    return handle;
}

static int a4988_cleanup(a4988_handle_t handle)
{
	free(handle->pin_clk);
	free(handle->pin_dir);
	free(handle);

	return 0;
}
