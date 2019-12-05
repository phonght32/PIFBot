/* Includes ------------------------------------------------------------------*/
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
int tb6560_init(tb6560_config_t *config)
{
	pwm_init(&config->pin_clk);
	gpio_output_init(&config->pin_dir);
	return 0;
}
