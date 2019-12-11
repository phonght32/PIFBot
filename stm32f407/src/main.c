
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#include "system_timetick.h"

#include "../components/driver_step_motor/include/tb6560.h"


int main(void)
{

	SystemInit();

    SysTick_Config(SystemCoreClock/100);

    tb6560_config_t tb6560_cfg;
    tb6560_cfg.pin_clk.pwm_channel = PWM_CHANNEL_1;
    tb6560_cfg.pin_clk.pwm_pins_pack = PWM_PINS_PACK_2;
    tb6560_cfg.pin_clk.pwm_duty = 50;
    tb6560_cfg.pin_clk.timer = TIMER_NUM_4;
    tb6560_cfg.pin_clk.timer_prescaler = 9;
    tb6560_cfg.pin_clk.timer_period = 167;
    tb6560_cfg.pin_dir.GPIOx = GPIOD;
    tb6560_cfg.pin_dir.GPIO_Pin = GPIO_Pin_15;
    tb6560_cfg.pin_dir.pull_reg = GPIO_PULL_REG_DISABLE;
    tb6560_cfg.micro_step_div = MICRO_STEP_DIV16;
    tb6560_handle_t tb6560_handle = tb6560_init(&tb6560_cfg);
    tb6560_start(tb6560_handle);
    tb6560_set_dir(tb6560_handle,1);

    while (1)
    {
        if(tick_count == 100)
        {
            tick_count = 0;
        }
    }

    return 0;
}




