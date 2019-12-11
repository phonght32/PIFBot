
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#include "system_timetick.h"

#include "../components/driver/include/timer.h"


int main(void)
{

	SystemInit();

    SysTick_Config(SystemCoreClock/100);

    pwm_config_t pwm_cfg;
    pwm_cfg.timer 			= TIMER_NUM_2;
    pwm_cfg.pwm_pins_pack 	= PWM_PINS_PACK_2;
    pwm_cfg.pwm_channel 	= PWM_CHANNEL_3;
    pwm_cfg.pwm_duty 		= 50;
    pwm_cfg.timer_prescaler = 83;
    pwm_cfg.timer_period 	= 9999;

    pwm_handle_t pwm_handle = pwm_init(&pwm_cfg);
    pwm_set_freq(pwm_handle, 100);
    pwm_set_duty(pwm_handle, 50);
    pwm_start(pwm_handle);


    while (1)
    {
        if(tick_count == 100)
        {
            tick_count = 0;
        }
    }

    return 0;
}




