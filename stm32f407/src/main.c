
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
    pwm_cfg.timer 			= TIMER_NUM_4;
    pwm_cfg.pwm_pins_pack 	= PWM_PINS_PACK_2;
    pwm_cfg.pwm_channel 	= PWM_CHANNEL_1;
    pwm_cfg.duty_percent 	= 50;
    pwm_cfg.freq_hz 		= 1;
    pwm_handle_t pwm_handle = pwm_init(&pwm_cfg);
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




