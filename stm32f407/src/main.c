
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

#include "../components/driver_step_motor/include/tb6560.h"
#include "../components/driver_step_motor/include/a4988.h"


#include "system_timetick.h"

void delay_01ms(uint16_t period){

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  	TIM6->PSC = 8399;		// clk = SystemCoreClock / 4 / (PSC+1) *2 = 10KHz
  	TIM6->ARR = period-1;
  	TIM6->CNT = 0;
  	TIM6->EGR = 1;		// update registers;

  	TIM6->SR  = 0;		// clear overflow flag
  	TIM6->CR1 = 1;		// enable Timer6

  	while (!TIM6->SR);

  	TIM6->CR1 = 0;		// stop Timer6
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
}

int main(void)
{
    SysTick_Config(SystemCoreClock);

    a4988_config_t a4988_cfg;
    a4988_cfg.pin_clk.pwm_channel = PWM_CHANNEL_1;
    a4988_cfg.pin_clk.pwm_pins_pack = PWM_PINS_PACK_2;
    a4988_cfg.pin_clk.pwm_duty = 50;
    a4988_cfg.pin_clk.timer = TIMER_NUM_4;
    a4988_cfg.pin_clk.timer_prescaler = 0;
    a4988_cfg.pin_clk.timer_period = 0;
    a4988_cfg.pin_dir.GPIOx = GPIOD;
    a4988_cfg.pin_dir.GPIO_Pin = GPIO_Pin_15;
    a4988_cfg.pin_dir.pull_reg = GPIO_PULL_REG_DISABLE;
    a4988_cfg.micro_step_div = MICRO_STEP_DIV16;
    a4988_handle_t a4988_handle = a4988_init(&a4988_cfg);

    a4988_set_freq(a4988_handle,2000);
    a4988_set_dir(a4988_handle,0);
    a4988_start(a4988_handle);

    while (1)
    {
    	a4988_stop(a4988_handle);
    	a4988_toggle_dir(a4988_handle);
    	delay_01ms(5000);
    	a4988_start(a4988_handle);
        delay_01ms(10000);
    }

    return 0;
}




