
#include "stm32f4xx.h"

#include "../components/driver/include/gpio.h"


void delay_01ms(uint16_t period);

int main(void)
{	
	gpio_config_t gpio_cfg;
	gpio_cfg.GPIOx = GPIOD;
	gpio_cfg.GPIO_Pin = GPIO_Pin_12;
	gpio_handle_t gpio_d15 = gpio_output_init(&gpio_cfg);
	while(1)
	{
		gpio_toggle(gpio_d15);
		delay_01ms(1000);
	}
}

void delay_01ms(uint16_t period)
{

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    TIM6->PSC = 8399;       // clk = SystemCoreClock / 4 / (PSC+1) *2 = 10KHz
    TIM6->ARR = period-1;
    TIM6->CNT = 0;
    TIM6->EGR = 1;      // update registers;

    TIM6->SR  = 0;      // clear overflow flag
    TIM6->CR1 = 1;      // enable Timer6

    while (!TIM6->SR);

    TIM6->CR1 = 0;      // stop Timer6
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
}


