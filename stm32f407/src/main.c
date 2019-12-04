
#include "stm32f4xx.h"
//#include "system_timetick.h"
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"

#include "../components/driver/include/uart.h"

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
uint8_t data_send[10];
int main(void)
{
	usart_config_t config;
	config.usart_baudrate = 115200;
	config.usart_num = UART_NUM_4;
	config.usart_pins_pack = USART_PINS_PACK_2;
	usart_handle_t handle = uart_init(&config);

//	uint8_t *data_send = calloc(10, sizeof(uint8_t));

	while(1)
	{

//		while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
//		uart_write_bytes(handle,data_send,12);

		uart_read_bytes(handle, data_send, 8);
		uart_write_bytes(handle,data_send, 8);


//		delay_01ms(1000);
	}
}


