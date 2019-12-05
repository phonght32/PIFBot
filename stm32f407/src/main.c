
#include "stm32f4xx.h"
#include "system_timetick.h"

#include "../components/driver/include/dma.h"
#include "../components/driver/include/uart.h"
#include "../components/driver/include/timer.h"






void init_main(void);

int main(void)
{

	SysTick_Config(SystemCoreClock/100);

	init_main();

	while(1){
		if(tick_count == 100){
			tick_count = 0;

		}
	}
}

void init_main(void)
{
  usart_config_t uart_config;
  uart_config.usart_baudrate = 115200;
  uart_config.usart_num = UART_NUM_4;
  uart_config.usart_pins_pack = USART_PINS_PACK_1;
  usart_handle_t uart_handle = uart_init(&uart_config);

  uart_dma_enable_rx(uart_handle);

  dma_config_t dma_config;
//  dma_config.buffer = rxbuff;
  dma_config.buffer_size = 4;
  dma_config.dma_channel = DMA_CHANNEL_4;
  dma_config.dma_mode = DMA_Mode_Normal;
  dma_config.dma_num = DMA_NUM_1;
  dma_config.dma_priority = DMA_Priority_High;
  dma_config.dma_stream = DMA_STREAM_2;
  dma_handle_t dma_handle = dma_init(&dma_config);

  dma_intr_enable(dma_handle, DMA_IT_TC);
}


