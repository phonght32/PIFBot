#include "stm32f4xx.h"
#include "system_timetick.h"
#include "stdint.h"
#include "../components/driver/include/dma.h"
#include "../components/driver/include/uart.h"
#include "../components/driver/include/timer.h"

 #include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

 void led_cb( const std_msgs::UInt16& cmd_msg)
 {
 }
 ros::NodeHandle nh;
 std_msgs::String str_msg;
 ros::Publisher chatter("chatter", &str_msg);
 ros::Subscriber<std_msgs::UInt16> sub("led", led_cb);


//#define		BUFF_SIZE			4
//
uint8_t 	rxbuff[4];
//uint8_t 	a[4*BUFF_SIZE];
//uint16_t	index = 0;
//uint16_t 	rcv_flag = 0;

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

	SysTick_Config(SystemCoreClock/1000);
	STM32Hardware stm32hardware;
	stm32hardware.init();
//	usart_config_t uart_config;
//	  uart_config.usart_baudrate = 57600;
//	  uart_config.usart_num = UART_NUM_4;
//	  uart_config.usart_pins_pack = USART_PINS_PACK_1;
//	  usart_handle_t uart_handle = uart_init(&uart_config);
//
//	  uart_dma_enable_rx(uart_handle);
//
//	  dma_config_t dma_config;
//	  dma_config.buffer = (uint32_t*)rxbuff;
//	  dma_config.buffer_size = 4;
//	  dma_config.dma_channel = DMA_CHANNEL_4;
//	  dma_config.dma_mode = DMA_Mode_Normal;
//	  dma_config.dma_num = DMA_NUM_1;
//	  dma_config.dma_priority = DMA_Priority_High;
//	  dma_config.dma_stream = DMA_STREAM_2;
//	  dma_handle_t dma_handle = dma_init(&dma_config);
//
//	  dma_intr_enable(dma_handle, DMA_IT_TC);

	  nh.initNode();
	  nh.advertise(chatter);
	  nh.subscribe(sub);
	  const char * hello = "Hello World!!";
	  int chatter_interval = 1000.0 / 2;
	  int chatter_last = 100;

	while(1)
	{
//		if(tick_count == 100)
//		{
//			tick_count = 0;
//			if (nh.connected())
//			{
//				str_msg.data = hello;
//				chatter.publish(&str_msg);
//			}
//			nh.spinOnce();
//			delay_01ms(1);
//		}

		if (nh.connected())
		{
			str_msg.data = hello;
			chatter.publish(&str_msg);
		}
		nh.spinOnce();
		delay_01ms(1000);
	}
}



void DMA1_Stream2_IRQHandler(void)
{
//  uint16_t i;
//
//  /* Clear the DMA1_Stream2 TCIF2 pending bit */
//  DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
//
//  for(i=0; i<BUFF_SIZE; i++)
//
//    a[index + i] = rxbuff[i];
//
//	index = index + BUFF_SIZE;
//  rcv_flag = 1;
//
//	DMA_Cmd(DMA1_Stream2, ENABLE);
}


