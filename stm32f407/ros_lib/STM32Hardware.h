#ifndef _STM32_HARDWARE_H_
#define _STM32_HARDWARE_H_


#include "../components/driver/include/uart.h"

usart_handle_t UartHandle;
uint8_t Buffer_1;
uint8_t *RxBuffer = &Buffer_1;


#define BUF_SIZE 512


uint8_t rx_buf[BUF_SIZE];
uint8_t tx_buf[BUF_SIZE];

uint8_t tx_buf_test[512] = {' ','0','0','3','0',',',
        					' ','0','0','0','0',',',
							' ','0','0','0','0',',',
							' ','0','0','0','4',',',
							' ','0','0','0','5',',',
							' ','0','0','0','6',',',
							' ','0','0','0','7',',',
							' ','0','0','0','8',',',
							' ','0','0','0','9',',',
							' ','0','0','1','0',',',
							' ','0','0','1','1',',',
							' ','0','0','1','3',
							0x0D,0x0A};

class STM32Hardware
{
private:
	static constexpr uint16_t buf_mask = BUF_SIZE - 1;


	uint16_t rx_tail = 0;
	uint16_t tx_head = 0;
	uint16_t tx_tail = 0;
  	public:
    STM32Hardware() {}
    void init()
    {
    	usart_config_t uart_config;
    	uart_config.usart_baudrate = 57600;
    	uart_config.usart_num = UART_NUM_4;
    	uart_config.usart_pins_pack = USART_PINS_PACK_1;
    	UartHandle  = uart_init(&uart_config);
    	uart_dma_enable_rx(UartHandle);
    	uart_dma_enable_tx(UartHandle);

    	dma_config_t dma_config_rx;
    	dma_config_rx.buffer = (uint32_t*)rx_buf;
    	dma_config_rx.buffer_size = BUF_SIZE;
    	dma_config_rx.dma_channel = DMA_CHANNEL_4;
    	dma_config_rx.dma_mode = DMA_Mode_Normal;
    	dma_config_rx.dma_num = DMA_NUM_1;
    	dma_config_rx.dma_priority = DMA_Priority_High;
    	dma_config_rx.dma_stream = DMA_STREAM_2;
    	dma_handle_t dma_handle_rx = dma_init(&dma_config_rx);
    	dma_intr_enable(dma_handle_rx, DMA_IT_TC);

    	dma_config_t dma_config_tx;
    	dma_config_tx.buffer = (uint32_t*)tx_buf_test;
    	dma_config_tx.buffer_size = BUF_SIZE;
    	dma_config_tx.dma_channel = DMA_CHANNEL_4;
    	dma_config_tx.dma_mode = DMA_Mode_Normal;
    	dma_config_tx.dma_num = DMA_NUM_1;
    	dma_config_tx.dma_priority = DMA_Priority_High;
    	dma_config_tx.dma_stream = DMA_STREAM_4;
    	dma_handle_t dma_handle_tx = dma_init(&dma_config_tx);



    	rx_tail = 0;
    }
    // Initialize the ATM32


    // Read a byte of data from ROS connection.
    // If no data , hal_uart-timeout, returns -1
    int read()
    {
    	int c = (int) rx_buf[rx_tail++];
//    			rx_tail &= buf_mask;
    			return c;

//      BSP_LED_Toggle(LED1);
//    	while (USART_GetFlagStatus(UART4, USART_FLAG_RXNE) == RESET);
////      return (uart_read_bytes(&UartHandle, RxBuffer, 1)) ? *RxBuffer : -1;
//    	return USART_ReceiveData(UART4);;
    }

    // Send a byte of data to ROS connection
    void write(uint8_t* data, int length)
    {

//    	uint16_t i;
//    	    for (i = 0; i < length; i++)
//    	    {
//    	        while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
//    	        UART4->DR = (uint16_t)(data[i]);
//    	        while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
//    	    }


    	if (length > BUF_SIZE || length < 1)
    			{
    				return;
    			}


    			for (int i = 0; i < length; i++)
    			{
    				tx_buf[i] = data[i];
    				while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
    				UART4->DR = (uint16_t)tx_buf[i];
    			}

//    				 while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
//    				 UART4->DR = (uint16_t)(*tx_buf);

    }

    // Returns milliseconds since start of program
    unsigned long time(void)
    {
//      BSP_LED_Toggle(LED3);
//      return HAL_GetTick();
    	return tick_count;
    }

};

//void DMA1_Stream2_IRQHandler(void)
//{
////  uint16_t i;
////
////  /* Clear the DMA1_Stream2 TCIF2 pending bit */
////  DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
////
////  for(i=0; i<BUFF_SIZE; i++)
////
////    a[index + i] = rxbuff[i];
////
////	index = index + BUFF_SIZE;
////  rcv_flag = 1;
////
////	DMA_Cmd(DMA1_Stream2, ENABLE);
//}

#endif


