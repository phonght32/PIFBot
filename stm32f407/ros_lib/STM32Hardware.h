#ifndef _STM32_HARDWARE_H_
#define _STM32_HARDWARE_H_


#include "../components/driver/include/uart.h"

usart_handle_t UartHandle;
uint8_t Buffer_1;
uint8_t *RxBuffer = &Buffer_1;

#define BUF_SIZE 2048

class STM32Hardware
{
private:
	static constexpr uint16_t buf_mask = BUF_SIZE - 1;
	uint8_t tx_buf[BUF_SIZE];
	uint8_t rx_buf[BUF_SIZE];
	bool tx_cplt = true;
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
        }
    // Initialize the ATM32


    // Read a byte of data from ROS connection.
    // If no data , hal_uart-timeout, returns -1
    int read()
    {
//      BSP_LED_Toggle(LED1);
    	while (USART_GetFlagStatus(UART4, USART_FLAG_RXNE) == RESET);
//      return (uart_read_bytes(&UartHandle, RxBuffer, 1)) ? *RxBuffer : -1;
    	return USART_ReceiveData(UART4);;
    }

    // Send a byte of data to ROS connection
    void write(uint8_t* data, int length)
    {

    	uint16_t i;
    	    for (i = 0; i < length; i++)
    	    {
    	        while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
    	        UART4->DR = (uint16_t)(data[i]);
    	        while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
    	    }


//    	if (length > BUF_SIZE || length < 1)
//    			{
//    				return;
//    			}
//
//    			while(!tx_cplt)
//    			{
//
//    			}
//
//    			for (int i = 0; i < length; i++)
//    			{
//    				tx_buf[i] = data[i];
//    			}
//
//    			if(tx_cplt)
//    			{
//    				tx_cplt = false;
//    				 while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
//    				 UART4->DR = (uint16_t)(*tx_buf);
//    			}
    }

    // Returns milliseconds since start of program
    unsigned long time(void)
    {
//      BSP_LED_Toggle(LED3);
//      return HAL_GetTick();
    	return tick_count;
    }

};

#endif


