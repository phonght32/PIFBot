/*
 * SerialClass.h
 *
 *  Created on: Aug 15, 2018
 *      Author: yusaku
 */

#ifndef ROSSERIAL_CLIENT_SRC_ROS_LIB_SERIALCLASS_H_
#define ROSSERIAL_CLIENT_SRC_ROS_LIB_SERIALCLASS_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

extern UART_HandleTypeDef huart4;
//extern UART_HandleTypeDef huart2;
//extern UART_HandleTypeDef huart3;
static constexpr uint16_t BUF_SIZE = 2048;

class SerialClass
{
private:
	static constexpr uint16_t buf_mask = BUF_SIZE - 1;
	uint8_t tx_buf[BUF_SIZE];
	uint8_t rx_buf[BUF_SIZE];
	bool tx_cplt = true;
	uint16_t rx_tail = 0;
	uint16_t tx_head = 0;
	uint16_t tx_tail = 0;
	UART_HandleTypeDef &huart;

public:
	SerialClass(UART_HandleTypeDef &huart) : huart(huart)
	{
		//this->huart = huart;
	}

	inline UART_HandleTypeDef * const get_handle(void)
	{
		return &huart;
	}

	inline void start_dma(void)
	{
		tx_cplt = true;
		rx_tail = 0;
		HAL_UART_Receive_DMA(&huart, (uint8_t *) rx_buf, BUF_SIZE);
	}



	inline int read(void)
	{
		uint16_t rx_head = (BUF_SIZE - huart.hdmarx->Instance->NDTR)
				& buf_mask;
		if (rx_tail == rx_head)
		{
			return -1;
		}

		int c = (int) rx_buf[rx_tail++];
		rx_tail &= buf_mask;
		return c;
	}

	inline void write(const uint8_t * const c, const int length)
	{
		if (length > BUF_SIZE || length < 1)
		{
			return;
		}

		while(!tx_cplt)
		{

		}

		for (int i = 0; i < length; i++)
		{
			tx_buf[i] = c[i];
		}

		if(tx_cplt)
		{
			tx_cplt = false;
			HAL_UART_Transmit_DMA(&huart, tx_buf, length);
		}
	}

	inline void tx_cplt_callback(void)
	{
		tx_cplt = true;
	}

    inline void reset_rbuf(void){
      HAL_UART_Receive_DMA(&huart, (uint8_t *) rx_buf, BUF_SIZE);
    }
};

SerialClass serial(huart4);
//SerialClass serial(huart2);
//SerialClass serial(huart3);

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	// comparing pointer
	if (huart->Instance == serial.get_handle()->Instance)
	{
		serial.tx_cplt_callback();
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	}
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	serial.reset_rbuf();
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

}

#endif /* ROSSERIAL_CLIENT_SRC_ROS_LIB_SERIALCLASS_H_ */
