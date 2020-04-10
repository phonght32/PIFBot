// MIT License

// Copyright (c) 2020 thanhphong98 & thuanpham98

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef ROSSERIAL_CLIENT_SRC_ROS_LIB_SERIALCLASS_H_
#define ROSSERIAL_CLIENT_SRC_ROS_LIB_SERIALCLASS_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

static constexpr uint16_t BUF_SIZE = 4096;

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

	inline void init(void)
	{
		__HAL_RCC_GPIOH_CLK_ENABLE();
		__HAL_RCC_UART4_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_DMA1_CLK_ENABLE();

		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		huart4.Instance = UART4;
		huart4.Init.BaudRate = 57600;
		huart4.Init.WordLength = UART_WORDLENGTH_8B;
		huart4.Init.StopBits = UART_STOPBITS_1;
		huart4.Init.Parity = UART_PARITY_NONE;
		huart4.Init.Mode = UART_MODE_TX_RX;
		huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart4.Init.OverSampling = UART_OVERSAMPLING_16;
		HAL_UART_Init(&huart4);

		hdma_uart4_rx.Instance = DMA1_Stream2;
		hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_uart4_rx.Init.Mode = DMA_NORMAL;
		hdma_uart4_rx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_uart4_rx);
		__HAL_LINKDMA(&huart4, hdmarx, hdma_uart4_rx);

		hdma_uart4_tx.Instance = DMA1_Stream4;
		hdma_uart4_tx.Init.Channel = DMA_CHANNEL_4;
		hdma_uart4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_uart4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_uart4_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_uart4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_uart4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_uart4_tx.Init.Mode = DMA_NORMAL;
		hdma_uart4_tx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_uart4_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_uart4_tx);
		__HAL_LINKDMA(&huart4, hdmatx, hdma_uart4_tx);

		HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(UART4_IRQn);

		HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

		HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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

		while (!tx_cplt)
		{

		}

		for (int i = 0; i < length; i++)
		{
			tx_buf[i] = c[i];
		}

		if (tx_cplt)
		{
			tx_cplt = false;
			HAL_UART_Transmit_DMA(&huart, tx_buf, length);
		}
	}

	inline void tx_cplt_callback(void)
	{
		tx_cplt = true;
	}

	inline void reset_rbuf(void) {
		HAL_UART_Receive_DMA(&huart, (uint8_t *) rx_buf, BUF_SIZE);
	}
};

SerialClass serial(huart4);

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

extern "C" void DMA1_Stream2_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_uart4_rx);
}

extern "C" void DMA1_Stream4_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_uart4_tx);
}

extern "C" void UART4_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart4);
}

#endif /* ROSSERIAL_CLIENT_SRC_ROS_LIB_SERIALCLASS_H_ */
