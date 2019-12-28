#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"

#include "stdlib.h"

#include "include/uart.h"


#define UART_WORDLENGTH_DEFAULT 		UART_WORDLENGTH_8B
#define UART_STOPBITS_DEFAULT 			UART_STOPBITS_1
#define UART_PARITY_DEFAULT 			UART_PARITY_NONE
#define UART_HW_FLOWCTRL_DEFAULT 		UART_HWCONTROL_NONE
#define UART_OVERSAMPLING_DEFAULT 		UART_OVERSAMPLING_16
#define UART_MODE_DEFAULT 				UART_MODE_TX_RX


typedef enum {
	USART_PARAM_MAPPING_RCC_AHB1ENR_GPIOAEN_TX,
	USART_PARAM_MAPPING_RCC_AHB1ENR_GPIOAEN_RX,
	USART_PARAM_MAPPING_GPIO_PIN_x_TX,
	USART_PARAM_MAPPING_GPIO_PIN_x_RX,
	USART_PARAM_MAPPING_GPIO_AFx_USARTx,
	USART_PARAM_MAPPING_GPIOx_TX,
	USART_PARAM_MAPPING_GPIOx_RX,
	USART_PARAM_MAPPING_INDEX_MAX
} usart_param_mapping_index_t;

typedef struct uart {
    usart_num_t         uart_num;
    usart_pins_pack_t   uart_pins_pack;
    UART_HandleTypeDef  hal_handle;
} uart_t;

uint32_t USART_PARAM_MAPPING_PP1[USART_NUM_MAX][USART_PARAM_MAPPING_INDEX_MAX] = {
    {RCC_AHB1ENR_GPIOAEN, RCC_AHB1ENR_GPIOAEN, GPIO_PIN_9 , GPIO_PIN_10, GPIO_AF7_USART1, (uint32_t)GPIOA, (uint32_t)GPIOA},
    {RCC_AHB1ENR_GPIOAEN, RCC_AHB1ENR_GPIOAEN, GPIO_PIN_2 , GPIO_PIN_3 , GPIO_AF7_USART2, (uint32_t)GPIOA, (uint32_t)GPIOA},
    {RCC_AHB1ENR_GPIOBEN, RCC_AHB1ENR_GPIOBEN, GPIO_PIN_10, GPIO_PIN_11, GPIO_AF7_USART3, (uint32_t)GPIOB, (uint32_t)GPIOB},
    {RCC_AHB1ENR_GPIOAEN, RCC_AHB1ENR_GPIOAEN, GPIO_PIN_0 , GPIO_PIN_1 , GPIO_AF8_UART4 , (uint32_t)GPIOA, (uint32_t)GPIOA},
    {RCC_AHB1ENR_GPIOCEN, RCC_AHB1ENR_GPIODEN, GPIO_PIN_12, GPIO_PIN_2 , GPIO_AF8_UART5 , (uint32_t)GPIOC, (uint32_t)GPIOD},
    {RCC_AHB1ENR_GPIOCEN, RCC_AHB1ENR_GPIOCEN, GPIO_PIN_6 , GPIO_PIN_7 , GPIO_AF8_USART6, (uint32_t)GPIOC, (uint32_t)GPIOC}
};

uint32_t USART_PARAM_MAPPING_PP2[USART_NUM_MAX][USART_PARAM_MAPPING_INDEX_MAX] = {
    {RCC_AHB1ENR_GPIOBEN, RCC_AHB1ENR_GPIOBEN, GPIO_PIN_6 , GPIO_PIN_7 , GPIO_AF7_USART1, (uint32_t)GPIOB, (uint32_t)GPIOB},
    {RCC_AHB1ENR_GPIODEN, RCC_AHB1ENR_GPIODEN, GPIO_PIN_5 , GPIO_PIN_6 , GPIO_AF7_USART2, (uint32_t)GPIOD, (uint32_t)GPIOD},
    {RCC_AHB1ENR_GPIOCEN, RCC_AHB1ENR_GPIOCEN, GPIO_PIN_10, GPIO_PIN_11, GPIO_AF7_USART3, (uint32_t)GPIOC, (uint32_t)GPIOC},
    {RCC_AHB1ENR_GPIOCEN, RCC_AHB1ENR_GPIOCEN, GPIO_PIN_10, GPIO_PIN_11, GPIO_AF8_UART4 , (uint32_t)GPIOC, (uint32_t)GPIOC},
    {                  0,                   0,           0,           0,               0,               0,               0},
    {                  0,                   0,           0,           0,               0,               0,               0}
};

uint32_t USART_PARAM_MAPPING_PP3[USART_NUM_MAX][USART_PARAM_MAPPING_INDEX_MAX] = {
    {                  0,                   0,           0,           0,               0,               0,               0},
    {                  0,                   0,           0,           0,               0,               0,               0},
    {RCC_AHB1ENR_GPIODEN, RCC_AHB1ENR_GPIODEN, GPIO_PIN_8 , GPIO_PIN_9 , GPIO_AF7_USART3, (uint32_t)GPIOD, (uint32_t)GPIOD},
    {                  0,                   0,           0,           0,               0,               0,               0},
    {                  0,                   0,           0,           0,               0,               0,               0},
    {                  0,                   0,           0,           0,               0,               0,               0}
};

uint32_t RCC_APBxENR_USARTxEN_MAPPING[USART_NUM_MAX] ={
	RCC_APB2ENR_USART1EN,
	RCC_APB1ENR_USART2EN,
	RCC_APB1ENR_USART3EN,
	RCC_APB1ENR_UART4EN,
	RCC_APB1ENR_UART5EN,
	RCC_APB2ENR_USART6EN
};

USART_TypeDef *USARTx_MAPPING[USART_NUM_MAX] = {
	USART1,
	USART2,
	USART3,
	UART4,
	UART5,
	USART6
};

int uart_cleanup(uart_handle_t handle)
{
	free(handle);

	return 0;
}


uart_handle_t uart_init(uart_config_t *config)
{
	uart_handle_t handle;
	handle = calloc(1, sizeof(uart_t));
	if (handle == NULL)
	{
		return -1;
	}

	int err;

	uint32_t RCC_APBxENR_USARTxEN;
	USART_TypeDef *USARTx;
	uint32_t RCC_APB1ENR_GPIOxEN_TX;
	uint32_t RCC_APB1ENR_GPIOxEN_RX;
	uint16_t GPIO_PIN_x_TX;
	uint16_t GPIO_PIN_x_RX;
	uint8_t GPIO_AFx_USARTx;
	GPIO_TypeDef *GPIOx_TX;
	GPIO_TypeDef *GPIOx_RX;

	if(config->uart_pins_pack == USART_PINS_PACK_1)
	{
		RCC_APB1ENR_GPIOxEN_TX = (uint32_t)      USART_PARAM_MAPPING_PP1[config->uart_num][USART_PARAM_MAPPING_RCC_AHB1ENR_GPIOAEN_TX];
		RCC_APB1ENR_GPIOxEN_RX = (uint32_t)      USART_PARAM_MAPPING_PP1[config->uart_num][USART_PARAM_MAPPING_RCC_AHB1ENR_GPIOAEN_RX];
		GPIO_PIN_x_TX          = (uint16_t)      USART_PARAM_MAPPING_PP1[config->uart_num][USART_PARAM_MAPPING_GPIO_PIN_x_TX];
		GPIO_PIN_x_RX          = (uint16_t)      USART_PARAM_MAPPING_PP1[config->uart_num][USART_PARAM_MAPPING_GPIO_PIN_x_RX];
		GPIO_AFx_USARTx        = (uint8_t)       USART_PARAM_MAPPING_PP1[config->uart_num][USART_PARAM_MAPPING_GPIO_AFx_USARTx];
		GPIOx_TX               = (GPIO_TypeDef *)USART_PARAM_MAPPING_PP1[config->uart_num][USART_PARAM_MAPPING_GPIOx_TX];
		GPIOx_RX               = (GPIO_TypeDef *)USART_PARAM_MAPPING_PP1[config->uart_num][USART_PARAM_MAPPING_GPIOx_RX];
	}
	if(config->uart_pins_pack == USART_PINS_PACK_2)
	{
		RCC_APB1ENR_GPIOxEN_TX = (uint32_t)      USART_PARAM_MAPPING_PP2[config->uart_num][USART_PARAM_MAPPING_RCC_AHB1ENR_GPIOAEN_TX];
		RCC_APB1ENR_GPIOxEN_RX = (uint32_t)      USART_PARAM_MAPPING_PP2[config->uart_num][USART_PARAM_MAPPING_RCC_AHB1ENR_GPIOAEN_RX];
		GPIO_PIN_x_TX          = (uint16_t)      USART_PARAM_MAPPING_PP2[config->uart_num][USART_PARAM_MAPPING_GPIO_PIN_x_TX];
		GPIO_PIN_x_RX          = (uint16_t)      USART_PARAM_MAPPING_PP2[config->uart_num][USART_PARAM_MAPPING_GPIO_PIN_x_RX];
		GPIO_AFx_USARTx        = (uint8_t)       USART_PARAM_MAPPING_PP2[config->uart_num][USART_PARAM_MAPPING_GPIO_AFx_USARTx];
		GPIOx_TX               = (GPIO_TypeDef *)USART_PARAM_MAPPING_PP2[config->uart_num][USART_PARAM_MAPPING_GPIOx_TX];
		GPIOx_RX               = (GPIO_TypeDef *)USART_PARAM_MAPPING_PP2[config->uart_num][USART_PARAM_MAPPING_GPIOx_RX];
	}
	if(config->uart_pins_pack == USART_PINS_PACK_2)
	{
		RCC_APB1ENR_GPIOxEN_TX = (uint32_t)      USART_PARAM_MAPPING_PP3[config->uart_num][USART_PARAM_MAPPING_RCC_AHB1ENR_GPIOAEN_TX];
		RCC_APB1ENR_GPIOxEN_RX = (uint32_t)      USART_PARAM_MAPPING_PP3[config->uart_num][USART_PARAM_MAPPING_RCC_AHB1ENR_GPIOAEN_RX];
		GPIO_PIN_x_TX          = (uint16_t)      USART_PARAM_MAPPING_PP3[config->uart_num][USART_PARAM_MAPPING_GPIO_PIN_x_TX];
		GPIO_PIN_x_RX          = (uint16_t)      USART_PARAM_MAPPING_PP3[config->uart_num][USART_PARAM_MAPPING_GPIO_PIN_x_RX];
		GPIO_AFx_USARTx        = (uint8_t)       USART_PARAM_MAPPING_PP3[config->uart_num][USART_PARAM_MAPPING_GPIO_AFx_USARTx];
		GPIOx_TX               = (GPIO_TypeDef *)USART_PARAM_MAPPING_PP3[config->uart_num][USART_PARAM_MAPPING_GPIOx_TX];
		GPIOx_RX               = (GPIO_TypeDef *)USART_PARAM_MAPPING_PP3[config->uart_num][USART_PARAM_MAPPING_GPIOx_RX];
	}

	RCC_APBxENR_USARTxEN = RCC_APBxENR_USARTxEN_MAPPING[config->uart_num];
	USARTx = USARTx_MAPPING[config->uart_num];

	do {
		__IO uint32_t tmpreg = 0x00U;
		if ((config->uart_num == USART_NUM_1) || (config->uart_num == USART_NUM_6)) {
			SET_BIT(RCC->APB2ENR, RCC_APBxENR_USARTxEN);
			tmpreg = READ_BIT(RCC->APB2ENR, RCC_APBxENR_USARTxEN);
		}
		else {
			SET_BIT(RCC->APB1ENR, RCC_APBxENR_USARTxEN);
			tmpreg = READ_BIT(RCC->APB1ENR, RCC_APBxENR_USARTxEN);
		}
		UNUSED(tmpreg);
	} while (0U);

	do {
		__IO uint32_t tmpreg = 0x00U;
		SET_BIT(RCC->AHB1ENR,  RCC_APB1ENR_GPIOxEN_TX);
		tmpreg = READ_BIT(RCC->AHB1ENR, RCC_APB1ENR_GPIOxEN_TX);
		UNUSED(tmpreg);
	} while (0U);
	do {
		__IO uint32_t tmpreg = 0x00U;
		SET_BIT(RCC->AHB1ENR, RCC_APB1ENR_GPIOxEN_RX);
		tmpreg = READ_BIT(RCC->AHB1ENR, RCC_APB1ENR_GPIOxEN_RX);
		UNUSED(tmpreg);
	} while (0U);


	handle->hal_handle.Instance = USARTx;
	handle->hal_handle.Init.BaudRate = config->baudrate;
	handle->hal_handle.Init.WordLength = UART_WORDLENGTH_DEFAULT;
	handle->hal_handle.Init.StopBits = UART_STOPBITS_DEFAULT;
	handle->hal_handle.Init.Parity = UART_PARITY_DEFAULT;
	handle->hal_handle.Init.Mode = UART_MODE_DEFAULT;
	handle->hal_handle.Init.HwFlowCtl = UART_HW_FLOWCTRL_DEFAULT;
	handle->hal_handle.Init.OverSampling = UART_OVERSAMPLING_DEFAULT;
	err = HAL_UART_Init(&handle->hal_handle);
	if(err != HAL_OK)
	{
		uart_cleanup(handle);
		return -1;
	}

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_x_TX;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AFx_USARTx;
	HAL_GPIO_Init(GPIOx_TX, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_x_RX;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AFx_USARTx;
	HAL_GPIO_Init(GPIOx_RX, &GPIO_InitStruct);

	handle->uart_num = config->uart_num;
	handle->uart_pins_pack = config->uart_pins_pack;

    return handle;
}

int uart_write_bytes(uart_handle_t handle, uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
	HAL_UART_Transmit(&handle->hal_handle,data, 7, 100);

	return 0;
}

int uart_read_bytes(uart_handle_t handle, uint8_t *buf, uint16_t length, uint32_t timeout_ms)
{
	return HAL_UART_Receive(&handle->hal_handle, buf, length, timeout_ms);
}

DMA_HandleTypeDef uart_dma_tx_init(uart_handle_t handle)
{
	DMA_HandleTypeDef *hdma_uart_tx= calloc(1, sizeof(DMA_HandleTypeDef));

//	hdma_uart_tx->Instance = DMA1_Stream4;
//	hdma_uart_tx->Init.Channel = DMA_CHANNEL_4;
//	hdma_uart_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
//	hdma_uart_tx->Init.PeriphInc = DMA_PINC_DISABLE;
//	hdma_uart_tx->Init.MemInc = DMA_MINC_ENABLE;
//	hdma_uart_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//	hdma_uart_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//	hdma_uart_tx->Init.Mode = DMA_NORMAL;
//	hdma_uart_tx->Init.Priority = DMA_PRIORITY_LOW;
//	hdma_uart_tx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//	HAL_DMA_Init(hdma_uart_tx);
//	__HAL_LINKDMA(&handle->hal_handle, hdmatx, *hdma_uart_tx);

	hdma_uart_tx->Instance = DMA1_Stream4;
	    hdma_uart_tx->Init.Channel = DMA_CHANNEL_4;
	    hdma_uart_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
	    hdma_uart_tx->Init.PeriphInc = DMA_PINC_DISABLE;
	    hdma_uart_tx->Init.MemInc = DMA_MINC_ENABLE;
	    hdma_uart_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	    hdma_uart_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	    hdma_uart_tx->Init.Mode = DMA_NORMAL;
	    hdma_uart_tx->Init.Priority = DMA_PRIORITY_LOW;
	    hdma_uart_tx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	    HAL_DMA_Init(hdma_uart_tx);


	    __HAL_LINKDMA(&handle->hal_handle,hdmatx,*hdma_uart_tx);

	return *hdma_uart_tx;
}

DMA_HandleTypeDef uart_dma_rx_init(uart_handle_t handle)
{
	DMA_HandleTypeDef hdma_uart_rx;
	hdma_uart_rx.Instance = DMA1_Stream2;
	hdma_uart_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_uart_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_uart_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_uart_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_uart_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_uart_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_uart_rx.Init.Mode = DMA_NORMAL;
	hdma_uart_rx.Init.Priority = DMA_PRIORITY_HIGH;
	hdma_uart_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&hdma_uart_rx);
	__HAL_LINKDMA(&handle->hal_handle, hdmarx, hdma_uart_rx);

	return hdma_uart_rx;
}

int uart_dma_write(uart_handle_t handle, uint8_t *data, uint32_t length)
{
	HAL_UART_Transmit_DMA(&handle->hal_handle, data, length);

	return 0;
}

UART_HandleTypeDef uart_get_hal_handle(uart_handle_t handle)
{
	return handle->hal_handle;
}

