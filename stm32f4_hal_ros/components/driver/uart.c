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

#define UART1_PP1_HW_INFO	{.rcc_apbenr_usarten = RCC_APB2ENR_USART1EN,     \
                             .usart = USART1,                                \
                             .rcc_ahbenr_gpioen_tx = RCC_AHB1ENR_GPIOAEN,    \
                             .rcc_ahbenr_gpioen_rx = RCC_AHB1ENR_GPIOAEN,    \
                             .port_tx = GPIOA,                               \
                             .port_rx = GPIOA,                               \
                             .pin_tx = GPIO_PIN_9,                           \
                             .pin_rx = GPIO_PIN_10,                          \
                             .alternate_func = GPIO_AF7_USART1}

#define UART1_PP2_HW_INFO   {.rcc_apbenr_usarten = RCC_APB2ENR_USART1EN,      \
                             .usart = USART1,                                \
                             .rcc_ahbenr_gpioen_tx = RCC_AHB1ENR_GPIOBEN,    \
                             .rcc_ahbenr_gpioen_rx = RCC_AHB1ENR_GPIOBEN,    \
                             .port_tx = GPIOB,                               \
                             .port_rx = GPIOB,                               \
                             .pin_tx = GPIO_PIN_7,                           \
                             .pin_rx = GPIO_PIN_7,                           \
                             .alternate_func = GPIO_AF7_USART1}

#define UART2_PP1_HW_INFO   {.rcc_apbenr_usarten = RCC_APB1ENR_USART2EN,      \
                             .usart = USART2,                                \
                             .rcc_ahbenr_gpioen_tx = RCC_AHB1ENR_GPIOAEN,    \
                             .rcc_ahbenr_gpioen_rx = RCC_AHB1ENR_GPIOAEN,    \
                             .port_tx = GPIOA,                               \
                             .port_rx = GPIOA,                               \
                             .pin_tx = GPIO_PIN_2,                           \
                             .pin_rx = GPIO_PIN_3,                           \
                             .alternate_func = GPIO_AF7_USART2}

#define UART2_PP2_HW_INFO   {.rcc_apbenr_usarten = RCC_APB1ENR_USART2EN,      \
                             .usart = USART2,                                \
                             .rcc_ahbenr_gpioen_tx = RCC_AHB1ENR_GPIODEN,    \
                             .rcc_ahbenr_gpioen_rx = RCC_AHB1ENR_GPIODEN,    \
                             .port_tx = GPIOD,                               \
                             .port_rx = GPIOD,                               \
                             .pin_tx = GPIO_PIN_5,                           \
                             .pin_rx = GPIO_PIN_6,                           \
                             .alternate_func = GPIO_AF7_USART2}
                                
#define UART3_PP1_HW_INFO   {.rcc_apbenr_usarten = RCC_APB1ENR_USART3EN,      \
                             .usart = USART3,                                \
                             .rcc_ahbenr_gpioen_tx = RCC_AHB1ENR_GPIOBEN,    \
                             .rcc_ahbenr_gpioen_rx = RCC_AHB1ENR_GPIOBEN,    \
                             .port_tx = GPIOB,                               \
                             .port_rx = GPIOB,                               \
                             .pin_tx = GPIO_PIN_10,                          \
                             .pin_rx = GPIO_PIN_11,                          \
                             .alternate_func = GPIO_AF7_USART3}

#define UART3_PP2_HW_INFO   {.rcc_apbenr_usarten = RCC_APB1ENR_USART3EN,      \
                             .usart = USART3,                                \
                             .rcc_ahbenr_gpioen_tx = RCC_AHB1ENR_GPIOCEN,    \
                             .rcc_ahbenr_gpioen_rx = RCC_AHB1ENR_GPIOCEN,    \
                             .port_tx = GPIOC,                               \
                             .port_rx = GPIOC,                               \
                             .pin_tx = GPIO_PIN_10,                          \
                             .pin_rx = GPIO_PIN_11,                          \
                             .alternate_func = GPIO_AF7_USART3}

#define UART3_PP3_HW_INFO   {.rcc_apbenr_usarten = RCC_APB1ENR_USART3EN,      \
                             .usart = USART3,                                \
                             .rcc_ahbenr_gpioen_tx = RCC_AHB1ENR_GPIODEN,    \
                             .rcc_ahbenr_gpioen_rx = RCC_AHB1ENR_GPIODEN,    \
                             .port_tx = GPIOD,                               \
                             .port_rx = GPIOD,                               \
                             .pin_tx = GPIO_PIN_8,                           \
                             .pin_rx = GPIO_PIN_9,                           \
                             .alternate_func = GPIO_AF7_USART3}

#define UART4_PP1_HW_INFO   {.rcc_apbenr_usarten = RCC_APB1ENR_UART4EN,      \
                             .usart = UART4,                                 \
                             .rcc_ahbenr_gpioen_tx = RCC_AHB1ENR_GPIOAEN,    \
                             .rcc_ahbenr_gpioen_rx = RCC_AHB1ENR_GPIOAEN,    \
                             .port_tx = GPIOA,                               \
                             .port_rx = GPIOA,                               \
                             .pin_tx = GPIO_PIN_0,                           \
                             .pin_rx = GPIO_PIN_1,                           \
                             .alternate_func = GPIO_AF8_UART4}

#define UART4_PP2_HW_INFO   {.rcc_apbenr_usarten = RCC_APB1ENR_UART4EN,      \
                             .usart = UART4,                                 \
                             .rcc_ahbenr_gpioen_tx = RCC_AHB1ENR_GPIOCEN,    \
                             .rcc_ahbenr_gpioen_rx = RCC_AHB1ENR_GPIOCEN,    \
                             .port_tx = GPIOC,                               \
                             .port_rx = GPIOC,                               \
                             .pin_tx = GPIO_PIN_10,                          \
                             .pin_rx = GPIO_PIN_11,                          \
                             .alternate_func = GPIO_AF8_UART4}
         
#define UART5_PP1_HW_INFO   {.rcc_apbenr_usarten = RCC_APB1ENR_UART5EN,      \
                             .usart = UART5,                                 \
                             .rcc_ahbenr_gpioen_tx = RCC_AHB1ENR_GPIOCEN,    \
                             .rcc_ahbenr_gpioen_rx = RCC_AHB1ENR_GPIODEN,    \
                             .port_tx = GPIOC,                               \
                             .port_rx = GPIOD,                               \
                             .pin_tx = GPIO_PIN_12,                          \
                             .pin_rx = GPIO_PIN_2,                           \
                             .alternate_func = GPIO_AF8_UART5}

#define UART6_PP1_HW_INFO   {.rcc_apbenr_usarten = RCC_APB2ENR_USART6EN,      \
                             .usart = USART6,                                \
                             .rcc_ahbenr_gpioen_tx = RCC_AHB1ENR_GPIOCEN,    \
                             .rcc_ahbenr_gpioen_rx = RCC_AHB1ENR_GPIOCEN,    \
                             .port_tx = GPIOC,                               \
                             .port_rx = GPIOC,                               \
                             .pin_tx = GPIO_PIN_6,                           \
                             .pin_rx = GPIO_PIN_7,                           \
                             .alternate_func = GPIO_AF8_USART6}

#define UART6_PP2_HW_INFO   {.rcc_apbenr_usarten = RCC_APB2ENR_USART6EN,      \
                             .usart = USART6,                                \
                             .rcc_ahbenr_gpioen_tx = RCC_AHB1ENR_GPIOGEN,    \
                             .rcc_ahbenr_gpioen_rx = RCC_AHB1ENR_GPIOGEN,    \
                             .port_tx = GPIOG,                               \
                             .port_rx = GPIOG,                               \
                             .pin_tx = GPIO_PIN_14,                          \
                             .pin_rx = GPIO_PIN_9,                           \
                             .alternate_func = GPIO_AF8_USART6}

typedef struct {
	uint32_t       rcc_apbenr_usarten;
    USART_TypeDef  *usart;
	uint32_t       rcc_ahbenr_gpioen_tx;
	uint32_t       rcc_ahbenr_gpioen_rx;
	GPIO_TypeDef   *port_tx;
	GPIO_TypeDef   *port_rx;
	uint16_t       pin_tx;
	uint16_t       pin_rx;
	uint8_t        alternate_func;
} uart_hw_info_t;


typedef struct uart {
    uart_num_t         uart_num;
    uart_pins_pack_t   uart_pins_pack;
    uart_hw_info_t     hw_info;
    UART_HandleTypeDef hal_handle;
} uart_t;


uart_hw_info_t UART_HW_INFO_MAPPING[UART_NUM_MAX][UART_PINS_PACK_MAX] = {
    {UART1_PP1_HW_INFO, UART1_PP2_HW_INFO,                 0},
    {UART2_PP1_HW_INFO, UART2_PP2_HW_INFO,                 0},
    {UART3_PP1_HW_INFO, UART3_PP2_HW_INFO, UART3_PP3_HW_INFO},
    {UART4_PP1_HW_INFO, UART4_PP2_HW_INFO,                 0},
    {UART5_PP1_HW_INFO,                 0,                 0},
    {UART6_PP1_HW_INFO, UART6_PP2_HW_INFO,                 0}
};

static uart_hw_info_t uart_get_hw_info(uart_num_t uart_num, uart_pins_pack_t uart_pins_pack)
{
    uart_hw_info_t hw_info;
    hw_info = UART_HW_INFO_MAPPING[uart_num][uart_pins_pack];
    
    return hw_info;
}

static int uart_cleanup(uart_handle_t handle)
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
    handle->hw_info = uart_get_hw_info(config->uart_num, config->uart_pins_pack);

	int err;

	do {
		__IO uint32_t tmpreg = 0x00U;
		if ((config->uart_num == UART_NUM_1) || (config->uart_num == UART_NUM_6)) {
			SET_BIT(RCC->APB2ENR, handle->hw_info.rcc_apbenr_usarten);
			tmpreg = READ_BIT(RCC->APB2ENR, handle->hw_info.rcc_apbenr_usarten);
		}
		else {
			SET_BIT(RCC->APB1ENR, handle->hw_info.rcc_apbenr_usarten);
			tmpreg = READ_BIT(RCC->APB1ENR, handle->hw_info.rcc_apbenr_usarten);
		}
		UNUSED(tmpreg);
	} while (0U);

	do {
		__IO uint32_t tmpreg = 0x00U;
		SET_BIT(RCC->AHB1ENR,  handle->hw_info.rcc_ahbenr_gpioen_tx);
		tmpreg = READ_BIT(RCC->AHB1ENR, handle->hw_info.rcc_ahbenr_gpioen_tx);
		UNUSED(tmpreg);
	} while (0U);
	do {
		__IO uint32_t tmpreg = 0x00U;
		SET_BIT(RCC->AHB1ENR, handle->hw_info.rcc_ahbenr_gpioen_rx);
		tmpreg = READ_BIT(RCC->AHB1ENR, handle->hw_info.rcc_ahbenr_gpioen_rx);
		UNUSED(tmpreg);
	} while (0U);

	handle->hal_handle.Instance = handle->hw_info.usart;
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
	GPIO_InitStruct.Pin = handle->hw_info.pin_tx;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = handle->hw_info.alternate_func;
	HAL_GPIO_Init(handle->hw_info.port_tx, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = handle->hw_info.pin_rx;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = handle->hw_info.alternate_func;
	HAL_GPIO_Init(handle->hw_info.port_rx, &GPIO_InitStruct);

	handle->uart_num = config->uart_num;
	handle->uart_pins_pack = config->uart_pins_pack;

    return handle;
}

int uart_write_bytes(uart_handle_t handle, uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
    if(handle == NULL || data == NULL)
    {
        return -1;
    }

	HAL_UART_Transmit(&handle->hal_handle,data, 7, timeout_ms);
	return 0;
}

int uart_read_bytes(uart_handle_t handle, uint8_t *buf, uint16_t length, uint32_t timeout_ms)
{
    if(handle == NULL || length == NULL)
    {
        return -1;
    }

	return HAL_UART_Receive(&handle->hal_handle, buf, length, timeout_ms);
}

void uart_dma_tx_init(uart_handle_t handle,DMA_HandleTypeDef *hdma_uart_tx)
{
//	DMA_HandleTypeDef *hdma_uart_tx= calloc(1, sizeof(DMA_HandleTypeDef));

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
	__HAL_LINKDMA(&handle->hal_handle, hdmatx, *hdma_uart_tx);

//	hdma_uart_tx->Instance = DMA1_Stream4;
//	    hdma_uart_tx->Init.Channel = DMA_CHANNEL_4;
//	    hdma_uart_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
//	    hdma_uart_tx->Init.PeriphInc = DMA_PINC_DISABLE;
//	    hdma_uart_tx->Init.MemInc = DMA_MINC_ENABLE;
//	    hdma_uart_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//	    hdma_uart_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//	    hdma_uart_tx->Init.Mode = DMA_NORMAL;
//	    hdma_uart_tx->Init.Priority = DMA_PRIORITY_LOW;
//	    hdma_uart_tx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//	    HAL_DMA_Init(hdma_uart_tx);




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

