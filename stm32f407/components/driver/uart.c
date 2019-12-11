/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"

#include "stdlib.h"


#include "include/uart.h"


/* Internal define -----------------------------------------------------------*/


/* Internal typedef ----------------------------------------------------------*/
typedef enum {
    USART_PARAM_MAPPING_TX_GPIOx = 0,
    USART_PARAM_MAPPING_TX_GPIO_Pin_x,
    USART_PARAM_MAPPING_TX_RCC_AHBxPeriph_GPIOx,
    USART_PARAM_MAPPING_TX_PinSourcex,
    USART_PARAM_MAPPING_RX_GPIOx,
    USART_PARAM_MAPPING_RX_GPIO_Pin_x,
    USART_PARAM_MAPPING_RX_RCC_AHBxPeriph_GPIOx,
    USART_PARAM_MAPPING_RX_PinSourcex,
    USART_PARAM_MAPPING_GPIO_AF_USARTx,
    USART_PARAM_MAPPING_RCC_APBxPeriph_USARTx,
    USART_PARAM_MAPPING_INDEX_MAX
} usart_param_mapping_index_t;

typedef struct usart {
    usart_num_t         usart_num;
    usart_pins_pack_t   usart_pins_pack;
    uint32_t            usart_baudrate;
} usart_t;


/* Internal variable ---------------------------------------------------------*/
uint32_t USART_PARAM_MAPPING[USART_PINS_PACK_MAX][USART_NUM_MAX][USART_PARAM_MAPPING_INDEX_MAX] = {
    {   {(uint32_t)GPIOA,  GPIO_Pin_9, RCC_AHB1Periph_GPIOA,  GPIO_PinSource9, (uint32_t)GPIOA, GPIO_Pin_10, RCC_AHB1Periph_GPIOA, GPIO_PinSource10, GPIO_AF_USART1, RCC_APB2Periph_USART1},
        {(uint32_t)GPIOA,  GPIO_Pin_2, RCC_AHB1Periph_GPIOA,  GPIO_PinSource2, (uint32_t)GPIOA,  GPIO_Pin_3, RCC_AHB1Periph_GPIOA,  GPIO_PinSource3, GPIO_AF_USART2, RCC_APB1Periph_USART2},
        {(uint32_t)GPIOB, GPIO_Pin_10, RCC_AHB1Periph_GPIOB, GPIO_PinSource10, (uint32_t)GPIOB, GPIO_Pin_11, RCC_AHB1Periph_GPIOB, GPIO_PinSource11, GPIO_AF_USART3, RCC_APB1Periph_USART3},
        {(uint32_t)GPIOA,  GPIO_Pin_0, RCC_AHB1Periph_GPIOA,  GPIO_PinSource0, (uint32_t)GPIOA,  GPIO_Pin_1, RCC_AHB1Periph_GPIOA,  GPIO_PinSource1,  GPIO_AF_UART4,  RCC_APB1Periph_UART4},
        {(uint32_t)GPIOC, GPIO_Pin_12, RCC_AHB1Periph_GPIOC, GPIO_PinSource12, (uint32_t)GPIOD,  GPIO_Pin_2, RCC_AHB1Periph_GPIOD,  GPIO_PinSource2,  GPIO_AF_UART5,  RCC_APB1Periph_UART5},
        {(uint32_t)GPIOC,  GPIO_Pin_6, RCC_AHB1Periph_GPIOC,  GPIO_PinSource6, (uint32_t)GPIOC,  GPIO_Pin_7, RCC_AHB1Periph_GPIOC,  GPIO_PinSource7, GPIO_AF_USART6, RCC_APB2Periph_USART6},
        {(uint32_t)GPIOE,  GPIO_Pin_8, RCC_AHB1Periph_GPIOE,  GPIO_PinSource8, (uint32_t)GPIOE,  GPIO_Pin_7, RCC_AHB1Periph_GPIOE,  GPIO_PinSource7,  GPIO_AF_UART7,  RCC_APB1Periph_UART7},
        {(uint32_t)GPIOE,  GPIO_Pin_1, RCC_AHB1Periph_GPIOE,  GPIO_PinSource1, (uint32_t)GPIOE,  GPIO_Pin_0, RCC_AHB1Periph_GPIOE,  GPIO_PinSource0,  GPIO_AF_UART8,  RCC_APB1Periph_UART8}
    },

    {   {(uint32_t)GPIOB,  GPIO_Pin_6, RCC_AHB1Periph_GPIOB,  GPIO_PinSource6, (uint32_t)GPIOB,  GPIO_Pin_7, RCC_AHB1Periph_GPIOB,  GPIO_PinSource7, GPIO_AF_USART1, RCC_APB2Periph_USART1},
        {(uint32_t)GPIOD,  GPIO_Pin_5, RCC_AHB1Periph_GPIOD,  GPIO_PinSource5, (uint32_t)GPIOD,  GPIO_Pin_6, RCC_AHB1Periph_GPIOD,  GPIO_PinSource6, GPIO_AF_USART2, RCC_APB1Periph_USART2},
        {(uint32_t)GPIOC, GPIO_Pin_10, RCC_AHB1Periph_GPIOC, GPIO_PinSource10, (uint32_t)GPIOC, GPIO_Pin_11, RCC_AHB1Periph_GPIOC, GPIO_PinSource11, GPIO_AF_USART3, RCC_APB1Periph_USART3},
        {(uint32_t)GPIOC, GPIO_Pin_10, RCC_AHB1Periph_GPIOC, GPIO_PinSource10, (uint32_t)GPIOC, GPIO_Pin_11, RCC_AHB1Periph_GPIOC, GPIO_PinSource11,  GPIO_AF_UART4,  RCC_APB1Periph_UART4},
        {              0,           0,                    0,                0,               0,           0,                    0,                0,              0,                     0},
        {(uint32_t)GPIOG, GPIO_Pin_14, RCC_AHB1Periph_GPIOG, GPIO_PinSource14, (uint32_t)GPIOG,  GPIO_Pin_9, RCC_AHB1Periph_GPIOG,  GPIO_PinSource9, GPIO_AF_USART6, RCC_APB2Periph_USART6},
        {(uint32_t)GPIOF,  GPIO_Pin_7, RCC_AHB1Periph_GPIOF,  GPIO_PinSource7, (uint32_t)GPIOF,  GPIO_Pin_6, RCC_AHB1Periph_GPIOF,  GPIO_PinSource6,  GPIO_AF_UART7,  RCC_APB1Periph_UART7},
        {              0,           0,                    0,                0,               0,           0,                    0,                0,              0,                     0}
    },

    {   {              0,           0,                    0,                0,               0,           0,                    0,                0,              0,                     0},
        {              0,           0,                    0,                0,               0,           0,                    0,                0,              0,                     0},
        {(uint32_t)GPIOD,  GPIO_Pin_8, RCC_AHB1Periph_GPIOD,  GPIO_PinSource8, (uint32_t)GPIOD,  GPIO_Pin_9, RCC_AHB1Periph_GPIOD,  GPIO_PinSource9, GPIO_AF_USART3, RCC_APB1Periph_USART3},
        {              0,           0,                    0,                0,               0,           0,                    0,                0,              0,                     0},
        {              0,           0,                    0,                0,               0,           0,                    0,                0,              0,                     0},
        {              0,           0,                    0,                0,               0,           0,                    0,                0,              0,                     0},
        {              0,           0,                    0,                0,               0,           0,                    0,                0,              0,                     0},
        {              0,           0,                    0,                0,               0,           0,                    0,                0,              0,                     0}
    }
};

USART_TypeDef *USARTx_MAPPING[USART_NUM_MAX] = {
    USART1,
    USART2,
    USART3,
    UART4,
    UART5,
    USART6,
    UART7,
    UART8
};



/* Internal function ---------------------------------------------------------*/


/* External function ---------------------------------------------------------*/
usart_handle_t uart_init(usart_config_t *config)
{
    /*Mapping implement */
    GPIO_TypeDef *GPIOx_Tx, *GPIOx_Rx;
    uint16_t GPIO_Pin_x_Tx, GPIO_Pin_x_Rx;
    uint32_t RCC_AHBxPeriph_GPIOx_Tx, RCC_AHBxPeriph_GPIOx_Rx;
    uint8_t GPIO_PinSourcex_Tx, GPIO_PinSourcex_Rx;
    uint8_t GPIO_AF_USARTx;
    uint32_t RCC_APBxPeriph_USARTx;
    USART_TypeDef *USARTx;

    GPIOx_Tx                = (GPIO_TypeDef *)USART_PARAM_MAPPING[config->usart_pins_pack][config->usart_num][USART_PARAM_MAPPING_TX_GPIOx];
    GPIO_Pin_x_Tx           = (uint16_t)      USART_PARAM_MAPPING[config->usart_pins_pack][config->usart_num][USART_PARAM_MAPPING_TX_GPIO_Pin_x];
    RCC_AHBxPeriph_GPIOx_Tx = (uint32_t)      USART_PARAM_MAPPING[config->usart_pins_pack][config->usart_num][USART_PARAM_MAPPING_TX_RCC_AHBxPeriph_GPIOx];
    GPIO_PinSourcex_Tx      = (uint8_t)       USART_PARAM_MAPPING[config->usart_pins_pack][config->usart_num][USART_PARAM_MAPPING_TX_PinSourcex];
    GPIOx_Rx                = (GPIO_TypeDef *)USART_PARAM_MAPPING[config->usart_pins_pack][config->usart_num][USART_PARAM_MAPPING_RX_GPIOx];
    GPIO_Pin_x_Rx           = (uint16_t)      USART_PARAM_MAPPING[config->usart_pins_pack][config->usart_num][USART_PARAM_MAPPING_RX_GPIO_Pin_x];
    RCC_AHBxPeriph_GPIOx_Rx = (uint32_t)      USART_PARAM_MAPPING[config->usart_pins_pack][config->usart_num][USART_PARAM_MAPPING_RX_RCC_AHBxPeriph_GPIOx];
    GPIO_PinSourcex_Rx      = (uint8_t)       USART_PARAM_MAPPING[config->usart_pins_pack][config->usart_num][USART_PARAM_MAPPING_RX_PinSourcex];
    GPIO_AF_USARTx          = (uint8_t)       USART_PARAM_MAPPING[config->usart_pins_pack][config->usart_num][USART_PARAM_MAPPING_GPIO_AF_USARTx];
    RCC_APBxPeriph_USARTx   = (uint32_t)      USART_PARAM_MAPPING[config->usart_pins_pack][config->usart_num][USART_PARAM_MAPPING_RCC_APBxPeriph_USARTx];
    USARTx                  =                 USARTx_MAPPING[config->usart_num];

    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHBxPeriph_GPIOx_Tx, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHBxPeriph_GPIOx_Rx, ENABLE);

    if ((config->usart_num == USART_NUM_1) || (config->usart_num == USART_NUM_6))
    {
        RCC_APB2PeriphClockCmd(RCC_APBxPeriph_USARTx, ENABLE);
    }
    else
    {
        RCC_APB1PeriphClockCmd(RCC_APBxPeriph_USARTx, ENABLE);
    }

    GPIO_PinAFConfig(GPIOx_Tx, GPIO_PinSourcex_Tx, GPIO_AF_USARTx);
    GPIO_PinAFConfig(GPIOx_Rx, GPIO_PinSourcex_Rx, GPIO_AF_USARTx);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_x_Tx;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx_Tx, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_x_Rx;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_Init(GPIOx_Rx, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = config->usart_baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USARTx, &USART_InitStructure);

    USART_Cmd(USARTx, ENABLE);

    usart_handle_t handle  = calloc(1, sizeof(usart_t));
    if(!handle)
    {
        return -1;
    }
    handle->usart_num = config->usart_num;
    handle->usart_pins_pack = config->usart_pins_pack;
    handle->usart_baudrate = config->usart_baudrate;

    return handle;
}

int uart_write_bytes(usart_handle_t handle, uint8_t *data, uint16_t length)
{
    uint16_t i;
    for (i = 0; i < length; i++)
    {
        while (USART_GetFlagStatus(USARTx_MAPPING[handle->usart_num], USART_FLAG_TXE) == RESET);
        USARTx_MAPPING[handle->usart_num]->DR = (uint16_t)(data[i]);
        while (USART_GetFlagStatus(USARTx_MAPPING[handle->usart_num], USART_FLAG_TXE) == RESET);
    }

    return 0;
}

int uart_read_bytes(usart_handle_t handle, uint8_t *buffer, uint16_t length)
{
    int i = 0;
    for (i = 0; i < length; i++)
    {
        while (USART_GetFlagStatus(USARTx_MAPPING[handle->usart_num], USART_FLAG_RXNE) == RESET);
        buffer[i++] = (uint8_t)USART_ReceiveData(USARTx_MAPPING[handle->usart_num]);
    }

    return 0;
}

int uart_dma_enable_rx(usart_handle_t handle)
{
    USART_DMACmd(USARTx_MAPPING[handle->usart_num], USART_DMAReq_Rx, ENABLE);

    return 0;
}

int uart_dma_enable_tx(usart_handle_t handle)
{
    USART_DMACmd(USARTx_MAPPING[handle->usart_num], USART_DMAReq_Tx, ENABLE);

    return 0;
}

int uart_deinit(usart_handle_t handle)
{
    USART_DeInit(USARTx_MAPPING[handle->usart_num]);
    free(handle);

    return 0;
}




