/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_rcc.h"

#include "stdlib.h"

#include "include/uart.h"


/* Internal define -----------------------------------------------------------*/


/* Internal typedef ----------------------------------------------------------*/


/* Internal variable ---------------------------------------------------------*/
uint32_t UART_PARAM_MAPPING[USART_PINS_PACK_MAX][USART_NUM_MAX] = {
    {   {(uint32_t)GPIOA,  GPIO_Pin_9, RCC_AHB1Periph_GPIOA,  GPIO_PinSource9, (uint32_t)GPIOA, GPIO_Pin_10, RCC_AHB1Periph_GPIOA, GPIO_PinSource10, GPIO_AF_USART1},
        {(uint32_t)GPIOA,  GPIO_Pin_2, RCC_AHB1Periph_GPIOA,  GPIO_PinSource2, (uint32_t)GPIOA,  GPIO_Pin_3, RCC_AHB1Periph_GPIOA,  GPIO_PinSource3, GPIO_AF_USART2},
        {(uint32_t)GPIOB, GPIO_Pin_10, RCC_AHB1Periph_GPIOB, GPIO_PinSource10, (uint32_t)GPIOB, GPIO_Pin_11, RCC_AHB1Periph_GPIOB, GPIO_PinSource11, GPIO_AF_USART3},
        {(uint32_t)GPIOA,  GPIO_Pin_0, RCC_AHB1Periph_GPIOA,  GPIO_PinSource0, (uint32_t)GPIOA,  GPIO_Pin_1, RCC_AHB1Periph_GPIOA,  GPIO_PinSource1,  GPIO_AF_UART4},
        {(uint32_t)GPIOC, GPIO_Pin_12, RCC_AHB1Periph_GPIOC, GPIO_PinSource12, (uint32_t)GPIOD,  GPIO_Pin_2, RCC_AHB1Periph_GPIOD,  GPIO_PinSource2,  GPIO_AF_UART5},
        {(uint32_t)GPIOC,  GPIO_Pin_6, RCC_AHB1Periph_GPIOC,  GPIO_PinSource6, (uint32_t)GPIOC,  GPIO_Pin_7, RCC_AHB1Periph_GPIOC,  GPIO_PinSource7, GPIO_AF_USART6},
        {(uint32_t)GPIOE,  GPIO_Pin_8, RCC_AHB1Periph_GPIOE,  GPIO_PinSource8, (uint32_t)GPIOE,  GPIO_Pin_7, RCC_AHB1Periph_GPIOE,  GPIO_PinSource7,  GPIO_AF_UART7},
        {(uint32_t)GPIOE,  GPIO_Pin_1, RCC_AHB1Periph_GPIOE,  GPIO_PinSource1, (uint32_t)GPIOE,  GPIO_Pin_0, RCC_AHB1Periph_GPIOE,  GPIO_PinSource0,  GPIO_AF_UART8}
    },

    {   {(uint32_t)GPIOB,  GPIO_Pin_6, RCC_AHB1Periph_GPIOB,  GPIO_PinSource6, (uint32_t)GPIOB,  GPIO_Pin_7, RCC_AHB1Periph_GPIOB,  GPIO_PinSource7, GPIO_AF_USART1},
        {(uint32_t)GPIOD,  GPIO_Pin_5, RCC_AHB1Periph_GPIOD,  GPIO_PinSource5, (uint32_t)GPIOD,  GPIO_Pin_6, RCC_AHB1Periph_GPIOD,  GPIO_PinSource6, GPIO_AF_USART2},
        {(uint32_t)GPIOC, GPIO_Pin_10, RCC_AHB1Periph_GPIOC, GPIO_PinSource10, (uint32_t)GPIOC, GPIO_Pin_11, RCC_AHB1Periph_GPIOC, GPIO_PinSource11, GPIO_AF_USART3},
        {(uint32_t)GPIOC, GPIO_Pin_10, RCC_AHB1Periph_GPIOC, GPIO_PinSource10, (uint32_t)GPIOC, GPIO_Pin_11, RCC_AHB1Periph_GPIOC, GPIO_PinSource11,  GPIO_AF_UART4},
        {               ,            ,                     ,                 ,                ,            ,                     ,                 ,               },
        {(uint32_t)GPIOG, GPIO_Pin_14, RCC_AHB1Periph_GPIOG, GPIO_PinSource14, (uint32_t)GPIOG,  GPIO_Pin_9, RCC_AHB1Periph_GPIOG,  GPIO_PinSource9, GPIO_AF_USART6},
        {(uint32_t)GPIOF,  GPIO_Pin_7, RCC_AHB1Periph_GPIOF,  GPIO_PinSource7, (uint32_t)GPIOF,  GPIO_Pin_6, RCC_AHB1Periph_GPIOF,  GPIO_PinSource6,  GPIO_AF_UART7},
        {               ,            ,                     ,                 ,                ,            ,                     ,                 ,               }
    },

    {   {               ,            ,                     ,                 ,                ,            ,                     ,                 ,               },
        {               ,            ,                     ,                 ,                ,            ,                     ,                 ,               },
        {(uint32_t)GPIOD,  GPIO_Pin_8, RCC_AHB1Periph_GPIOD,  GPIO_PinSource8, (uint32_t)GPIOD,  GPIO_Pin_9, RCC_AHB1Periph_GPIOD,  GPIO_PinSource9, GPIO_AF_USART3},
        {               ,            ,                     ,                 ,                ,            ,                     ,                 ,               },
        {               ,            ,                     ,                 ,                ,            ,                     ,                 ,               },
        {               ,            ,                     ,                 ,                ,            ,                     ,                 ,               },
        {               ,            ,                     ,                 ,                ,            ,                     ,                 ,               },
        {               ,            ,                     ,                 ,                ,            ,                     ,                 ,               }
    }
}

/* Internal function ---------------------------------------------------------*/


/* External function ---------------------------------------------------------*/
