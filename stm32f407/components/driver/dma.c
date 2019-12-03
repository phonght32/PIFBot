/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"

#include "include/dma.h"


/* Internal define -----------------------------------------------------------*/
#define		BUFF_SIZE			4

uint8_t 	rxbuff[BUFF_SIZE];

/* Internal typedef ----------------------------------------------------------*/


/* Internal variable ---------------------------------------------------------*/
uint32_t DMA_CHANNEL_MAPPING[DMA_CHANNEL_MAX] = {
	DMA_Channel_0 = 0,
	DMA_Channel_1,
	DMA_Channel_2,
	DMA_Channel_3,
	DMA_Channel_4,
	DMA_Channel_5,
	DMA_Channel_6,
	DMA_Channel_7
}

/* Internal function ---------------------------------------------------------*/


/* External function ---------------------------------------------------------*/

void init(void)
{
	NVIC_InitTypeDef  NVIC_InitStructure;
	USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
	DMA_InitTypeDef   DMA_InitStructure;
		/* DMA1 Stream2 Channel4 for USART4 Rx configuration */
	  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
	  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxbuff;
	  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	  DMA_InitStructure.DMA_BufferSize = BUFF_SIZE;
	  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
	  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	  DMA_Init(DMA1_Stream2, &DMA_InitStructure);
	  DMA_Cmd(DMA1_Stream2, ENABLE);

		/* Enable DMA Interrupt to the highest priority */
	  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  /* Transfer complete interrupt mask */
	  DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
}