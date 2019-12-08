/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"

#include "include/dma.h"


/* Internal define -----------------------------------------------------------*/
#define ADC1_ADDR	0
#define ADC2_ADDR	0
#define ADC3_ADDR	0

#define DAC1_ADDR	0
#define DAC2_ADDR	0

#define SPI1_ADDR	0
#define SPI2_ADDR	0
#define SPI3_ADDR	0

#define I2C1_ADDR	0
#define I2C2_ADDR	0
#define I2C3_ADDR	0

#define UART4_ADDR	&UART4->DR
#define UART5_ADDR	&UART5->DR

#define USART1_ADDR	0
#define USART2_ADDR	0
#define USART3_ADDR	0
#define USART6_ADDR	0

#define I2S1_ADDR	0
#define I2S2_ADDR	0
#define I2S3_ADDR   0

#define TIM1_ADDR	0
#define TIM2_ADDR	0
#define TIM3_ADDR	0
#define TIM4_ADDR	0
#define TIM5_ADDR	0
#define TIM6_ADDR	0
#define TIM7_ADDR	0
#define TIM8_ADDR	0

#define SDIO_ADDR	0

#define DCMI_ADDR	0

#define CRYP_ADDR	0

#define HASH_ADDR	0

#define DMA_PERIPH_INC_DEFAULT			DMA_PeripheralInc_Disable
#define DMA_MEM_INC_DEFAULT 			DMA_MemoryInc_Enable
#define DMA_PERIPH_DATA_SIZE_DEFAULT 	DMA_PeripheralDataSize_Byte
#define DMA_MEM_DATA_SIZE_DEFAULT 		DMA_MemoryDataSize_Byte
#define DMA_FIFO_MODE_DEFAULT  			DMA_FIFOMode_Disable
#define DMA_MEM_BURST_DEFAULT  			DMA_MemoryBurst_Single
#define DMA_PERIPH_BURST_DEFAULT		DMA_PeripheralBurst_Single
#define DMA_FIFO_THRESHOLD_DEFAULT  	DMA_FIFOThreshold_HalfFull

#define DMA_DIR_IN		DMA_DIR_PeripheralToMemory
#define DMA_DIR_OUT		DMA_DIR_MemoryToPeripheral

/* Internal typedef ----------------------------------------------------------*/
typedef struct dma {
	dma_stream_t    dma_stream;
	dma_channel_t   dma_channel;
	dma_num_t       dma_num;
	uint32_t        dma_mode;
	uint8_t         *buffer;
	uint8_t         buffer_size;
	uint32_t        dma_priority;
} dma_t;

typedef enum {
	DMA_PARAM_MAPPING_PeripheralBaseAddr = 0,
	DMA_PARAM_MAPPING_DIR,
	DMA_PARAM_MAPPING_INDEX_MAX
} dma_mapping_index_t;

/* Internal variable ---------------------------------------------------------*/
uint32_t DMA1_PARAM_MAPPING[DMA_STREAM_MAX][DMA_CHANNEL_MAX][DMA_PARAM_MAPPING_INDEX_MAX] = {
	{	{   SPI3_ADDR,  DMA_DIR_IN},
		{   I2C1_ADDR,  DMA_DIR_IN},
		{   TIM4_ADDR,           0},
		{   I2S3_ADDR,           0},
		{  UART5_ADDR,  DMA_DIR_IN},
		{           0,           0},
		{   TIM5_ADDR,},
		{           0,           0}
	},

	{	{           0,           0},
		{           0,           0},
		{           0,           0},
		{   TIM2_ADDR,           0},
		{ USART3_ADDR,  DMA_DIR_IN},
		{           0,           0},
		{   TIM5_ADDR,           0},
		{   TIM6_ADDR,           0}
	},

	{	{   SPI3_ADDR,  DMA_DIR_IN},
		{   TIM7_ADDR,           0},
		{   I2S2_ADDR,  DMA_DIR_IN},
		{   I2C3_ADDR,  DMA_DIR_IN},
		{  UART4_ADDR,  DMA_DIR_IN},
		{   TIM3_ADDR,           0},
		{   TIM5_ADDR,           0},
		{   I2C2_ADDR,  DMA_DIR_IN}
	},

	{	{   SPI2_ADDR,  DMA_DIR_IN},
		{           0,           0},
		{   TIM4_ADDR,           0},
		{   I2S2_ADDR,  DMA_DIR_IN},
		{ USART3_ADDR,  DMA_DIR_IN},
		{           0,           0},
		{   TIM5_ADDR,           0},
		{   I2C2_ADDR,  DMA_DIR_IN}
	},

	{	{   SPI2_ADDR,           0},
		{   TIM7_ADDR,           0},
		{   I2S2_ADDR,           0},
		{   I2C3_ADDR,           0},
		{  UART4_ADDR,           0},
		{   TIM3_ADDR,           0},
		{   TIM5_ADDR,           0},
		{ USART3_ADDR,           0}
	},

	{	{   SPI3_ADDR,           0},
		{   I2C1_ADDR,           0},
		{   I2S3_ADDR,           0},
		{   TIM2_ADDR,           0},
		{ USART2_ADDR,  DMA_DIR_IN},
		{   TIM3_ADDR,           0},
		{           0,           0},
		{   DAC1_ADDR,           0}
	},

	{	{           0,           0},
		{   I2C1_ADDR,           0},
		{   TIM4_ADDR,           0},
		{   TIM2_ADDR,           0},
		{ USART2_ADDR,  DMA_DIR_IN},
		{           0,           0},
		{   TIM5_ADDR,           0},
		{   DAC2_ADDR,           0}
	},

	{	{   SPI3_ADDR,           0},
		{   I2C1_ADDR,           0},
		{   TIM4_ADDR,           0},
		{   TIM2_ADDR,           0},
		{  UART5_ADDR,           0},
		{   TIM3_ADDR,           0},
		{           0,           0},
		{   I2C2_ADDR,           0}
	},


};

uint32_t DMA2_PARAM_MAPPING[DMA_STREAM_MAX][DMA_CHANNEL_MAX][DMA_PARAM_MAPPING_INDEX_MAX] = {
	{	{   ADC1_ADDR,  DMA_DIR_IN},
		{           0,           0},
		{   ADC3_ADDR,  DMA_DIR_IN},
		{   SPI1_ADDR,  DMA_DIR_IN},
		{           0,           0},
		{           0,           0},
		{   TIM1_ADDR,  DMA_DIR_IN},
		{           0,           0}
	},

	{	{           0,           0},
		{   DCMI_ADDR,           0},
		{   ADC3_ADDR,  DMA_DIR_IN},
		{           0,           0},
		{           0,           0},
		{ USART6_ADDR,  DMA_DIR_IN},
		{   TIM1_ADDR,  DMA_DIR_IN},
		{   TIM8_ADDR,  DMA_DIR_IN}
	},

	{	{   TIM8_ADDR,  DMA_DIR_IN},
		{   ADC2_ADDR,  DMA_DIR_IN},
		{           0,           0},
		{   SPI1_ADDR,  DMA_DIR_IN},
		{ USART1_ADDR,  DMA_DIR_IN},
		{ USART6_ADDR,  DMA_DIR_IN},
		{   TIM2_ADDR,  DMA_DIR_IN},
		{   TIM8_ADDR,  DMA_DIR_IN}
	},

	{	{           0,           0},
		{   ADC2_ADDR,  DMA_DIR_IN},
		{           0,           0},
		{   SPI1_ADDR, DMA_DIR_OUT},
		{   SDIO_ADDR,           0},
		{           0,           0},
		{   TIM1_ADDR,  DMA_DIR_IN},
		{   TIM8_ADDR,  DMA_DIR_IN}
	},

	{	{   ADC1_ADDR,  DMA_DIR_IN},
		{           0,           0},
		{           0,           0},
		{           0,           0},
		{           0,           0},
		{           0,           0},
		{   TIM1_ADDR,  DMA_DIR_IN},
		{   TIM8_ADDR,  DMA_DIR_IN}
	},

	{	{           0,           0},
		{           0,           0},
		{   CRYP_ADDR,           0},
		{   SPI1_ADDR, DMA_DIR_OUT},
		{ USART1_ADDR,  DMA_DIR_IN},
		{           0,           0},
		{   TIM1_ADDR,  DMA_DIR_IN},
		{           0,           0}
	},

	{	{   TIM1_ADDR,  DMA_DIR_IN},
		{           0,           0},
		{   CRYP_ADDR,           0},
		{           0,           0},
		{   SDIO_ADDR,           0},
		{ USART6_ADDR, DMA_DIR_OUT},
		{   TIM3_ADDR,  DMA_DIR_IN},
		{           0,           0}
	},

	{	{           0,           0},
		{   DCMI_ADDR,           0},
		{   HASH_ADDR,           0},
		{           0,           0},
		{ USART1_ADDR, DMA_DIR_OUT},
		{ USART6_ADDR, DMA_DIR_OUT},
		{           0,           0},
		{   TIM8_ADDR,           0}
	},
};
uint32_t DMA_CHANNEL_MAPPING[DMA_CHANNEL_MAX] = {
	DMA_Channel_0,
	DMA_Channel_1,
	DMA_Channel_2,
	DMA_Channel_3,
	DMA_Channel_4,
	DMA_Channel_5,
	DMA_Channel_6,
	DMA_Channel_7
};

DMA_Stream_TypeDef *DMA_STREAM_MAPPING[DMA_STREAM_MAX][DMA_NUM_MAX] = {
	{DMA1_Stream0, DMA2_Stream0},
	{DMA1_Stream1, DMA2_Stream1},
	{DMA1_Stream2, DMA2_Stream2},
	{DMA1_Stream3, DMA2_Stream3},
	{DMA1_Stream4, DMA2_Stream4},
	{DMA1_Stream5, DMA2_Stream5},
	{DMA1_Stream6, DMA2_Stream6},
	{DMA1_Stream7, DMA2_Stream7}
};

uint8_t DMA_Stream_IRQ_MAPPING[DMA_STREAM_MAX][DMA_NUM_MAX] = {
	{DMA1_Stream0_IRQn, DMA2_Stream0_IRQn},
	{DMA1_Stream1_IRQn, DMA2_Stream1_IRQn},
	{DMA1_Stream2_IRQn, DMA2_Stream2_IRQn},
	{DMA1_Stream3_IRQn, DMA2_Stream3_IRQn},
	{DMA1_Stream4_IRQn, DMA2_Stream4_IRQn},
	{DMA1_Stream5_IRQn, DMA2_Stream5_IRQn},
	{DMA1_Stream6_IRQn, DMA2_Stream6_IRQn},
	{DMA1_Stream7_IRQn, DMA2_Stream7_IRQn},
};
/* Internal function ---------------------------------------------------------*/


/* External function ---------------------------------------------------------*/
dma_handle_t dma_init(dma_config_t *config)
{
	uint32_t PeriphBaseAddr;
	uint32_t DMA_Dir;
	DMA_Stream_TypeDef *DMA_Stream;

	if (config->dma_num == DMA_NUM_1)
	{
		PeriphBaseAddr = DMA1_PARAM_MAPPING[config->dma_stream][config->dma_channel][DMA_PARAM_MAPPING_PeripheralBaseAddr];
		DMA_Dir        = DMA1_PARAM_MAPPING[config->dma_stream][config->dma_channel][DMA_PARAM_MAPPING_DIR];

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	}
	else
	{
		PeriphBaseAddr = DMA2_PARAM_MAPPING[config->dma_stream][config->dma_channel][DMA_PARAM_MAPPING_PeripheralBaseAddr];
		DMA_Dir        = DMA2_PARAM_MAPPING[config->dma_stream][config->dma_channel][DMA_PARAM_MAPPING_DIR];

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	}
	DMA_Stream = DMA_STREAM_MAPPING[config->dma_stream][config->dma_num];

	DMA_InitTypeDef   DMA_InitStructure;
	DMA_InitStructure.DMA_Channel            = DMA_CHANNEL_MAPPING[config->dma_channel];
	DMA_InitStructure.DMA_PeripheralBaseAddr = PeriphBaseAddr;
	DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)config->buffer;
	DMA_InitStructure.DMA_DIR                = DMA_Dir;
	DMA_InitStructure.DMA_BufferSize         = config->buffer_size;
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PERIPH_INC_DEFAULT;
	DMA_InitStructure.DMA_MemoryInc          = DMA_MEM_INC_DEFAULT;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PERIPH_DATA_SIZE_DEFAULT;
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MEM_DATA_SIZE_DEFAULT;
	DMA_InitStructure.DMA_Mode               = config->dma_mode;
	DMA_InitStructure.DMA_Priority           = config->dma_priority;
	DMA_InitStructure.DMA_FIFOMode           = DMA_FIFO_MODE_DEFAULT;
	DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFO_THRESHOLD_DEFAULT;
	DMA_InitStructure.DMA_MemoryBurst        = DMA_MEM_BURST_DEFAULT;
	DMA_InitStructure.DMA_PeripheralBurst    = DMA_PERIPH_BURST_DEFAULT;
	DMA_Init(DMA_Stream, &DMA_InitStructure);
	DMA_Cmd(DMA_Stream, ENABLE);

	dma_handle_t handle  = calloc(1, sizeof(dma_t));
	if(!handle) 
	{
		return -1;
	}
	handle->dma_stream   = config->dma_stream;
	handle->dma_channel  = config->dma_channel;
	handle->dma_num      = config->dma_num;
	handle->dma_mode     = config->dma_mode;
	handle->dma_priority = config->dma_priority;
	handle->buffer_size  = config->buffer_size;

	return handle;
}

int dma_intr_enable(dma_handle_t handle, uint32_t intr_type)
{
	NVIC_InitTypeDef  NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA_Stream_IRQ_MAPPING[handle->dma_stream][handle->dma_num];
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA_STREAM_MAPPING[handle->dma_stream][handle->dma_num], intr_type, ENABLE);
	return 0;
}




