#ifndef DMA_H_
#define DMA_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Table below shows all possible DMA.
 *
 *  Periperal | Stream 0  | Stream 1  | Stream 2  | Stream 3  | Stream 4  | Stream 5  | Stream 6  | Stream 7  |     
 *  requests  |           |           |           |           |           |           |           |           |
 *  ----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|
 *  Channel 0 | SPI3_RX   |           | SPI3_RX   | SPI2_RX   | SPI2_TX   | SPI3_TX   |           | SPI3_TX   |
 *  ----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|
 *  Channel 1 | I2C1_RX   |           | TIM7_UP   |           | TIM7_UP   | I2C1_RX   | I2C1_TX   | I2C1_TX   |
 *  ----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|
 *  Channel 2 | TIM4_CH1  |           | I2S2_EXT_ | TIM4_CH2  | I2S2_EXT  | I2S3_EXT_ | TIM4_UP   | TIM4_CH3  |
 *            |           |           | RX        |           | TX        | TX        |           |           |
 *  ----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|
 *  Channel 3 | I2S3_EXT_ | TIM2_UP   | I2C3_RX   | I2S2_EXT_ | I2C3_TX   | TIM2_CH1  | TIM2_CH2  | TIM2_UP   |
 *            | RX        | TIM2_CH3  |           | RX        |           |           | TIM2_CH4  | TIM2_CH4  |
 *  ----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|
 *  Channel 4 | UART5_RX  | USART3_RX | UART4_RX  | USART3_TX | UART4_TX  | USART2_RX | USART2_TX | UART5_TX  |
 *  ----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|
 *  Channel 5 |           |           | TIM3_CH4  |           | TIM3_CH1  | TIM3_CH2  |           | TIM3_CH3  |
 *            |           |           | TIM3_UP   |           | TIM3_TRIG |           |           |           |
 *  ----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|
 *  Channel 6 | TIM5_CH3  | TIM5_CH4  | TIM5_CH1  | TIM5_CH4  | TIM5_CH2  |           | TIM5_UP   |           |
 *            | TIM5_UP   | TIM5_TRIG |           | TIM5_TRIG |           |           |           |           |
 *  ----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|             
 *  Channel 7 |           | TIM6_UP   | I2C2_RX   | I2C2_RX   | USART3_TX | DAC1      | DAC2      | I2C2_TX   |
 *  ----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|
 *
 */  

typedef struct dma *dma_handle_t;

typedef enum {
    DMA_STREAM_0 = 0,
    DMA_STREAM_1,
    DMA_STREAM_2,
    DMA_STREAM_3,
    DMA_STREAM_4,
    DMA_STREAM_5,
    DMA_STREAM_6,
    DMA_STREAM_7,
    DMA_STREAM_MAX
} dma_stream_t;

typedef enum {
    DMA_CHANNEL_0 = 0,
    DMA_CHANNEL_1,
    DMA_CHANNEL_2,
    DMA_CHANNEL_3,
    DMA_CHANNEL_4,
    DMA_CHANNEL_5,
    DMA_CHANNEL_6,
    DMA_CHANNEL_7,
    DMA_CHANNEL_MAX
} dma_channel_t;

typedef enum {
    DMA_NUM_1 = 0,
    DMA_NUM_2,
    DMA_NUM_MAX
} dma_num_t;

typedef struct {
    dma_stream_t    dma_stream;
    dma_channel_t   dma_channel;
    dma_num_t       dma_num;
    uint32_t        dma_mode;
    uint32_t        *buffer;
    uint8_t         buffer_size;
    uint32_t        dma_priority;
} dma_config_t;


dma_handle_t dma_init(dma_config_t *config);
int dma_intr_enable(dma_handle_t handle, uint32_t intr_type);
int dma_deinit(dma_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* DMA_H_ */
