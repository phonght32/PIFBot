#ifndef DMA_H_
#define DMA_H_

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif

#endif /* DMA_H_ */
