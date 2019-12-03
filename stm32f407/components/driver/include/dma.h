#ifndef DMA_H_
#define DMA_H_

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif

#endif /* DMA_H_ */