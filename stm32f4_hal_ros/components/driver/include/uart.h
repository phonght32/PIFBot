#ifndef _UART_H_
#define _UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_dma.h"


/* Table below shows all possible pins for each uart
 *
 *  U(S)ARTx | Pins pack 1 | Pins pack 2 | Pins pack 3 |  APB  |
 *           |  TX    RX   |  TX    RX   |  TX    RX   |       |
 *--------------------------------------------------------------
 *  USART1   |  PA9   PA10 |  PB6   PB7  |  -     -    |   2   |
 *  USART2   |  PA2   PA3  |  PD5   PD6  |  -     -    |   1   |
 *  USART3   |  PB10  PB11 |  PC10  PC11 |  PD8   PD9  |   1   |
 *  USART4   |  PA0   PA1  |  PC10  PC11 |  -     -    |   1   |
 *  USART5   |  PC12  PD2  |  -     -    |  -     -    |   1   |
 *  USART6   |  PC6   PC7  |  PG14  PG9  |  -     -    |   2   |
 *  USART7   |  PE8   PE7  |  PF7   PF6  |  -     -    |   1   |
 *  USART8   |  PE1   PE0  |  -     -    |  -     -    |   1   |
 */

typedef struct uart *uart_handle_t;

typedef enum {
    USART_NUM_1 = 0,
    USART_NUM_2,
    USART_NUM_3,
    UART_NUM_4,
    UART_NUM_5,
    USART_NUM_6,
    USART_NUM_MAX
} usart_num_t;

typedef enum {
    USART_PINS_PACK_1 = 0,
    USART_PINS_PACK_2,
    USART_PINS_PACK_3,
    USART_PINS_PACK_MAX
} usart_pins_pack_t;

typedef struct {
    usart_num_t         uart_num;
    usart_pins_pack_t   uart_pins_pack;
    uint32_t            baudrate;
} uart_config_t;

uart_handle_t uart_init(uart_config_t *config);
int uart_write_bytes(uart_handle_t handle, uint8_t *data, uint16_t length, uint32_t timeout_ms);
int uart_read_bytes(uart_handle_t handle, uint8_t *buf, uint16_t length, uint32_t timeout_ms);
void uart_dma_tx_init(uart_handle_t handle,DMA_HandleTypeDef *hdma_uart_tx);
DMA_HandleTypeDef uart_dma_rx_init(uart_handle_t handle);
int uart_dma_write(uart_handle_t handle, uint8_t *data, uint32_t length);
UART_HandleTypeDef uart_get_hal_handle(uart_handle_t handle);
#ifdef __cplusplus
}
#endif

#endif /* _UART_H_ */
