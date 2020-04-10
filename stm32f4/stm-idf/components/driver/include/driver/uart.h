// MIT License

// Copyright (c) 2020 thanhphong98 & thuanpham98

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef _UART_H_
#define _UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdlib.h"
#include "stdint.h"

#include "stm32f4xx_hal.h"

#include "stm_err.h"
#include "stm_log.h"


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

typedef enum {
    UART_NUM_1 = 0,     /*!< UART Num 1 selected */
    UART_NUM_2,         /*!< UART Num 2 selected */
    UART_NUM_3,         /*!< UART Num 3 selected */
    UART_NUM_4,         /*!< UART Num 4 selected */
    UART_NUM_5,         /*!< UART Num 5 selected */
    UART_NUM_6,         /*!< UART Num 6 selected */
    UART_NUM_MAX
} uart_num_t;

typedef enum {
    UART_PINS_PACK_1 = 0,   /*!< UART Pins Pack 1 selected */
    UART_PINS_PACK_2,       /*!< UART Pins Pack 2 selected */
    UART_PINS_PACK_3,       /*!< UART Pins Pack 3 selected */
    UART_PINS_PACK_MAX
} uart_pins_pack_t;

typedef enum {
    UART_FRAME_8N1 = 0,     /*!< 8 bit data, none parity, 1 stop bit */
    UART_FRAME_8N2,         /*!< 8 bit data, none parity, 2 stop bit */
    UART_FRAME_9N1,         /*!< 9 bit data, none parity, 1 stop bit */
    UART_FRAME_9N2,         /*!< 9 bit data, none parity, 2 stop bit */
    UART_FRAME_8E1,         /*!< 8 bit data, even parity, 1 stop bit */
    UART_FRAME_8E2,         /*!< 8 bit data, even parity, 2 stop bit */
    UART_FRAME_9E1,         /*!< 9 bit data, even parity, 1 stop bit */
    UART_FRAME_9E2,         /*!< 9 bit data, even parity, 2 stop bit */
    UART_FRAME_8O1,         /*!< 8 bit data, odd parity, 1 stop bit */
    UART_FRAME_8O2,         /*!< 8 bit data, odd parity, 2 stop bit */
    UART_FRAME_9O1,         /*!< 9 bit data, odd parity, 1 stop bit */
    UART_FRAME_9O2,         /*!< 9 bit data, odd parity, 2 stop bit */
    UART_FRAME_MAX_TYPE
} uart_frame_format_t;

typedef enum {
    UART_TRANSFER_MODE_RX = 0,  /*!< UART mode receive*/
    UART_TRANSFER_MODE_TX,      /*!< UART mode transfer*/
    UART_TRANSFER_MODE_TX_RX,   /*!< UART mode transfer and receive */
    UART_TRANSFER_MODE_MAX
} uart_transfer_mode_t;

typedef enum {
    UART_HW_FLW_CTRL_NONE = 0,  /* No hardware flow control */
    UART_HW_FLW_CTRL_RTS,       /* Use RTS */
    UART_HW_FLW_CTRL_CTS,       /* Use CTS */
    UART_HW_FLW_CTRL_RTS_CTS,   /* Use RTS and CTS */
    UART_HW_FLW_CTRL_MAX_TYPE
} uart_hw_flw_ctrl_t;

typedef struct {
    uart_num_t              uart_num;               /*!< UART Num */
    uart_pins_pack_t        uart_pins_pack;         /*!< UART Pins Pack */
    uint32_t                baudrate;               /*!< UART Baudrate */
    uart_frame_format_t     frame_format;           /*!< UART Frame Format */
    uart_transfer_mode_t    mode;                   /*!< UART Mode */ 
    uart_hw_flw_ctrl_t      hw_flw_ctrl;
} uart_config_t;

/*
 * @brief   Initialize UART.
 * @param   config Struct pointer.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t uart_config(uart_config_t *config);

/*
 * @brief   UART write bytes.
 * @param   uart_num UART num.
 * @param   data Data transmit pointer.
 * @param   length Data length.
 * @param   timeout_ms Time out in microsecond.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t uart_write_bytes(uart_num_t uart_num, uint8_t *data, uint16_t length, uint32_t timeout_ms);

/*
 * @brief   UART read bytes.
 * @param   uart_num UART num.
 * @param   buf Data receive pointer.
 * @param   length Data length.
 * @param   timeout_ms Time out in microsecond.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t uart_read_bytes(uart_num_t uart_num, uint8_t *buf, uint16_t length, uint32_t timeout_ms);


#ifdef __cplusplus
}
#endif

#endif /* _UART_H_ */
