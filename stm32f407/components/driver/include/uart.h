#ifndef _UART_H 
#define _UART_H 

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/* NOTE: Change HSE_VALUE in stm32f4xx.h to 8000000 before start your application */    

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
    USART_NUM_1 = 0,
    USART_NUM_2,
    USART_NUM_3,
    USART_NUM_4,
    USART_NUM_5,
    USART_NUM_6,
    USART_NUM_7,
    USART_NUM_8,
    USART_NUM_MAX
}usart_num_t;

typedef enum {
    USART_PINS_PACK_1 = 0,
    USART_PINS_PACK_2,
    USART_PINS_PACK_3,
    USART_PINS_PACK_MAX
} usart_pins_pack_t;

#ifdef __cplusplus
}
#endif

#endif /* _UART_H */