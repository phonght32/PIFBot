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

#ifndef _I2C_H_
#define _I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdlib.h"
#include "stdint.h"

#include "stm32f4xx_hal.h"

#include "stm_err.h"
#include "stm_log.h"

/* Table below shows all possible pins for each i2c.
 *
 *  I2Cx | Pins pack 1 | Pins pack 2 | Pins pack 3 |  APB  |
 *       |  SCL   SDA  |  SCL   SDA  |  SCL    SDA |       |
 *----------------------------------------------------------
 *  I2C1 |  PB6   PB7  |  PB8   PB9  |  PB6    PB9 |   1   |
 *  I2C2 |  PB10  PB11 |  PF1   PF0  |  PH4    PH5 |   1   |
 *  I2C3 |  PA8   PC9  |  PH7   PH8  |  -      -   |   1   |
 */

/*
 * I2C Num Select Typedef.
 */
typedef enum {
    I2C_NUM_1 = 0,      /*!< I2C Num 1 selected */
    I2C_NUM_2,          /*!< I2C Num 2 selected */
    I2C_NUM_3,          /*!< I2C Num 3 selected */
    I2C_NUM_MAX
} i2c_num_t;

/*
 * I2C Pins Pack Select Typedef.
 */
typedef enum {
    I2C_PINS_PACK_1 = 0,    /*!< I2C Pins Pack 1 selected */
    I2C_PINS_PACK_2,        /*!< I2C Pins Pack 2 selected */
    I2C_PINS_PACK_3,        /*!< I2C Pins Pack 3 selected */
    I2C_PINS_PACK_MAX
} i2c_pins_pack_t;

/*
 * I2C Configuration Typedef.
 */
typedef struct {
    i2c_num_t       i2c_num;        /*!< I2C Num */
    i2c_pins_pack_t i2c_pins_pack;  /*!< I2C Pins Pack */
    uint32_t        clk_speed;      /*!< I2C Clock Speed */
} i2c_config_t;

/*
 * @brief   Initialize I2C.
 * @param   config Struct pointer.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t i2c_config(i2c_config_t *config);

/*
 * @brief   I2C master transmit data.
 * @param   i2c_num I2C num.
 * @param   dev_addr Device address.
 * @param   data Data send.
 * @param   length Data length.
 * @param   timeout_ms Time out in microsecond.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t i2c_write_bytes(i2c_num_t i2c_num, uint16_t dev_addr, uint8_t *data, uint16_t length, uint32_t timeout_ms);

/*
 * @brief   I2C master receive data.
 * @param   i2c_num I2C num.
 * @param   dev_addr Device address.
 * @param   buf Buffer receive.
 * @param   length Data length.
 * @param   timeout_ms Time out in microsecond.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t i2c_read_bytes(i2c_num_t i2c_num, uint16_t dev_addr, uint8_t *buf, uint16_t length, uint32_t timeout_ms);


#ifdef __cplusplus
}
#endif

#endif /* _I2C_H_ */
