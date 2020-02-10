#ifndef _I2C_H_
#define _I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

/* Table below shows all possible pins for each i2c
 *
 *  I2Cx | Pins pack 1 | Pins pack 2 | Pins pack 3 |  APB  |
 *       |  SCL   SDA  |  SCL   SDA  |  SCL    SDA |       |
 *----------------------------------------------------------
 *  I2C1 |  PB6   PB7  |  PB8   PB9  |  PB6    PB9 |   1   |
 *  I2C2 |  PB10  PB11 |  PF1   PF0  |  PH4    PH5 |   1   |
 *  I2C3 |  PA8   PC9  |  PH7   PH8  |  -      -   |   1   |
 */

typedef struct i2c *i2c_handle_t;

typedef enum {
    I2C_NUM_1 = 0,
    I2C_NUM_2,
    I2C_NUM_3,
    I2C_NUM_MAX
} i2c_num_t;

typedef enum {
    I2C_PINS_PACK_1 = 0,
    I2C_PINS_PACK_2,
    I2C_PINS_PACK_3,
    I2C_PINS_PACK_MAX
} i2c_pins_pack_t;

typedef struct {
    i2c_num_t       i2c_num;
    i2c_pins_pack_t i2c_pins_pack;
} i2c_config_t;

i2c_handle_t i2c_init(i2c_config_t *config);
I2C_HandleTypeDef i2c_get_I2C_HandleTypeDef(i2c_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* _I2C_H_ */
