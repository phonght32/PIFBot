#ifndef _I2C_H_
#define _I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

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
