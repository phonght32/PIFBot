#include "include/i2c.h"

#define I2C_DUTYCYCLE_DEFAULT 			I2C_DUTYCYCLE_2
#define I2C_CLOCKSPEED_DEFAULT			100000
#define I2C_OWN_ADDRESS1_DEFAULT		0
#define I2C_ADDRESSING_MODE_DEFAULT		I2C_ADDRESSINGMODE_7BIT
#define I2C_DUAL_ADDRESS_MODE_DEFAULT 	I2C_DUALADDRESS_DISABLE
#define I2C_ONW_ADDRESS2_DEFAULT		0
#define I2C_GENERALCALL_MODE_DEFAULT	I2C_GENERALCALL_DISABLE
#define I2C_NOSTRETCH_MODE_DEFAULT		I2C_NOSTRETCH_DISABLE

#define I2C1_PP1_HW_INFO    {.i2c = I2C1,                                   \
                             .rcc_apbenr_i2cen = RCC_APB1ENR_I2C1EN,        \
                             .rcc_ahbenr_gpio_scl = RCC_AHB1ENR_GPIOBEN,    \
                             .rcc_ahbenr_gpio_sda = RCC_AHB1ENR_GPIOBEN,    \
                             .port_scl = GPIOB,                             \
                             .port_sda = GPIOB,                             \
                             .pin_scl = GPIO_PIN_6,                         \
                             .pin_sda = GPIO_PIN_7,                         \
                             .alternate_func = GPIO_AF4_I2C1}

#define I2C1_PP2_HW_INFO    {.i2c = I2C1,                                   \
                             .rcc_apbenr_i2cen = RCC_APB1ENR_I2C1EN,        \
                             .rcc_ahbenr_gpio_scl = RCC_AHB1ENR_GPIOBEN,    \
                             .rcc_ahbenr_gpio_sda = RCC_AHB1ENR_GPIOBEN,    \
                             .port_scl = GPIOB,                             \
                             .port_sda = GPIOB,                             \
                             .pin_scl = GPIO_PIN_8,                         \
                             .pin_sda = GPIO_PIN_9,                         \
                             .alternate_func = GPIO_AF4_I2C1}

#define I2C1_PP3_HW_INFO    {.i2c = I2C1,                                   \
                             .rcc_apbenr_i2cen = RCC_APB1ENR_I2C1EN,        \
                             .rcc_ahbenr_gpio_scl = RCC_AHB1ENR_GPIOBEN,    \
                             .rcc_ahbenr_gpio_sda = RCC_AHB1ENR_GPIOBEN,    \
                             .port_scl = GPIOB,                             \
                             .port_sda = GPIOB,                             \
                             .pin_scl = GPIO_PIN_6,                         \
                             .pin_sda = GPIO_PIN_9,                         \
                             .alternate_func = GPIO_AF4_I2C1}

#define I2C2_PP1_HW_INFO    {.i2c = I2C2,                                   \
                             .rcc_apbenr_i2cen = RCC_APB1ENR_I2C2EN,        \
                             .rcc_ahbenr_gpio_scl = RCC_AHB1ENR_GPIOBEN,    \
                             .rcc_ahbenr_gpio_sda = RCC_AHB1ENR_GPIOBEN,    \
                             .port_scl = GPIOB,                             \
                             .port_sda = GPIOB,                             \
                             .pin_scl = GPIO_PIN_10,                        \
                             .pin_sda = GPIO_PIN_11,                        \
                             .alternate_func = GPIO_AF4_I2C2}

#define I2C2_PP2_HW_INFO    {.i2c = I2C2,                                   \
                             .rcc_apbenr_i2cen = RCC_APB1ENR_I2C2EN,        \
                             .rcc_ahbenr_gpio_scl = RCC_AHB1ENR_GPIOFEN,    \
                             .rcc_ahbenr_gpio_sda = RCC_AHB1ENR_GPIOFEN,    \
                             .port_scl = GPIOF,                             \
                             .port_sda = GPIOF,                             \
                             .pin_scl = GPIO_PIN_1,                         \
                             .pin_sda = GPIO_PIN_0,                         \
                             .alternate_func = GPIO_AF4_I2C2}

#define I2C2_PP3_HW_INFO    {.i2c = I2C2,                                   \
                             .rcc_apbenr_i2cen = RCC_APB1ENR_I2C2EN,        \
                             .rcc_ahbenr_gpio_scl = RCC_AHB1ENR_GPIOHEN,    \
                             .rcc_ahbenr_gpio_sda = RCC_AHB1ENR_GPIOHEN,    \
                             .port_scl = GPIOH,                             \
                             .port_sda = GPIOH,                             \
                             .pin_scl = GPIO_PIN_4,                         \
                             .pin_sda = GPIO_PIN_5,                         \
                             .alternate_func = GPIO_AF4_I2C2}

#define I2C3_PP1_HW_INFO    {.i2c = I2C3,                                   \
                             .rcc_apbenr_i2cen = RCC_APB1ENR_I2C3EN,        \
                             .rcc_ahbenr_gpio_scl = RCC_AHB1ENR_GPIOAEN,    \
                             .rcc_ahbenr_gpio_sda = RCC_AHB1ENR_GPIOCEN,    \
                             .port_scl = GPIOA,                             \
                             .port_sda = GPIOC,                             \
                             .pin_scl = GPIO_PIN_8,                         \
                             .pin_sda = GPIO_PIN_9,                         \
                             .alternate_func = GPIO_AF4_I2C3}

#define I2C3_PP2_HW_INFO    {.i2c = I2C3,                                   \
                             .rcc_apbenr_i2cen = RCC_APB1ENR_I2C3EN,        \
                             .rcc_ahbenr_gpio_scl = RCC_AHB1ENR_GPIOHEN,    \
                             .rcc_ahbenr_gpio_sda = RCC_AHB1ENR_GPIOHEN,    \
                             .port_scl = GPIOH,                             \
                             .port_sda = GPIOH,                             \
                             .pin_scl = GPIO_PIN_7,                         \
                             .pin_sda = GPIO_PIN_8,                         \
                             .alternate_func = GPIO_AF4_I2C3}

typedef struct {
    I2C_TypeDef     *i2c;
    uint32_t        rcc_apbenr_i2cen;
    uint32_t        rcc_ahbenr_gpio_scl;
    uint32_t        rcc_ahbenr_gpio_sda;
    GPIO_TypeDef    *port_scl;
    GPIO_TypeDef    *port_sda;
    uint16_t        pin_scl;
    uint16_t        pin_sda;
    uint8_t         alternate_func;
} i2c_hw_info_t;

typedef struct i2c {
    i2c_num_t i2c_num;
    i2c_pins_pack_t i2c_pins_pack;
    i2c_hw_info_t hw_info;
    I2C_HandleTypeDef hal_handle;
} i2c_t;

i2c_hw_info_t I2C_HW_INFO_MAPPING[I2C_NUM_MAX][I2C_PINS_PACK_MAX] = {
    {I2C1_PP1_HW_INFO, I2C1_PP2_HW_INFO, I2C1_PP3_HW_INFO},
    {I2C2_PP1_HW_INFO, I2C2_PP2_HW_INFO, I2C2_PP3_HW_INFO},
    {I2C3_PP1_HW_INFO, I2C3_PP2_HW_INFO, 0               }
};

static i2c_hw_info_t i2c_get_hw_info(i2c_num_t i2c_num, i2c_pins_pack_t i2c_pins_pack)
{
    i2c_hw_info_t hw_info;
    hw_info = I2C_HW_INFO_MAPPING[i2c_num][i2c_pins_pack];

    return hw_info;
}

i2c_handle_t i2c_init(i2c_config_t *config)
{
    i2c_handle_t handle;
    handle = calloc(1, sizeof(i2c_t));
    if(handle == NULL)
    {
        return -1;
    }
    handle->hw_info = i2c_get_hw_info(config->i2c_num, config->i2c_pins_pack);

    int err;

    do {
        __IO uint32_t tmpreg = 0x00U;
        SET_BIT(RCC->APB1ENR, handle->hw_info.rcc_apbenr_i2cen);
        tmpreg = READ_BIT(RCC->APB1ENR, handle->hw_info.rcc_apbenr_i2cen);
        UNUSED(tmpreg);
    } while (0U);
    do {
        __IO uint32_t tmpreg = 0x00U;
        SET_BIT(RCC->AHB1ENR, handle->hw_info.rcc_ahbenr_gpio_scl);
        tmpreg = READ_BIT(RCC->AHB1ENR, handle->hw_info.rcc_ahbenr_gpio_scl);
        UNUSED(tmpreg);
    } while (0U);
    do {
        __IO uint32_t tmpreg = 0x00U;
        SET_BIT(RCC->AHB1ENR, handle->hw_info.rcc_ahbenr_gpio_sda);
        tmpreg = READ_BIT(RCC->AHB1ENR, handle->hw_info.rcc_ahbenr_gpio_sda);
        UNUSED(tmpreg);
    } while (0U);

	handle->hal_handle.Instance = handle->hw_info.i2c;
    handle->hal_handle.Init.ClockSpeed = I2C_CLOCKSPEED_DEFAULT;
    handle->hal_handle.Init.DutyCycle = I2C_DUTYCYCLE_DEFAULT;
    handle->hal_handle.Init.OwnAddress1 = I2C_OWN_ADDRESS1_DEFAULT;
    handle->hal_handle.Init.AddressingMode = I2C_ADDRESSING_MODE_DEFAULT;
    handle->hal_handle.Init.DualAddressMode = I2C_DUAL_ADDRESS_MODE_DEFAULT;
    handle->hal_handle.Init.OwnAddress2 = I2C_ONW_ADDRESS2_DEFAULT;
    handle->hal_handle.Init.GeneralCallMode = I2C_GENERALCALL_MODE_DEFAULT;
    handle->hal_handle.Init.NoStretchMode = I2C_NOSTRETCH_MODE_DEFAULT;
    HAL_I2C_Init(&handle->hal_handle);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = handle->hw_info.pin_scl;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = handle->hw_info.alternate_func;
    HAL_GPIO_Init(handle->hw_info.port_scl, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = handle->hw_info.pin_sda;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = handle->hw_info.alternate_func;
    HAL_GPIO_Init(handle->hw_info.port_sda, &GPIO_InitStruct);

    handle->i2c_num = config->i2c_num;
    handle->i2c_pins_pack = config->i2c_pins_pack;

    return handle;
}

I2C_HandleTypeDef i2c_get_I2C_HandleTypeDef(i2c_handle_t handle)
{
	return handle->hal_handle;
}
