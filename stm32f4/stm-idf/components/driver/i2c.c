#include "driver/i2c.h"

#define I2C_DUTYCYCLE_DEFAULT           I2C_DUTYCYCLE_2             /*!< Default I2C duty cycle */
#define I2C_CLOCKSPEED_DEFAULT          100000                      /*!< Default I2C clock speed */
#define I2C_OWN_ADDRESS1_DEFAULT        0                           /*!< Default I2C own address 1 */
#define I2C_ADDRESSING_MODE_DEFAULT     I2C_ADDRESSINGMODE_7BIT     /*!< Default I2C address mode */
#define I2C_DUAL_ADDRESS_MODE_DEFAULT   I2C_DUALADDRESS_DISABLE     /*!< Default I2C dual address mode */
#define I2C_OWN_ADDRESS2_DEFAULT        0                           /*!< Default I2C own address 2 */
#define I2C_GENERALCALL_MODE_DEFAULT    I2C_GENERALCALL_DISABLE     /*!< Default I2C general call mode */
#define I2C_NOSTRETCH_MODE_DEFAULT      I2C_NOSTRETCH_DISABLE       /*!< Default I2C nostretch mode */

#define GPIO_MODE_DEFAULT               GPIO_MODE_AF_OD             /*!< Default GPIO mode */
#define GPIO_PULL_REG_DEFAULT           GPIO_PULLUP                 /*!< Default pull registor mode */
#define GPIO_SPEED_FREQ_DEFAULT         GPIO_SPEED_FREQ_VERY_HIGH   /*!< Default GPIO speed */

static const char* I2C_TAG = "DRIVER I2C";
#define I2C_CHECK(a, str, ret)  if(!(a)) {                                             \
        STM_LOGE(I2C_TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);      \
        return (ret);                                                                  \
        }

#define I2C_INIT_ERR_STR        "i2c init error"
#define I2C_MALLOC_ERR_STR      "i2c malloc error"
#define I2C_NUM_ERR_STR         "i2c num error"
#define I2C_PINS_PACK_ERR_STR   "i2c pins pack error"
#define I2C_CLOCKSPEED_ERR_STR  "i2c clock speed error"
#define I2C_TRANS_ERR_STR       "i2c transmit data error"
#define I2C_REC_ERR_STR         "i2c receive data error"

/*
 * I2C Hardware Information Define.
 */
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

/*
 * I2C Hardware Information Structure.
 */
typedef struct {
    I2C_TypeDef     *i2c;                   /*!< I2C */
    uint32_t        rcc_apbenr_i2cen;       /*!< I2C RCC APBENR register */
    uint32_t        rcc_ahbenr_gpio_scl;    /*!< SCL GPIO RCC AHPENR register */
    uint32_t        rcc_ahbenr_gpio_sda;    /*!< SDA GPIO RCC AHPENR register */
    GPIO_TypeDef    *port_scl;              /*!< SCL General Purpose I/O */
    GPIO_TypeDef    *port_sda;              /*!< SDA General Purpose I/O */
    uint16_t        pin_scl;                /*!< SCL GPIO Pin */
    uint16_t        pin_sda;                /*!< SDA GPIO Pin */
    uint8_t         alternate_func;         /*!< I2C Alternate Function */
} i2c_hw_info_t;

/* 
 * I2C Handle.
 */
static I2C_HandleTypeDef i2c_handle[I2C_NUM_MAX];

/*
 * I2C Hardware Information Mapping Table.
 */
static i2c_hw_info_t I2C_HW_INFO_MAPPING[I2C_NUM_MAX][I2C_PINS_PACK_MAX] = {
    {I2C1_PP1_HW_INFO, I2C1_PP2_HW_INFO, I2C1_PP3_HW_INFO},
    {I2C2_PP1_HW_INFO, I2C2_PP2_HW_INFO, I2C2_PP3_HW_INFO},
    {I2C3_PP1_HW_INFO, I2C3_PP2_HW_INFO, {0}             }
};

static i2c_hw_info_t _i2c_get_hw_info(i2c_num_t i2c_num, i2c_pins_pack_t i2c_pins_pack)
{
    i2c_hw_info_t hw_info;
    hw_info = I2C_HW_INFO_MAPPING[i2c_num][i2c_pins_pack];

    return hw_info;
}

stm_err_t i2c_config(i2c_config_t *config)
{
    /* Check input condition */
    I2C_CHECK(config, I2C_INIT_ERR_STR, STM_ERR_INVALID_ARG);
    I2C_CHECK(config->i2c_num < I2C_NUM_MAX, I2C_NUM_ERR_STR, STM_ERR_INVALID_ARG);
    I2C_CHECK(config->i2c_pins_pack < I2C_PINS_PACK_MAX, I2C_PINS_PACK_ERR_STR, STM_ERR_INVALID_ARG);

    /* Get I2C hardware information */
    i2c_hw_info_t hw_info = _i2c_get_hw_info(config->i2c_num, config->i2c_pins_pack);

    /* Enable I2C clock */
    uint32_t tmpreg = 0x00;
    SET_BIT(RCC->APB1ENR, hw_info.rcc_apbenr_i2cen);
    tmpreg = READ_BIT(RCC->APB1ENR, hw_info.rcc_apbenr_i2cen);
    UNUSED(tmpreg);

    /* Enable SCL GPIO Port clock */
    tmpreg = 0x00;
    SET_BIT(RCC->AHB1ENR, hw_info.rcc_ahbenr_gpio_scl);
    tmpreg = READ_BIT(RCC->AHB1ENR, hw_info.rcc_ahbenr_gpio_scl);
    UNUSED(tmpreg);

    /* Enable SDA GPIO Port clock */
    tmpreg = 0x00;
    SET_BIT(RCC->AHB1ENR, hw_info.rcc_ahbenr_gpio_sda);
    tmpreg = READ_BIT(RCC->AHB1ENR, hw_info.rcc_ahbenr_gpio_sda);
    UNUSED(tmpreg);

    int err;

    /* Configure I2C */
    i2c_handle[config->i2c_num].Instance = hw_info.i2c;
    i2c_handle[config->i2c_num].Init.ClockSpeed = config->clk_speed;
    i2c_handle[config->i2c_num].Init.DutyCycle = I2C_DUTYCYCLE_DEFAULT;
    i2c_handle[config->i2c_num].Init.OwnAddress1 = I2C_OWN_ADDRESS1_DEFAULT;
    i2c_handle[config->i2c_num].Init.AddressingMode = I2C_ADDRESSING_MODE_DEFAULT;
    i2c_handle[config->i2c_num].Init.DualAddressMode = I2C_DUAL_ADDRESS_MODE_DEFAULT;
    i2c_handle[config->i2c_num].Init.OwnAddress2 = I2C_OWN_ADDRESS2_DEFAULT;
    i2c_handle[config->i2c_num].Init.GeneralCallMode = I2C_GENERALCALL_MODE_DEFAULT;
    i2c_handle[config->i2c_num].Init.NoStretchMode = I2C_NOSTRETCH_MODE_DEFAULT;
    err = HAL_I2C_Init(&i2c_handle[config->i2c_num]);
    I2C_CHECK(err == HAL_OK, I2C_INIT_ERR_STR, STM_FAIL);
    
    /* Configure SCL Pin */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = hw_info.pin_scl;
    GPIO_InitStruct.Mode = GPIO_MODE_DEFAULT;
    GPIO_InitStruct.Pull = GPIO_PULL_REG_DEFAULT;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_DEFAULT;
    GPIO_InitStruct.Alternate = hw_info.alternate_func;
    HAL_GPIO_Init(hw_info.port_scl, &GPIO_InitStruct);

    /* Configure SDA Pin */
    GPIO_InitStruct.Pin = hw_info.pin_sda;
    GPIO_InitStruct.Mode = GPIO_MODE_DEFAULT;
    GPIO_InitStruct.Pull = GPIO_PULL_REG_DEFAULT;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_DEFAULT;
    GPIO_InitStruct.Alternate = hw_info.alternate_func;
    HAL_GPIO_Init(hw_info.port_sda, &GPIO_InitStruct);

    return STM_OK;
}

stm_err_t i2c_write_bytes(i2c_num_t i2c_num, uint16_t dev_addr, uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
    I2C_CHECK(i2c_num < I2C_NUM_MAX, I2C_TRANS_ERR_STR, STM_ERR_INVALID_ARG);

    int ret = HAL_I2C_Master_Transmit(&i2c_handle[i2c_num], dev_addr, data, length, timeout_ms);
    I2C_CHECK(!ret, I2C_TRANS_ERR_STR, STM_FAIL);

    return STM_OK;
}

stm_err_t i2c_read_bytes(i2c_num_t i2c_num, uint16_t dev_addr, uint8_t *buf, uint16_t length, uint32_t timeout_ms)
{
    I2C_CHECK(i2c_num < I2C_NUM_MAX, I2C_REC_ERR_STR, STM_ERR_INVALID_ARG)

    int ret = HAL_I2C_Master_Receive(&i2c_handle[i2c_num], dev_addr, buf, length, timeout_ms);
    I2C_CHECK(!ret, I2C_REC_ERR_STR, STM_FAIL);

    return STM_OK;
}
