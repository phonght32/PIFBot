#include "stepmotor.h"

#define STEPMOTOR_INIT_ERR_STR                  "step motor init error"
#define STEPMOTOR_SET_DIR_ERR_STR               "step motor set direction error"
#define STEPMOTOR_TOGGLE_DIR_ERR_STR            "step motor toggle direction error"
#define STEPMOTOR_SET_PWM_FREQ_ERR_STR          "step motor set pwm frequency error"
#define STEPMOTOR_SET_PWM_DUTYCYCLE_ERR_STR     "step motor set pwm duty cycle error"
#define STEPMOTOR_START_ERR_STR                 "step motor start error"
#define STEPMOTOR_STOP_ERR_STR                  "step motor stop error"

static const char* STEPMOTOR_TAG = "STEP MOTOR";
#define STEPMOTOR_CHECK(a, str, ret)  if(!(a)) {                                            \
        STM_LOGE(STEPMOTOR_TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);     \
        return (ret);                                                                       \
        }

/* Mutex define */
#define mutex_lock(x)       while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x)     xSemaphoreGive(x)
#define mutex_create()      xSemaphoreCreateMutex()
#define mutex_destroy(x)    vQueueDelete(x)

typedef struct stepmotor {
    gpio_port_t         dir_gpio_port;          /*!< Pin dir GPIO Port */
    gpio_num_t          dir_gpio_num;           /*!< Pin dir GPIO Num */
    timer_num_t         pulse_timer_num;        /*!< Pin pulse Timer Num */
    timer_pins_pack_t   pulse_timer_pins_pack;  /*!< Pin pulse Timer Pins Pack */
    timer_channel_t     pulse_timer_channel;    /*!< Pin pulse Timer Channel */
    bool                dir;                    /*!< Direction */
    uint32_t            freq_hz;                /*!< PWM frequency in Hz */
    uint8_t             duty;                   /*!< PWM duty cycle in % */
    stepmotor_status_t  status;                 /*!< Step motor status */
    SemaphoreHandle_t   lock;                   /*!< Step motor mutex */
} stepmotor_t;

stepmotor_handle_t stepmotor_config(stepmotor_config_t *config)
{
    /* Check input conditions */
    STEPMOTOR_CHECK(config, STEPMOTOR_INIT_ERR_STR, NULL);
    STEPMOTOR_CHECK(config->dir_gpio_port < GPIO_PORT_MAX, STEPMOTOR_INIT_ERR_STR, NULL);
    STEPMOTOR_CHECK(config->dir_gpio_num < GPIO_NUM_MAX, STEPMOTOR_INIT_ERR_STR, NULL);
    STEPMOTOR_CHECK(config->pulse_timer_num < TIMER_NUM_MAX, STEPMOTOR_INIT_ERR_STR, NULL);
    STEPMOTOR_CHECK(config->pulse_timer_pins_pack < TIMER_PINS_PACK_MAX, STEPMOTOR_INIT_ERR_STR, NULL);
    STEPMOTOR_CHECK(config->pulse_timer_channel < TIMER_CHANNEL_MAX, STEPMOTOR_INIT_ERR_STR, NULL);

    /* Allocate memory for handle structure */
    stepmotor_handle_t handle = calloc(1,sizeof(stepmotor_t));
    STEPMOTOR_CHECK(handle, STEPMOTOR_INIT_ERR_STR, NULL);

    int ret;

    /* Configure pin direction */
    gpio_config_t dir_cfg;
    dir_cfg.gpio_port = config->dir_gpio_port;
    dir_cfg.gpio_num = config->dir_gpio_num;
    dir_cfg.mode = GPIO_OUTPUT;
    dir_cfg.pull_mode = GPIO_REG_PULL_NONE;
    ret = gpio_config(&dir_cfg);
    STEPMOTOR_CHECK(!ret, STEPMOTOR_INIT_ERR_STR, NULL);
    ret = gpio_set_level(config->dir_gpio_port, config->dir_gpio_num, 0);
    STEPMOTOR_CHECK(!ret, STEPMOTOR_INIT_ERR_STR, NULL);

    /* Configure pin pulse */
    pwm_config_t pulse_cfg;
    pulse_cfg.timer_num = config->pulse_timer_num;
    pulse_cfg.timer_pins_pack = config->pulse_timer_pins_pack;
    pulse_cfg.timer_channel = config->pulse_timer_channel;
    ret = pwm_config(&pulse_cfg);
    STEPMOTOR_CHECK(!ret, STEPMOTOR_INIT_ERR_STR, NULL);
    ret = pwm_set_params(config->pulse_timer_num, config->pulse_timer_channel, 0, 0);
    STEPMOTOR_CHECK(!ret, STEPMOTOR_INIT_ERR_STR, NULL);

    /* Update handle structure */
    handle->dir_gpio_port = config->dir_gpio_port;
    handle->dir_gpio_num = config->dir_gpio_num;
    handle->pulse_timer_num = config->pulse_timer_num;
    handle->pulse_timer_pins_pack = config->pulse_timer_pins_pack;
    handle->pulse_timer_channel = config->pulse_timer_channel;
    handle->dir = 0;
    handle->freq_hz = 0;
    handle->duty = 0;
    handle->status = STEPMOTOR_READY;
    handle->lock = mutex_create();

    return handle;
}

stm_err_t stepmotor_set_dir(stepmotor_handle_t handle, bool dir)
{
    mutex_lock(handle->lock);
    int ret = gpio_set_level(handle->dir_gpio_port, handle->dir_gpio_num, dir);
    STEPMOTOR_CHECK(!ret, STEPMOTOR_SET_DIR_ERR_STR, STM_FAIL);

    handle->dir = dir;
    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t stepmotor_toggle_dir(stepmotor_handle_t handle)
{
    mutex_lock(handle->lock);
    int ret = gpio_toggle_level(handle->dir_gpio_port, handle->dir_gpio_num);
    STEPMOTOR_CHECK(!ret, STEPMOTOR_TOGGLE_DIR_ERR_STR, STM_FAIL);

    handle->dir = !handle->dir;
    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t stepmotor_set_pwm_freq(stepmotor_handle_t handle, uint32_t freq_hz)
{
    mutex_lock(handle->lock);
    int ret = pwm_set_params(handle->pulse_timer_num, handle->pulse_timer_channel, freq_hz, handle->duty);
    STEPMOTOR_CHECK(!ret, STEPMOTOR_SET_PWM_FREQ_ERR_STR, STM_FAIL);

    handle->freq_hz = freq_hz;
    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t stepmotor_set_pwm_duty(stepmotor_handle_t handle, uint8_t duty)
{
    mutex_lock(handle->lock);
    int ret = pwm_set_params(handle->pulse_timer_num, handle->pulse_timer_channel, handle->freq_hz, duty);
    STEPMOTOR_CHECK(!ret, STEPMOTOR_SET_PWM_DUTYCYCLE_ERR_STR, STM_FAIL);

    handle->duty = duty;
    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t stepmotor_start(stepmotor_handle_t handle)
{
    mutex_lock(handle->lock);
    int ret = pwm_start(handle->pulse_timer_num, handle->pulse_timer_channel);
    STEPMOTOR_CHECK(!ret, STEPMOTOR_START_ERR_STR, STM_FAIL);

    handle->status = STEPMOTOR_RUNNING;
    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t stepmotor_stop(stepmotor_handle_t handle)
{
    mutex_lock(handle->lock);
    int ret = pwm_stop(handle->pulse_timer_num, handle->pulse_timer_channel);
    STEPMOTOR_CHECK(!ret, STEPMOTOR_STOP_ERR_STR, STM_FAIL);

    handle->status = STEPMOTOR_STOP;
    mutex_unlock(handle->lock);
    return STM_OK;
}