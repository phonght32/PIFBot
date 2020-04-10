#include "software_resolver.h"

#define SOFTWARE_RESOLVER_INIT_ERR_STR				"software resolver init error"
#define SOFTWARE_RESOLVER_SET_COUNTER_MODE_ERR_STR	"software resolver set counter mode error"
#define SOFTWARE_RESOLVER_GET_VALUE_ERR_STR			"software resolver get counter value error"
#define SOFTWARE_RESOLVER_SET_VALUE_ERR_STR			"software resolver set counter value error"
#define SOFTWARE_RESOLVER_START_ERR_STR				"software resolver start error"
#define SOFTWARE_RESOLVER_STOP_ERR_STR				"software resolver stop error"

static const char* SOFTWARE_RESOLVER_TAG = "SOFTWARE RESOLVER";
#define SOFTWARE_RESOLVER_CHECK(a, str, ret)  if(!(a)) {                                            \
        STM_LOGE(SOFTWARE_RESOLVER_TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);     \
        return (ret);                                                                       		\
        }

/* Mutex define */
#define mutex_lock(x)       while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x)     xSemaphoreGive(x)
#define mutex_create()      xSemaphoreCreateMutex()
#define mutex_destroy(x)    vQueueDelete(x)

typedef struct software_resolver {
	timer_num_t 			timer_num;				/*!< Timer num */
	timer_pins_pack_t		timer_pins_pack;		/*!< Timer pins pack */
	uint32_t 				max_reload;				/*!< Max reload value */
	timer_counter_mode_t	counter_mode;			/*!< Counter mode */
	SemaphoreHandle_t   	lock;                   /*!< Software resolver mutex */
} software_resolver_t;

software_resolver_handle_t software_resolver_config(software_resolver_config_t *config)
{
	/* Check input conditions */
	SOFTWARE_RESOLVER_CHECK(config, SOFTWARE_RESOLVER_INIT_ERR_STR, NULL);
	SOFTWARE_RESOLVER_CHECK(config->timer_num < TIMER_NUM_MAX, SOFTWARE_RESOLVER_INIT_ERR_STR, NULL);
	SOFTWARE_RESOLVER_CHECK(config->timer_pins_pack < TIMER_PINS_PACK_MAX, SOFTWARE_RESOLVER_INIT_ERR_STR, NULL);
	SOFTWARE_RESOLVER_CHECK(config->counter_mode < TIMER_COUNTER_MODE_MAX, SOFTWARE_RESOLVER_INIT_ERR_STR, NULL);
	
	/* Allocate memory for handle structure */
	software_resolver_handle_t handle = calloc(1, sizeof(software_resolver_t));
	SOFTWARE_RESOLVER_CHECK(handle, SOFTWARE_RESOLVER_INIT_ERR_STR, NULL);
	
	int ret;
	
	/* Configure ETR */
	etr_config_t software_resolver_cfg;
	software_resolver_cfg.timer_num = config->timer_num;
	software_resolver_cfg.timer_pins_pack = config->timer_pins_pack;
	software_resolver_cfg.max_reload = config->max_reload;
	software_resolver_cfg.counter_mode = config->counter_mode;
	ret = etr_config(&software_resolver_cfg);
	SOFTWARE_RESOLVER_CHECK(!ret, SOFTWARE_RESOLVER_INIT_ERR_STR, NULL);
	
	/* Update handle structure */
	handle->timer_num = config->timer_num;
	handle->timer_pins_pack = config->timer_pins_pack;
	handle->counter_mode = config->counter_mode;
	handle->max_reload = config->max_reload;
	handle->lock = mutex_create();

	return handle;
}

stm_err_t software_resolver_start(software_resolver_handle_t handle)
{
	mutex_lock(handle->lock);
	int ret = etr_start(handle->timer_num);
	SOFTWARE_RESOLVER_CHECK(!ret, SOFTWARE_RESOLVER_START_ERR_STR, STM_FAIL);
	mutex_unlock(handle->lock);

	return STM_OK;
}

stm_err_t software_resolver_stop(software_resolver_handle_t handle)
{
	mutex_lock(handle->lock);
	int ret = etr_stop(handle->timer_num);
	SOFTWARE_RESOLVER_CHECK(!ret, SOFTWARE_RESOLVER_STOP_ERR_STR, STM_FAIL);
	mutex_unlock(handle->lock);

	return STM_OK;
}

stm_err_t software_resolver_get_value(software_resolver_handle_t handle, uint32_t *value)
{
	mutex_lock(handle->lock);
	int ret = etr_get_value(handle->timer_num, value);
	SOFTWARE_RESOLVER_CHECK(!ret, SOFTWARE_RESOLVER_GET_VALUE_ERR_STR, STM_FAIL);
	mutex_unlock(handle->lock);

	return STM_OK;
}

stm_err_t software_resolver_set_value(software_resolver_handle_t handle, uint32_t value)
{
	mutex_lock(handle->lock);
	int ret = etr_set_value(handle->timer_num, value);
	SOFTWARE_RESOLVER_CHECK(!ret, SOFTWARE_RESOLVER_SET_VALUE_ERR_STR, STM_FAIL);
	mutex_unlock(handle->lock);

	return STM_OK;
}

stm_err_t software_resolver_set_mode(software_resolver_handle_t handle, timer_counter_mode_t counter_mode)
{
	mutex_lock(handle->lock);
	int ret = etr_set_mode(handle->timer_num, counter_mode);
	SOFTWARE_RESOLVER_CHECK(!ret, SOFTWARE_RESOLVER_SET_COUNTER_MODE_ERR_STR, STM_FAIL);
	mutex_unlock(handle->lock);

	return STM_OK;
}