// MIT License

// Copyright (c) 2020 thanhphong98

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

#ifndef _SOFTWARE_RESOLVER_H_
#define _SOFTWARE_RESOLVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#include "stm32f4xx_hal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "stm_err.h"
#include "stm_log.h"
#include "driver/timer.h"

typedef struct software_resolver *software_resolver_handle_t;

typedef struct {
	timer_num_t 			timer_num;              /*!< Timer num */
	timer_pins_pack_t		timer_pins_pack;        /*!< Timer pins pack */
	uint32_t 				max_reload;             /*!< Max reload value */
	timer_counter_mode_t	counter_mode;           /*!< Counter mode */
} software_resolver_config_t;

/*
 * @brief   Initialize software resolver.
 * @param   config Struct pointer
 * @return  
 *      - Software resolver handle structure: Success.
 *      - 0: Fail.
 */
software_resolver_handle_t software_resolver_config(software_resolver_config_t *config);

/*
 * @brief   Start software resolver.
 * @param   handle Handle structure.
 * @return 
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t software_resolver_start(software_resolver_handle_t handle);

/*
 * @brief   Stop software resolver.
 * @param   handle Handle structure.
 * @return 
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t software_resolver_stop(software_resolver_handle_t handle);

/*
 * @brief   Get software resolver counter value.
 * @param   handle Handle structure.
 * @param   value Counter value.
 * @return 
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t software_resolver_get_value(software_resolver_handle_t handle, uint32_t *value);

/*
 * @brief   Set software resolver counter value.
 * @param   handle Handle structure.
 * @param   value Counter value.
 * @return 
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t software_resolver_set_value(software_resolver_handle_t handle, uint32_t value);

/*
 * @brief   Set software resolver counter mode.
 * @param   handle Handle structure.
 * @param   counter_mode Counter mode.
 * @return 
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t software_resolver_set_mode(software_resolver_handle_t handle, timer_counter_mode_t counter_mode);



#ifdef __cplusplus
}
#endif

#endif /* _SOFTWARE_RESOLVER_H_ */





















