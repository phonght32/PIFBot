#ifndef _UTILS_H_
#define _UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"


/*
 * @brief 	Initialize timer to count system time.
 * @param	None.
 * @return	None.
 */
void timer_interval_init(void);

/*
 * @brief	Get embedded time in milisecond.
 * @param	None.
 * @return	Time in ms.
 */
uint32_t millis(void);

/*
 * @brief	Constrain data.
 * @param	x Data.
 * @param 	low_val	Lower limit.
 * @param	high_val Upper limit.
 * @return	Value after constrain.
 */
float constrain(float x, float low_val, float high_val);

#ifdef __cplusplus
}
#endif

#endif /* _UTILS_H_ */
