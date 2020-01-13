#ifndef _UTILS_H_
#define _UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

void timer_interval_init(void);
uint32_t millis(void);
float constrain(float x, float low_val, float high_val);

#ifdef __cplusplus
}
#endif

#endif /* _UTILS_H_ */
