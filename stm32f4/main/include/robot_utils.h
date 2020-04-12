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

#ifndef _ROBOT_UTILS_H_
#define _ROBOT_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#include "stm32f4xx_hal.h"

/*
 * @brief   Get embedded time in milisecond.
 * @param   None.
 * @return  Time in ms.
 */
uint32_t millis(void);

/*
 * @brief   Constrain data.
 * @param   x Data.
 * @param   low_val Lower limit.
 * @param   high_val Upper limit.
 * @return  Value after constrain.
 */
float constrain(float x, float low_val, float high_val);


#ifdef __cplusplus
}
#endif

#endif /* _ROBOT_UTILS_H_ */
