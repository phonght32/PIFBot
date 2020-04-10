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

#ifndef _IMU_H_
#define _IMU_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stdlib.h"

typedef struct {
    int16_t x_axis;     
    int16_t y_axis;     
    int16_t z_axis;     
} imu_raw_data_t;

typedef struct {
    float x_axis;       
    float y_axis;       
    float z_axis;       
} imu_scale_data_t;

typedef struct {
    int16_t x_axis;     
    int16_t y_axis;     
    int16_t z_axis;     
} imu_cali_data_t;

typedef struct {
    int16_t x_axis;     
    int16_t y_axis;     
    int16_t z_axis;     
} imu_bias_data_t;

typedef struct {
    float x_axis;
    float y_axis;
    float z_axis;
} imu_mag_sens_adj_t;

typedef struct {
    float x_axis;
    float y_axis;
    float z_axis;
} imu_mag_scale_t;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} imu_quat_data_t;


#ifdef __cplusplus
}
#endif

#endif /* _IMU_DATA_H_ */