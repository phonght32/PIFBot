// MIT License

// Copyright (c) 2020 thanhphong98 & thuanpham98

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

#ifndef _IMU_MADGWICK_H_
#define _IMU_MADGWICK_H_

#ifdef __cplusplus 
extern "C" {
#endif

#include "stdint.h"
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "imu.h"
#include "stm_log.h"

typedef struct madgwick *madgwick_handle_t;

typedef struct {
    float beta;
    float sample_freq;
} madgwick_config_t;

/*
 * @brief   Configure Madgwick AHRS parameters.
 * @param   config Struct pointer.
 * @return
 *      - Madgwick handle structure: Success.
 *      - 0: Fail.
 */
madgwick_handle_t madgwick_config(madgwick_config_t *config);

/*
 * @brief   Set beta value.
 * @param   handle Handle structure.
 * @param   beta Beta.
 * @return  None.
 */
void madgwick_set_beta(madgwick_handle_t handle, float beta);

/*
 * @brief   Set sample frequency value.
 * @param   handle Handle structure.
 * @param   sample_freq Sample frequency.
 * @return  None.
 */
void madgwick_set_sample_frequency(madgwick_handle_t handle, float sample_freq);

/*
 * @brief   Get quaternion.
 * @param   handle Handle structure.
 * @param   quat_data Quaternion.
 * @return  None.
 */
void madgwick_get_quaternion(madgwick_handle_t handle, imu_quat_data_t *quat_data);

/*
 * @brief   Update Madgwick AHRS quaternion with 6 motions.
 * @param   handle Handle structure.
 * @param   gx Gyroscope along x axis.
 * @param   gy Gyroscope along y axis.
 * @param   gz Gyroscope along z axis.
 * @param   ax Accelerometer along x axis.
 * @param   ay Accelerometer along y axis.
 * @param   az Accelerometer along z axis.
 * @return  None.
 */
void madgwick_update_6dof(madgwick_handle_t handle, float gx, float gy, float gz, float ax, float ay, float az);

/*
 * @brief   Update Madgwick AHRS quaternion with 9 motions.
 * @param   handle Handle structure.
 * @param   gx Gyroscope along x axis.
 * @param   gy Gyroscope along y axis.
 * @param   gz Gyroscope along z axis.
 * @param   ax Accelerometer along x axis.
 * @param   ay Accelerometer along y axis.
 * @param   az Accelerometer along z axis.
 * @param   mx Magnetometer along x axis.
 * @param   my Magnetometer along y axis.
 * @param   mz Magnetometer along z axis.
 * @return  None.
 */
void madgwick_update_9dof(madgwick_handle_t handle, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

#ifdef __cplusplus 
}
#endif

#endif /* _IMU_MADGWICK_H_ */