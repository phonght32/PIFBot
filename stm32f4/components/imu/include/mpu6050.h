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

#ifndef _MPU6050_H_
#define _MPU6050_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stdlib.h"
#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "stm_log.h"
#include "imu.h"

typedef struct mpu6050 *mpu6050_handle_t;   /*!< MPU6050 Handle Typedef */

typedef enum {
    MPU6050_CLKSEL_INTERNAL_8_MHZ = 0,      /*!< Internal 8 MHz oscillator */
    MPU6050_CLKSEL_X_GYRO_REF,              /*!< PLL with X axis gyroscope reference */
    MPU6050_CLKSEL_Y_GYRO_REF,              /*!< PLL with Y axis gyroscope reference */
    MPU6050_CLKSEL_Z_GYRO_REF,              /*!< PLL with Z axis gyroscope reference */
    MPU6050_CLKSEL_EXTERNAL_32_768_KHZ,     /*!< PLL with eternal 32.768 KHz reference */
    MPU6050_CLKSEL_EXTERNAL_19_2_MHZ,       /*!< PLL with external 19.2 MHz reference */
    MPU6050_TIM_GEN_RESET = 7               /*!< Stops the clock and keeps the timing generator in reset */
} mpu6050_clksel_t;

typedef enum {
    MPU6050_260ACCEL_256GYRO_BW_HZ = 0,     /*!< 260 Hz accelerometer bandwidth, 256 Hz gyroscope bandwidth */
    MPU6050_184ACCEL_188GYRO_BW_HZ,         /*!< 184 Hz accelerometer bandwidth, 188 Hz gyroscope bandwidth */
    MPU6050_94ACCEL_98GYRO_BW_HZ,           /*!< 94 Hz accelerometer bandwidth, 98 Hz gyroscope bandwidth */
    MPU6050_44ACCEL_42GYRO_BW_HZ,           /*!< 44 Hz accelerometer bandwidth, 42 Hz gyroscope bandwidth */
    MPU6050_21ACCEL_20GYRO_BW_HZ,           /*!< 21 Hz accelerometer bandwidth, 20 Hz gyroscope bandwidth */
    MPU6050_10ACCEL_10GYRO_BW_HZ,           /*!< 10 Hz accelerometer bandwidth, 10 Hz gyroscope bandwidth */
    MPU6050_5ACCEL_5GYRO_BW_HZ              /*!< 5 Hz accelerometer bandwidth, 5 Hz gyroscope bandwidth */
} mpu6050_dlpf_cfg_t;

typedef enum {
    MPU6050_DISABLE_SLEEP_MODE = 0,         /*!< Disable sleep mode */
    MPU6050_LOW_PWR_SLEEP_MODE              /*!< Low power mode */
} mpu6050_sleep_mode_t;

typedef enum {
    MPU6050_FS_SEL_250 = 0,     /*!< 250 deg/s */
    MPU6050_FS_SEL_500,         /*!< 500 deg/s */
    MPU6050_FS_SEL_1000,        /*!< 1000 deg/s */
    MPU6050_FS_SEL_2000         /*!< 2000 deg/s */
} mpu6050_fs_sel_t;

typedef enum {
    MPU6050_AFS_SEL_2G = 0,     /*!< 2g */
    MPU6050_AFS_SEL_4G,         /*!< 4g */
    MPU6050_AFS_SEL_8G,         /*!< 8g */
    MPU6050_AFS_SEL_16G         /*!< 16g */
} mpu6050_afs_sel_t;

typedef struct {
    mpu6050_clksel_t        clksel;         /*!< MPU6050 clock source */
    mpu6050_dlpf_cfg_t      dlpf_cfg;       /*!< MPU6050 digital low pass filter (DLPF) */
    mpu6050_sleep_mode_t    sleep_mode;     /*!< MPU6050 sleep mode */
    mpu6050_fs_sel_t        fs_sel;         /*!< MPU6050 gyroscope full scale range */
    mpu6050_afs_sel_t       afs_sel;        /*!< MPU6050 accelerometer full scale range */
    imu_bias_data_t         accel_bias;     /*!< Acceleromter bias */
    imu_bias_data_t         gyro_bias;      /*!< Gyroscope bias */
    i2c_num_t               i2c_num;        /*!< I2C num */
} mpu6050_config_t;

/*
 * @brief   Initialize I2C communication and configure MPU6050 's parameters 
 *          such as clock source, digital low pass filter (DLPF), sleep mode,
 *          gyroscope and accelerometer full scale range, bias value, ...
 * @note:   This function only get I2C_NUM to handler communication, not 
 *          configure I2C 's parameters. You have to self configure I2C before
 *          pass I2C into this function.
 * @param   config Struct pointer.
 * @return
 *      - MPU6050 handle structure: Success.
 *      - 0: Fail.
 */
mpu6050_handle_t mpu6050_config(mpu6050_config_t *config);

/*
 * @brief   Get accelerometer raw value.
 * @param   handle Handle structure.
 * @param   raw_data Raw data.
 * @return
 *      - STM_OK:   Success.
 *      - STM_FAIL: Fail.
 */
stm_err_t mpu6050_get_accel_raw(mpu6050_handle_t handle, imu_raw_data_t *raw_data);

/*
 * @brief   Get gyroscope raw data.
 * @param   handle Handle structure.
 * @param   raw_data Raw data.
 * @return
 *      - STM_OK:   Success.
 *      - STM_FAIL: Fail.
 */

stm_err_t mpu6050_get_accel_cali(mpu6050_handle_t handle, imu_cali_data_t *cali_data);

/*
 * @brief   Get gyroscope calibrated data.
 * @param   handle Handle structure.
 * @param   cali_data Calibrated data.
 * @return
 *      - STM_OK:   Success.
 *      - STM_FAIL: Fail.
 */

stm_err_t mpu6050_get_accel_scale(mpu6050_handle_t handle, imu_scale_data_t *scale_data);

/*
 * @brief   Get gyroscope scale data.
 * @param   handle Handle structure.
 * @param   scale_data Scaled data.
 * @return
 *      - STM_OK:   Success.
 *      - STM_FAIL: Fail.
 */

stm_err_t mpu6050_get_gyro_raw(mpu6050_handle_t handle, imu_raw_data_t *raw_data);

/*
 * @brief   Get accelerometer calibrated data.
 * @param   handle Handle structure.
 * @param   cali_data Calibrated data.
 * @return
 *      - STM_OK:   Success.
 *      - STM_FAIL: Fail.
 */

stm_err_t mpu6050_get_gyro_cali(mpu6050_handle_t handle, imu_cali_data_t *cali_data);

/*
 * @brief   Get accelerometer scale data.
 * @param   handle Handle structure.
 * @param   handle Handle structure.
 * @param   scale_data Scaled data.
 * @return
 *      - STM_OK:   Success.
 *      - STM_FAIL: Fail.
 */

stm_err_t mpu6050_get_gyro_scale(mpu6050_handle_t handle, imu_scale_data_t *scale_data);

/*
 * @brief   Set accelerometer bias value.
 * @param   handle Handle structure.
 * @param   accel_bias Bias data.
 * @return  None.
 */
void mpu6050_set_accel_bias(mpu6050_handle_t handle, imu_bias_data_t accel_bias);

/*
 * @brief   Set gyroscopre bias value.
 * @param   handle Handle structure.
 * @param   gyro_bias Bias data.
 * @return  None.
 */
void mpu6050_set_gyro_bias(mpu6050_handle_t handle, imu_bias_data_t gyro_bias);

/*
 * @brief   Get accelerometer bias value.
 * @param   handle Handle structure.
 * @param   accel_bias Bias data.
 * @return  None.
 */
void mpu6050_get_accel_bias(mpu6050_handle_t handle, imu_bias_data_t *accel_bias);

/*
 * @brief   Get gyroscopre bias value.
 * @param   handle Handle structure.
 * @param   gyro_bias Bias data.
 * @return  None.
 */
void mpu6050_get_gyro_bias(mpu6050_handle_t handle, imu_bias_data_t *gyro_bias);

/*
 * @brief   Auto calibrate all acceleromter and gyroscope bias value.
 * @param   handle Handle structure.
 * @param   handle MPU6050 handle structure.
 * @return  None.
 */
void mpu6050_auto_calib(mpu6050_handle_t handle);


#ifdef __cplusplus
}
#endif

#endif /* _MPU6050_H_ */