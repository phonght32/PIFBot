#ifndef _MPU6050_H_
#define _MPU6050_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

typedef struct {
    int16_t x_axis;
    int16_t y_axis;
    int16_t z_axis;
} mpu6050_raw_data_t;

typedef struct {
    int16_t x_axis;
    int16_t y_axis;
    int16_t z_axis;
} mpu6050_scaled_data_t;

typedef enum {
    MPU6050_CLKSEL_INTERNAL_8_MHZ = 0,
    MPU6050_CLKSEL_X_GYRO_REF,
    MPU6050_CLKSEL_Y_GYRO_REF,
    MPU6050_CLKSEL_Z_GYRO_REF,
    MPU6050_CLKSEL_EXTERNAL_32_768_KHZ,
    MPU6050_CLKSEL_EXTERNAL_19_2_MHZ,
    MPU6050_TIM_GEN_RESET = 7
} mpu6050_clksel_t;

typedef enum {
    MPU6050_260ACCEL_256GYRO_BW_HZ = 0,
    MPU6050_184ACCEL_188GYRO_BW_HZ,
    MPU6050_94ACCEL_98GYRO_BW_HZ,
    MPU6050_44ACCEL_42GYRO_BW_HZ,
    MPU6050_21ACCEL_20GYRO_BW_HZ,
    MPU6050_10ACCEL_10GYRO_BW_HZ,
    MPU6050_5ACCEL_5GYRO_BW_HZ
} mpu6050_dlpf_cfg_t;

typedef enum {
    MPU6050_DISABLE_SLEEP_MODE = 0,
    MPU6050_LOW_PWR_SLEEP_MODE
} mpu6050_sleep_mode_t;

typedef enum {
    MPU6050_FS_SEL_250 = 0,
    MPU6050_FS_SEL_500,
    MPU6050_FS_SEL_1000,
    MPU6050_FS_SEL_2000
} mpu6050_fs_sel_t;

typedef enum {
    MPU6050_AFS_SEL_2G = 0,
    MPU6050_AFS_SEL_4G,
    MPU6050_AFS_SEL_8G,
    MPU6050_AFS_SEL_16G
} mpu6050_afs_sel_t;

typedef struct {
    mpu6050_clksel_t        clksel;
    mpu6050_dlpf_cfg_t      dlpf_cfg;
    mpu6050_sleep_mode_t    sleep_mode;
    mpu6050_fs_sel_t        fs_sel;
    mpu6050_afs_sel_t       afs_sel;
} mpu6050_config_t;

int mpu6050_i2c_config(I2C_HandleTypeDef *i2c_handle);
int mpu6050_init(mpu6050_config_t *config);
int mpu6050_get_accel_raw(mpu6050_raw_data_t *raw_data);
int mpu6050_get_gyro_raw(mpu6050_raw_data_t *raw_data);
int mpu6050_get_accel_scale(mpu6050_raw_data_t *scale_data);
int mpu6050_get_gyro_scale(mpu6050_raw_data_t *scale_data);

#ifdef __cplusplus
}
#endif

#endif /* _MPU6050_H_ */