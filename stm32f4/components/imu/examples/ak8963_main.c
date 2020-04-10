#include "stm32f4xx_hal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stm_log.h"
#include "i2c.h"
#include "mpu9250.h"
#include "ak8963.h"

const char *TAG = "APP_MAIN";

#define TEST_RAW
// #define TEST_CALI
// #define TEST_SCALE

// #define AUTO_CALIB

mpu9250_handle_t mpu9250_handle;
ak8963_handle_t ak8963_handle;


#ifdef TEST_RAW
imu_raw_data_t gyro_data, accel_data, mag_data;
#endif

#ifdef TEST_CALI
imu_cali_data_t gyro_data, accel_data, mag_data;
#endif

#ifdef TEST_SCALE
imu_scale_data_t gyro_data, accel_data, mag_data;
#endif


static void system_clock_init(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

static void example_task(void* arg)
{
    i2c_config_t i2c_cfg;
    i2c_cfg.i2c_num = I2C_NUM_1;
    i2c_cfg.i2c_pins_pack = I2C_PINS_PACK_1;
    i2c_cfg.clk_speed = 100000;
    i2c_config(&i2c_cfg);

    mpu9250_config_t mpu9250_cfg;
    mpu9250_cfg.afs_sel = MPU9250_AFS_SEL_8G;
    mpu9250_cfg.clksel = MPU9250_CLKSEL_AUTO;
    mpu9250_cfg.dlpf_cfg =  MPU9250_41ACEL_42GYRO_BW_HZ;
    mpu9250_cfg.fs_sel = MPU9250_FS_SEL_2000;
    mpu9250_cfg.sleep_mode = MPU9250_DISABLE_SLEEP_MODE;
    mpu9250_cfg.i2c_num = I2C_NUM_1;
    mpu9250_handle = mpu9250_config(&mpu9250_cfg);

    ak8963_config_t ak8963_cfg;
    ak8963_cfg.opr_mode = AK8963_CONT_MEASUREMENT_2;
    ak8963_cfg.mfs_sel = AK8963_MFS_16BIT;
    ak8963_cfg.i2c_num = I2C_NUM_1;
    ak8963_handle = ak8963_config(&ak8963_cfg);

#ifdef AUTO_CALIB
    mpu9250_auto_calib(mpu9250_handle);
    ak8963_auto_calib(ak8963_handle);
#endif

    while(1)
    {
#ifdef TEST_RAW 
        mpu9250_get_accel_raw(mpu9250_handle, &accel_data);
        mpu9250_get_gyro_raw(mpu9250_handle, &gyro_data);
        ak8963_get_mag_raw(ak8963_handle, &mag_data);
        STM_LOGI(TAG, "accel_x: %d\taccel_y: %d\taccel_z: %d", accel_data.x_axis, accel_data.y_axis, accel_data.z_axis);
        STM_LOGI(TAG, "gyro_x : %d\tgyro_y:  %d\tgyro_z:  %d", gyro_data.x_axis, gyro_data.y_axis, gyro_data.z_axis);
        STM_LOGI(TAG, "mag_x:   %d\tmag_y:   %d\tmag_z:   %d", mag_data.x_axis, mag_data.y_axis, mag_data.z_axis);
#endif

#ifdef TEST_CALI
        mpu9250_get_accel_cali(mpu9250_handle, &accel_data);
        mpu9250_get_gyro_cali(mpu9250_handle, &gyro_data);
        ak8963_get_mag_cali(ak8963_handle, &mag_data);
        STM_LOGI(TAG, "accel_x: %d\taccel_y: %d\taccel_z: %d", accel_data.x_axis, accel_data.y_axis, accel_data.z_axis);
        STM_LOGI(TAG, "gyro_x : %d\tgyro_y:  %d\tgyro_z:  %d", gyro_data.x_axis, gyro_data.y_axis, gyro_data.z_axis);
        STM_LOGI(TAG, "mag_x:   %d\tmag_y:   %d\tmag_z:   %d", mag_data.x_axis, mag_data.y_axis, mag_data.z_axis);
#endif

#ifdef TEST_SCALE
        mpu9250_get_accel_scale(mpu9250_handle, &accel_data);
        mpu9250_get_gyro_scale(mpu9250_handle, &gyro_data);
        ak8963_get_mag_scale(ak8963_handle, &mag_data);
        STM_LOGI(TAG, "accel_x: %f\taccel_y: %f\taccel_z: %f", accel_data.x_axis, accel_data.y_axis, accel_data.z_axis);
        STM_LOGI(TAG, "gyro_x : %f\tgyro_y:  %f\tgyro_z:  %f", gyro_data.x_axis, gyro_data.y_axis, gyro_data.z_axis);
        STM_LOGI(TAG, "mag_x:   %f\tmag_y:   %f\tmag_z:   %f", mag_data.x_axis, mag_data.y_axis, mag_data.z_axis);
#endif

        STM_LOGI(TAG, "*****************************");
        vTaskDelay(200/portTICK_PERIOD_MS);
    }
}

int main(void)
{
    HAL_Init();
    system_clock_init();

    stm_log_init();
    stm_log_level_set("*", STM_LOG_NONE);
    stm_log_level_set("APP_MAIN", STM_LOG_INFO);

    xTaskCreate(example_task, "example_task", 512, NULL, 5, NULL);
    vTaskStartScheduler();
}