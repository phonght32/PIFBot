#include "stm32f4xx_hal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stm_log.h"
#include "i2c.h"
#include "mpu6050.h"

const char *TAG = "APP_MAIN";

// #define TEST_RAW
// #define TEST_CALI
#define TEST_SCALE

mpu6050_handle_t mpu6050_handle;


#ifdef TEST_RAW
imu_raw_data_t gyro_data, accel_data;
#endif

#ifdef TEST_CALI
imu_cali_data_t gyro_data, accel_data;
#endif

#ifdef TEST_SCALE
imu_scale_data_t gyro_data, accel_data;
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

    mpu6050_config_t mpu6050_cfg;
    mpu6050_cfg.afs_sel = MPU6050_AFS_SEL_8G;
    mpu6050_cfg.clksel = MPU6050_CLKSEL_X_GYRO_REF;
    mpu6050_cfg.dlpf_cfg =  MPU6050_44ACCEL_42GYRO_BW_HZ;
    mpu6050_cfg.fs_sel = MPU6050_FS_SEL_2000;
    mpu6050_cfg.sleep_mode = MPU6050_DISABLE_SLEEP_MODE;
    mpu6050_cfg.i2c_num = I2C_NUM_1;
    mpu6050_handle = mpu6050_config(&mpu6050_cfg);

    mpu6050_auto_calib(mpu6050_handle);

    while(1)
    {
#ifdef TEST_RAW 
        mpu6050_get_accel_raw(mpu6050_handle, &accel_data);
        mpu6050_get_gyro_raw(mpu6050_handle, &gyro_data);
        STM_LOGI(TAG, "accel_x: %d\taccel_y: %d\taccel_z: %d", accel_data.x_axis, accel_data.y_axis, accel_data.z_axis);
        STM_LOGI(TAG, "gyro_x : %d\tgyro_y:  %d\tgyro_z:  %d", gyro_data.x_axis, gyro_data.y_axis, gyro_data.z_axis);
#endif

#ifdef TEST_CALI
        mpu6050_get_accel_cali(mpu6050_handle, &accel_data);
        mpu6050_get_gyro_cali(mpu6050_handle, &gyro_data);
        STM_LOGI(TAG, "accel_x: %d\taccel_y: %d\taccel_z: %d", accel_data.x_axis, accel_data.y_axis, accel_data.z_axis);
        STM_LOGI(TAG, "gyro_x : %d\tgyro_y:  %d\tgyro_z:  %d", gyro_data.x_axis, gyro_data.y_axis, gyro_data.z_axis);
#endif

#ifdef TEST_SCALE
        mpu6050_get_accel_scale(mpu6050_handle, &accel_data);
        mpu6050_get_gyro_scale(mpu6050_handle, &gyro_data);
        STM_LOGI(TAG, "accel_x: %f\taccel_y: %f\taccel_z: %f", accel_data.x_axis, accel_data.y_axis, accel_data.z_axis);
        STM_LOGI(TAG, "gyro_x : %f\tgyro_y:  %f\tgyro_z:  %f", gyro_data.x_axis, gyro_data.y_axis, gyro_data.z_axis);
#endif
        STM_LOGI(TAG, "********************************************");
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