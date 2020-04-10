#include "stm32f4xx_hal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stm_log.h"
#include "i2c.h"
#include "mpu9250.h"
#include "madgwick.h"

#define DEG2RAD     3.14f/180.0f

const char *TAG = "APP_MAIN";

mpu9250_handle_t mpu9250_handle;
madgwick_handle_t madgwick_handle;

imu_scale_data_t gyro_data, accel_data;
imu_quat_data_t quat_data;


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
    mpu9250_auto_calib(mpu9250_handle);

    madgwick_config_t madgwick_cfg;
    madgwick_cfg.beta = 0.1f;
    madgwick_cfg.sample_freq = 100;
    madgwick_handle = madgwick_config(&madgwick_cfg);

    while (1)
    {
        mpu9250_get_accel_scale(mpu9250_handle, &accel_data);
        mpu9250_get_gyro_scale(mpu9250_handle, &gyro_data);
        madgwick_update_6dof(madgwick_handle,
                             gyro_data.x_axis * DEG2RAD,
                             gyro_data.y_axis * DEG2RAD,
                             gyro_data.z_axis * DEG2RAD,
                             accel_data.x_axis,
                             accel_data.y_axis,
                             accel_data.z_axis);

        madgwick_get_quaternion(madgwick_handle, &quat_data);
        float roll = 180.0 / 3.14 * atan2(2 * (quat_data.q0 * quat_data.q1 + quat_data.q2 * quat_data.q3), 1 - 2 * (quat_data.q1 * quat_data.q1 + quat_data.q2 * quat_data.q2));
        float pitch = 180.0 / 3.14 * asin(2 * (quat_data.q0 * quat_data.q2 - quat_data.q3 * quat_data.q1));
        float yaw = 180.0 / 3.14 * atan2f(quat_data.q0 * quat_data.q3 + quat_data.q1 * quat_data.q2, 0.5f - quat_data.q2 * quat_data.q2 - quat_data.q3 * quat_data.q3);

        STM_LOGI(TAG, "roll: %7.4f\t\tpitch: %7.4f\t\tyaw: %7.4f\t", roll, pitch, yaw);
        STM_LOGI(TAG, "***************************************");
        vTaskDelay(100 / portTICK_PERIOD_MS);
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