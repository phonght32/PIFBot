#include "include/robot_hardware.h"
#include "../stm32f4_library/driver/include/i2c.h"

extern step_motor_handle_t motor_left, motor_right;
extern mpu6050_handle_t mpu6050;
extern I2C_HandleTypeDef mpu6050_i2c;

void robot_motor_init(void)
{
    step_motor_config_t motor_left_config;
    motor_left_config.pin_clk.timer_num = MOTORLEFT_TIMER_NUM;
    motor_left_config.pin_clk.timer_channel = MOTORLEFT_TIMER_CHANNEL;
    motor_left_config.pin_clk.timer_pins_pack = MOTORLEFT_TIMER_PINSPACK;
    motor_left_config.pin_dir.gpio_port = MOTORLEFT_GPIO_PORT;
    motor_left_config.pin_dir.gpio_num = MOTORLEFT_GPIO_NUM;
    motor_left_config.pin_dir.gpio_mode = GPIO_OUTPUT;
    motor_left_config.pin_dir.gpio_reg_pull = GPIO_REG_PULL_NONE;
    motor_left = step_motor_init(&motor_left_config);

    step_motor_config_t motor_right_config;
    motor_right_config.pin_clk.timer_num = MOTORRIGHT_TIMER_NUM;
    motor_right_config.pin_clk.timer_channel = MOTORRIGHT_TIMER_CHANNEL;
    motor_right_config.pin_clk.timer_pins_pack = MOTORRIGHT_TIMER_PINSPACK;
    motor_right_config.pin_dir.gpio_port = MOTORRIGHT_GPIO_PORT;
    motor_right_config.pin_dir.gpio_num = MOTORRIGHT_GPIO_NUM;
    motor_right_config.pin_dir.gpio_mode = GPIO_OUTPUT;
    motor_right_config.pin_dir.gpio_reg_pull = GPIO_REG_PULL_NONE;
    motor_right = step_motor_init(&motor_right_config);
}

void robot_mpu6050_init(void)
{
    i2c_config_t i2c_config;
    i2c_config.i2c_num = MPU6050_I2C_NUM;
    i2c_config.i2c_pins_pack = MPU6050_I2C_PINSPACK;
    i2c_handle_t i2c_handle = i2c_init(&i2c_config);

    mpu6050_i2c = i2c_get_I2C_HandleTypeDef(i2c_handle);
    mpu6050_i2c_config(&mpu6050_i2c);

    mpu6050_config_t mpu6050_config;
    mpu6050_config.afs_sel = MPU6050_AFS_SEL_4G;
    mpu6050_config.clksel = MPU6050_CLKSEL_INTERNAL_8_MHZ;
    mpu6050_config.dlpf_cfg =  MPU6050_184ACCEL_188GYRO_BW_HZ;
    mpu6050_config.fs_sel = MPU6050_FS_SEL_500;
    mpu6050_config.sleep_mode = MPU6050_DISABLE_SLEEP_MODE;
    mpu6050 = mpu6050_init(&mpu6050_config);
    mpu6050_auto_calib();
}
