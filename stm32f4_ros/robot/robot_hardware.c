#include "include/robot_hardware.h"
#include "../stm32f4_library/driver/include/i2c.h"

/*
 *  Convert from velocity (m/s) to frequency (Hz) for motor driver.
 *
 *                      2*pi*WHELL_RADIUS
 *  velocity (m/s) =  ------------------------------
 *                    NUM_PULSE_PER_ROUND * STEP_DIV
 *
 */
#define VEL2FREQ    	((NUM_PULSE_PER_ROUND*MICROSTEP_DIV)/(2*PI*WHEEL_RADIUS))


/*
 * Motor Handle Structure.
 */
step_motor_handle_t motor_left, motor_right;

/*
 * Encoder Handle Structure.
 */
encoder_handle_t enc_left, enc_right;

/*
 * IMU Handle Structure.
 */
mpu6050_handle_t mpu6050;
I2C_HandleTypeDef mpu6050_i2c;

/*
 * UART Rosserial Handle Structure.
 */
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;


void robot_motor_init(void)
{
	/* Configure motor left driver */
    step_motor_config_t motor_left_config;
    motor_left_config.pin_clk.timer_num 		= MOTORLEFT_PULSE_TIMER_NUM;
    motor_left_config.pin_clk.timer_channel 	= MOTORLEFT_PULSE_TIMER_CHANNEL;
    motor_left_config.pin_clk.timer_pins_pack 	= MOTORLEFT_PULSE_TIMER_PINSPACK;
    motor_left_config.pin_dir.gpio_port 		= MOTORLEFT_DIR_GPIO_PORT;
    motor_left_config.pin_dir.gpio_num 			= MOTORLEFT_DIR_GPIO_NUM;
    motor_left_config.pin_dir.gpio_mode			= GPIO_OUTPUT;
    motor_left_config.pin_dir.gpio_reg_pull 	= GPIO_REG_PULL_NONE;
    motor_left_config.num_pulse_per_round 		= NUM_PULSE_PER_ROUND;
    motor_left_config.microstep_div 			= MICROSTEP_DIV_4;
    motor_left = step_motor_init(&motor_left_config);

    /* Configure motor right driver */
    step_motor_config_t motor_right_config;
    motor_right_config.pin_clk.timer_num 		= MOTORRIGHT_PULSE_TIMER_NUM;
    motor_right_config.pin_clk.timer_channel 	= MOTORRIGHT_PULSE_TIMER_CHANNEL;
    motor_right_config.pin_clk.timer_pins_pack 	= MOTORRIGHT_PULSE_TIMER_PINSPACK;
    motor_right_config.pin_dir.gpio_port 		= MOTORRIGHT_DIR_GPIO_PORT;
    motor_right_config.pin_dir.gpio_num 		= MOTORRIGHT_DIR_GPIO_NUM;
    motor_right_config.pin_dir.gpio_mode 		= GPIO_OUTPUT;
    motor_right_config.pin_dir.gpio_reg_pull 	= GPIO_REG_PULL_NONE;
    motor_right_config.num_pulse_per_round 		= NUM_PULSE_PER_ROUND;
    motor_right_config.microstep_div 			= MICROSTEP_DIV_4;
    motor_right = step_motor_init(&motor_right_config);
}

void robot_imu_init(void)
{
	/* Configure I2C */
    i2c_config_t i2c_config;
    i2c_config.i2c_num = MPU6050_I2C_NUM;
    i2c_config.i2c_pins_pack = MPU6050_I2C_PINSPACK;
    i2c_handle_t i2c_handle = i2c_init(&i2c_config);

    /* Configure I2C for IMU */
    mpu6050_i2c = i2c_get_I2C_HandleTypeDef(i2c_handle);
    mpu6050_i2c_config(&mpu6050_i2c);

    /* Configure IMU parameters */
    mpu6050_config_t mpu6050_config;
    mpu6050_config.afs_sel = MPU6050_AFS_SEL_8G;
    mpu6050_config.clksel = MPU6050_CLKSEL_Z_GYRO_REF;
//    mpu6050_config.dlpf_cfg =  MPU6050_184ACCEL_188GYRO_BW_HZ;
    mpu6050_config.fs_sel = MPU6050_FS_SEL_1000;
    mpu6050_config.sleep_mode = MPU6050_DISABLE_SLEEP_MODE;
    mpu6050 = mpu6050_init(&mpu6050_config);

    /* Auto calibrate IMU bias value */
    mpu6050_auto_calib(mpu6050);
}


void robot_rosserial_init(void)
{
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart4.Instance = UART4;
    huart4.Init.BaudRate = ROSSERIAL_BAUDRATE;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart4);

    hdma_uart4_rx.Instance = DMA1_Stream2;
    hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_NORMAL;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_uart4_rx);
    __HAL_LINKDMA(&huart4, hdmarx, hdma_uart4_rx);

    hdma_uart4_tx.Instance = DMA1_Stream4;
    hdma_uart4_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_tx.Init.Mode = DMA_NORMAL;
    hdma_uart4_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart4_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_uart4_tx);
    __HAL_LINKDMA(&huart4, hdmatx, hdma_uart4_tx);

    HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);

    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

void robot_encoder_init(void)
{
	/* Initialize left encoder */
	encoder_config_t enc_left_config;
	enc_left_config.timer_num = MOTORLEFT_TICK_TIMER_NUM;
	enc_left_config.timer_pins_pack = MOTORLEFT_TICK_TIMER_PINSPACK;
	enc_left_config.max_reload = 60000;
	enc_left_config.counter_mode = TIMER_COUNTER_UP;
	enc_left = encoder_init(&enc_left_config);

	/* Initialize right encoder */
	encoder_config_t enc_right_config;
	enc_right_config.timer_num = MOTORRIGHT_TICK_TIMER_NUM;
	enc_right_config.timer_pins_pack = MOTORRIGHT_TICK_TIMER_PINSPACK;
	enc_right_config.max_reload = 60000;
	enc_right_config.counter_mode = TIMER_COUNTER_UP;
	enc_right = encoder_init(&enc_right_config);

	encoder_start(enc_left);
	encoder_start(enc_right);
}

void robot_motor_left_forward(void)
{
    step_motor_set_dir(motor_left, MOTORLEFT_FORWARD);
    encoder_set_mode(enc_left, TIMER_COUNTER_UP);
}

void robot_motor_left_backward(void)
{
    step_motor_set_dir(motor_left, MOTORLEFT_BACKWARD);
    encoder_set_mode(enc_left, TIMER_COUNTER_DOWN);
}

void robot_motor_right_forward(void)
{
    step_motor_set_dir(motor_right, MOTORRIGHT_FORWARD);
    encoder_set_mode(enc_right, TIMER_COUNTER_UP);
}

void robot_motor_right_backward(void)
{
    step_motor_set_dir(motor_right, MOTORRIGHT_BACKWARD);
    encoder_set_mode(enc_right, TIMER_COUNTER_DOWN);
}

void robot_motor_left_set_speed(float speed)
{
    step_motor_set_freq(motor_left, (uint32_t)(speed * VEL2FREQ));
}

void robot_motor_right_set_speed(float speed)
{
    step_motor_set_freq(motor_right, (uint32_t)(speed * VEL2FREQ));
}

uint32_t robot_encoder_left_get_tick(void)
{
	uint32_t resolver_tick;
	resolver_tick =  encoder_get_value(enc_left);
	return resolver_tick;
}

uint32_t robot_encoder_right_get_tick(void)
{
	uint32_t resolver_tick;
	resolver_tick =  encoder_get_value(enc_right);
	return resolver_tick;
}

void robot_motor_left_start(void)
{
	step_motor_start(motor_left);
}

void robot_motor_left_stop(void)
{
	step_motor_stop(motor_left);
}

void robot_motor_right_start(void)
{
	step_motor_start(motor_right);
}

void robot_motor_right_stop(void)
{
	step_motor_stop(motor_right);
}

void robot_encoder_left_reset(void)
{
	ext_counter_set_value(enc_left, 0);
}

void robot_encoder_right_reset(void)
{
	ext_counter_set_value(enc_right, 0);
}

bool robot_motor_left_get_dir(void)
{
	return step_motor_get_dir(motor_left);
}

bool robot_motor_right_get_dir(void)
{
	return step_motor_get_dir(motor_right);
}

void robot_imu_update_quat(void)
{
	mpu6050_update_quat();
}

void robot_imu_get_quat(float *quat)
{
	mpu6050_quat_data_t quat_data;
	mpu6050_get_quat(&quat_data);
	quat[0] = quat_data.q0;
	quat[1] = quat_data.q1;
	quat[2] = quat_data.q2;
	quat[3] = quat_data.q3;
}

void robot_imu_get_accel(float *accel)
{
	mpu6050_scale_data_t accel_data;
	mpu6050_get_accel_scale(&accel_data);
	accel[0] = accel_data.x_axis;
	accel[1] = accel_data.y_axis;
	accel[2] = accel_data.z_axis;
}

void robot_imu_get_gyro(float *gyro)
{
	mpu6050_scale_data_t gyro_data;
	mpu6050_get_gyro_scale(&gyro_data);
	gyro[0] = gyro_data.x_axis;
	gyro[1] = gyro_data.y_axis;
	gyro[2] = gyro_data.z_axis;
}
