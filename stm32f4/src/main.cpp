#include "main.h"

#include "stdio.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

#include "../components/driver/include/timer.h"
#include "../components/driver/include/i2c.h"
#include "../components/driver/include/gpio.h"
#include "../components/driver/include/uart.h"
#include "../components/mpu6050/include/mpu6050.h"
#include "../components/step_driver/include/step_driver.h"

//#define TEST_ROSSERIAL
//#define TEST_PWM
//#define TEST_MPU6050
#define TEST_STEP_DRIVER

void led_cb( const std_msgs::UInt16& cmd_msg)
{
}
ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Subscriber<std_msgs::UInt16> sub("led", led_cb);

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;


I2C_HandleTypeDef hi2c1;
mpu6050_raw_data_t accel_raw, gyro_raw;
char mpu6050_uart_buf[50];



void SystemClock_Config(void);
static void rosserial_init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

#ifdef TEST_ROSSERIAL
    /* ROS configuration */
    rosserial_init();
    nh.initNode();
    nh.advertise(chatter);
    nh.subscribe(sub);
    const char * hello = "Hello World!!";
    int chatter_interval = 1000.0 / 2;
    int chatter_last = HAL_GetTick();
#endif

#ifdef TEST_PWM
    /* PWM configuration */
    pwm_config_t pwm1_config;
    pwm1_config.timer_num = TIMER_NUM_4;
    pwm1_config.timer_channel = TIMER_CHANNEL_1;
    pwm1_config.timer_pins_pack = TIMER_PINS_PACK_2;
    pwm_handle_t pwm1_handle = pwm_init(&pwm1_config);

    pwm_set_freq(pwm1_handle, 1);
    pwm_set_duty(pwm1_handle, 50);

    pwm_config_t pwm2_config;
    pwm2_config.timer_num = TIMER_NUM_4;
    pwm2_config.timer_channel = TIMER_CHANNEL_2;
    pwm2_config.timer_pins_pack = TIMER_PINS_PACK_2;
    pwm_handle_t pwm2_handle = pwm_init(&pwm2_config);

    pwm_set_freq(pwm2_handle, 1);
    pwm_set_duty(pwm2_handle, 50);
#endif

#ifdef TEST_MPU6050
    /* MPU6050 configuration */
    i2c_config_t i2c_config;
    i2c_config.i2c_num = I2C_NUM_1;
    i2c_config.i2c_pins_pack = I2C_PINS_PACK_1;
    i2c_handle_t i2c_handle = i2c_init(&i2c_config);
    hi2c1 = i2c_get_I2C_HandleTypeDef(i2c_handle);

    mpu6050_i2c_config(&hi2c1);
    mpu6050_config_t mpu6050_config;
    mpu6050_config.afs_sel = MPU6050_AFS_SEL_4G;
    mpu6050_config.clksel = MPU6050_CLKSEL_INTERNAL_8_MHZ;
    mpu6050_config.dlpf_cfg =  MPU6050_184ACCEL_188GYRO_BW_HZ;
    mpu6050_config.fs_sel = MPU6050_FS_SEL_500;
    mpu6050_config.sleep_mode = MPU6050_DISABLE_SLEEP_MODE;
    mpu6050_init(&mpu6050_config);

    uart_config_t uart_debug;
    uart_debug.uart_num = UART_NUM_2;
    uart_debug.baudrate = 115200;
    uart_debug.uart_pins_pack = UART_PINS_PACK_1;
    uart_handle_t uart_debug_handle = uart_init(&uart_debug);
#endif

#ifdef TEST_STEP_DRIVER
    /* Step driver configuration */
    step_driver_config_t motor1_config;
    motor1_config.pin_clk.timer_num = TIMER_NUM_4;
    motor1_config.pin_clk.timer_pins_pack = TIMER_PINS_PACK_2;
    motor1_config.pin_clk.timer_channel = TIMER_CHANNEL_1;
    motor1_config.pin_dir.gpio_port = GPIO_PORT_D;
    motor1_config.pin_dir.gpio_num = GPIO_NUM_13;
    motor1_config.pin_dir.gpio_mode = GPIO_OUTPUT;
    motor1_config.pin_dir.gpio_reg_pull = GPIO_REG_PULL_NONE;
    step_driver_handle_t motor1_handle = step_driver_init(&motor1_config);

    step_driver_config_t motor2_config;
    motor2_config.pin_clk.timer_num = TIMER_NUM_4;
    motor2_config.pin_clk.timer_pins_pack = TIMER_PINS_PACK_2;
    motor2_config.pin_clk.timer_channel = TIMER_CHANNEL_3;
    motor2_config.pin_dir.gpio_port = GPIO_PORT_D;
    motor2_config.pin_dir.gpio_num = GPIO_NUM_15;
    motor2_config.pin_dir.gpio_mode = GPIO_OUTPUT;
    motor2_config.pin_dir.gpio_reg_pull = GPIO_REG_PULL_NONE;
    step_driver_handle_t motor2_handle = step_driver_init(&motor2_config);
#endif

    while (1)
    {

#ifdef TEST_STEP_DRIVER
    	step_driver_set_freq(motor1_handle, 1);
    	step_driver_set_dir(motor1_handle, 1);
    	step_driver_start(motor1_handle);

    	step_driver_set_freq(motor2_handle, 1);
    	step_driver_set_dir(motor2_handle, 1);
    	step_driver_start(motor2_handle);
#endif

#ifdef TEST_MPU6050 
        uint16_t num_bytes;
        mpu6050_get_accel_raw(&accel_raw);
        mpu6050_get_gyro_raw(&gyro_raw);

        num_bytes = sprintf((char*)mpu6050_uart_buf,(char*)"accel_x: %d \t\t", accel_raw.x_axis);
        uart_write_bytes(uart_debug_handle,(uint8_t*)mpu6050_uart_buf,num_bytes,100);
        num_bytes = sprintf((char*)mpu6050_uart_buf,(char*)"gyro_x: %d \r\n", gyro_raw.x_axis);
        uart_write_bytes(uart_debug_handle,(uint8_t*)mpu6050_uart_buf,num_bytes,100);

        num_bytes = sprintf((char*)mpu6050_uart_buf,(char*)"accel_y: %d \t\t", accel_raw.y_axis);
        uart_write_bytes(uart_debug_handle,(uint8_t*)mpu6050_uart_buf,num_bytes,100);
        num_bytes = sprintf((char*)mpu6050_uart_buf,(char*)"gyro_y: %d \r\n", gyro_raw.y_axis);
        uart_write_bytes(uart_debug_handle,(uint8_t*)mpu6050_uart_buf,num_bytes,100);

        num_bytes = sprintf((char*)mpu6050_uart_buf,(char*)"accel_z: %d \t\t", accel_raw.z_axis);
        uart_write_bytes(uart_debug_handle,(uint8_t*)mpu6050_uart_buf,num_bytes,100);
        num_bytes = sprintf((char*)mpu6050_uart_buf,(char*)"gyro_z: %d \r\n", gyro_raw.z_axis);
        uart_write_bytes(uart_debug_handle,(uint8_t*)mpu6050_uart_buf,num_bytes,100);

        uart_write_bytes(uart_debug_handle,(uint8_t*)"\n",1,100);

        HAL_Delay(200);
#endif

#ifdef TEST_PWM
        pwm_start(pwm1_handle);
        pwm_start(pwm2_handle);
#endif

#ifdef TEST_ROSSERIAL
        if (nh.connected())
        {
        	if (HAL_GetTick() - chatter_last > chatter_interval)
        	{
        		str_msg.data = hello;
        		chatter.publish(&str_msg);
        		chatter_last = HAL_GetTick();
        	}
        }
        nh.spinOnce();
#endif
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}



static void rosserial_init(void)
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
    huart4.Init.BaudRate = 57600;
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

void Error_Handler(void)
{

}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */

