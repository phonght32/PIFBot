#include "stm32f4xx_hal.h"

#include "string.h"
#include "stdio.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

#include "../components/driver/include/uart.h"
#include "../components/driver/include/gpio.h"
#include "../components/driver/include/timer.h"
#include "../components/driver/include/i2c.h"
#include "../components/mpu6050/include/mpu6050.h"

void led_cb( const std_msgs::UInt16& cmd_msg)
{
}

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Subscriber<std_msgs::UInt16> sub("led", led_cb);

mpu6050_raw_data_t accel_raw, gyro_raw;
int16_t accel[3];
char data[50];


I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

void SystemClock_Config(void);
static void MX_DMA_Init(void);


int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_DMA_Init();

    uart_config_t uart_config;
    uart_config.uart_num = UART_NUM_4;
    uart_config.baudrate = 57600;
    uart_config.uart_pins_pack = UART_PINS_PACK_1;
    uart_handle_t handle = uart_init(&uart_config);
    huart4 = uart_get_UART_HandleTypeDef(handle);

    uart_config_t uart_debug;
    uart_debug.uart_num = UART_NUM_2;
    uart_debug.baudrate = 115200;
    uart_debug.uart_pins_pack = UART_PINS_PACK_1;
    uart_handle_t uart_debug_handle = uart_init(&uart_debug);

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

    nh.initNode();
    nh.advertise(chatter);
    nh.subscribe(sub);
    const char * hello = "Hello World!!";
    int chatter_interval = 1000.0 / 2;
    int chatter_last = HAL_GetTick();


    while (1)
    {
    	mpu6050_get_accel_raw(&accel_raw);
    	accel[0] = accel_raw.x_axis;
    	accel[1] = accel_raw.y_axis;
    	accel[2] = accel_raw.z_axis;
    	sprintf((char*)data,(char*)"accel_y: %d \r\n", accel[1]);
    	uart_write_bytes(uart_debug_handle,(uint8_t*)data,20,100);

    	HAL_Delay(500);
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
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}




static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}


#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */


