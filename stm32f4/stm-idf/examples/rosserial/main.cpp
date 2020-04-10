#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stm_log.h"
#include "gpio.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

#define TASK_SIZE   1024
#define TASK_PRIOR  5

void led_cb( const std_msgs::UInt16& cmd_msg)
{
    gpio_toggle_level(GPIO_PORT_D, GPIO_NUM_12);
} 

ros::NodeHandle nh; 
std_msgs::String str_msg; 
ros::Publisher chatter("chatter", &str_msg); 
ros::Subscriber<std_msgs::UInt16> sub("led", led_cb);

const char * hello = "Hello World !"; 


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
    nh.initNode(); 
    nh.advertise(chatter); 
    nh.subscribe(sub);

    gpio_config_t gpio_cfg;
    gpio_cfg.port = GPIO_PORT_D;
    gpio_cfg.num = GPIO_NUM_12;
    gpio_cfg.mode = GPIO_OUTPUT;
    gpio_cfg.pull_mode = GPIO_REG_PULL_NONE;
    gpio_config(&gpio_cfg);

    while (1)
    {
        if (nh.connected())
        {
            str_msg.data = hello;
            chatter.publish(&str_msg);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        nh.spinOnce();
    }
}

int main(void)
{
    HAL_Init();
    system_clock_init();

    xTaskCreate(example_task, "example_task", TASK_SIZE, NULL, TASK_PRIOR, NULL);
    vTaskStartScheduler();
}