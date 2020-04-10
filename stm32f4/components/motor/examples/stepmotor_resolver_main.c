#include "stm32f4xx_hal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "stm_log.h"
#include "stepmotor.h"
#include "software_resolver.h"

#define TASK_SIZE   1024
#define TASK_PRIOR  5

static const char *TAG = "APP_MAIN";

stepmotor_handle_t stepmotor_handle;
software_resolver_handle_t resolver_handle;

uint32_t cnt;


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


    stepmotor_config_t stepmotor_cfg;
    stepmotor_cfg.dir_gpio_port = GPIO_PORT_D;
    stepmotor_cfg.dir_gpio_num = GPIO_NUM_13;
    stepmotor_cfg.pulse_timer_num = TIMER_NUM_4;
    stepmotor_cfg.pulse_timer_pins_pack = TIMER_PINS_PACK_2;
    stepmotor_cfg.pulse_timer_channel = TIMER_CHANNEL_1;
    stepmotor_handle = stepmotor_config(&stepmotor_cfg);

    stepmotor_set_pwm_duty(stepmotor_handle, 50);
    stepmotor_set_pwm_freq(stepmotor_handle, 100);
    stepmotor_set_dir(stepmotor_handle, 0);
    stepmotor_start(stepmotor_handle);

    software_resolver_config_t resolver_cfg;
    resolver_cfg.timer_num = TIMER_NUM_2;
    resolver_cfg.timer_pins_pack = TIMER_PINS_PACK_2;
    resolver_cfg.max_reload = 800;
    resolver_cfg.counter_mode = TIMER_COUNTER_UP;
    resolver_handle = software_resolver_config(&resolver_cfg);
    software_resolver_start(resolver_handle);

    while(1)
    {
        vTaskDelay(1000/portTICK_PERIOD_MS);
        stepmotor_stop(stepmotor_handle);

        software_resolver_get_value(resolver_handle, &cnt);
        STM_LOGI(TAG, "motor tick: %d", cnt);
        
        vTaskDelay(1000/portTICK_PERIOD_MS);
        stepmotor_start(stepmotor_handle);
    }
}

int main(void)
{
    HAL_Init();
    system_clock_init();
    stm_log_init();

    stm_log_level_set("*", STM_LOG_NONE);
    stm_log_level_set("APP_MAIN", STM_LOG_INFO);
    // stm_log_level_set("STEP MOTOR", STM_LOG_INFO);
    // stm_log_level_set("SOFTWARE RESOLVER", STM_LOG_INFO);

    xTaskCreate(example_task, "example_task", TASK_SIZE, NULL, TASK_PRIOR, NULL);
    vTaskStartScheduler();
}