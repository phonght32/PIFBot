#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern UART_HandleTypeDef huart4;

void NMI_Handler(void)
{

}

void HardFault_Handler(void)
{
    while (1)
    {

    }
}

void MemManage_Handler(void)
{
    while (1)
    {

    }
}

void BusFault_Handler(void)
{
    while (1)
    {

    }
}

void UsageFault_Handler(void)
{
    while (1)
    {

    }
}

void DebugMon_Handler(void)
{

}

void SysTick_Handler(void)
{
    HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
#endif /* INCLUDE_xTaskGetSchedulerState */
        xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
    }
#endif /* INCLUDE_xTaskGetSchedulerState */
}


