#include "main.h"
#include "stm32f4xx_it.h"

extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern UART_HandleTypeDef huart4;

extern TIM_HandleTypeDef htim5;

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


void SVC_Handler(void)
{

}


void DebugMon_Handler(void)
{

}

void PendSV_Handler(void)
{

}


void SysTick_Handler(void)
{
    HAL_IncTick();
}

void DMA1_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_uart4_rx);
}

void DMA1_Stream4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_uart4_tx);
}

void UART4_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart4);
}

void TIM5_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim5);
}
