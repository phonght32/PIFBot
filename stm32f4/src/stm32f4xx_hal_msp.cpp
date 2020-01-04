#include "main.h"

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}


void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
    if (huart->Instance == UART4)
    {
        __HAL_RCC_UART4_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);

        HAL_DMA_DeInit(huart->hdmarx);
        HAL_DMA_DeInit(huart->hdmatx);

        HAL_NVIC_DisableIRQ(UART4_IRQn);
    }
}

