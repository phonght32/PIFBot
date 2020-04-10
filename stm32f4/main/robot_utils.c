#include "robot_utils.h"

TIM_HandleTypeDef htim_interval;
uint32_t tickcount_ms;

void timer_interval_init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim_interval.Instance = TIM5;
    htim_interval.Init.Prescaler = 83;
    htim_interval.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_interval.Init.Period = 999;
    htim_interval.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim_interval.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim_interval);
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim_interval, &sClockSourceConfig);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim_interval, &sMasterConfig);

    HAL_TIM_Base_Start_IT(&htim_interval);
}

uint32_t millis(void)
{
    return tickcount_ms;
}

float constrain(float x, float low_val, float high_val)
{
    float value;
    if (x > high_val)
    {
        value = high_val;
    }
    else if (x < low_val)
    {
        value = low_val;
    }
    else
    {
        value = x;
    }
    return value;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance ==  htim_interval.Instance)
    {
        tickcount_ms++;
    }
}
