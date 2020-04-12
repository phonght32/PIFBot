#include "robot_utils.h"


uint32_t millis(void)
{
    return xTaskGetTickCount();
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

