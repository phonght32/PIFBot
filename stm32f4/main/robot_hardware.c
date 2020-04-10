#include "robot_hardware.h"

static const char* ROBOT_HARDWARE_TAG = "ROBOT HARDWARE";
#define HARDWARE_CHECK(a, str, ret)  if(!(a)) {                                                     \
        STM_LOGE(ROBOT_HARDWARE_TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);        \
        return (ret);                                                                               \
        }

stepmotor_handle_t motor_left, motor_right;
software_resolver_handle_t resolver_left, resolver_right;
mpu9250_handle_t mpu9250_handle;

void robot_motor_init(void)
{
    stepmotor_config_t motorleft_cfg;
    motorleft_cfg.dir_gpio_port = MOTORLEFT_DIR_GPIO_PORT;
    motorleft_cfg.dir_gpio_num = MOTORLEFT_DIR_GPIO_NUM;
    motorleft_cfg.pulse_timer_num = MOTORLEFT_PULSE_TIMER_NUM;
    motorleft_cfg.pulse_timer_pins_pack = MOTORLEFT_PULSE_TIMER_PINSPACK;
    motorleft_cfg.pulse_timer_channel = MOTORLEFT_PULSE_TIMER_CHANNEL;
    motor_left = stepmotor_config(&motorleft_cfg);

    stepmotor_config_t motorright_cfg;
    motorright_cfg.dir_gpio_port = MOTORRIGHT_DIR_GPIO_PORT;
    motorright_cfg.dir_gpio_num = MOTORRIGHT_DIR_GPIO_NUM;
    motorright_cfg.pulse_timer_num = MOTORRIGHT_PULSE_TIMER_NUM;
    motorright_cfg.pulse_timer_pins_pack = MOTORRIGHT_PULSE_TIMER_PINSPACK;
    motorright_cfg.pulse_timer_channel = MOTORRIGHT_PULSE_TIMER_CHANNEL;
    motor_right = stepmotor_config(&motorright_cfg);
}