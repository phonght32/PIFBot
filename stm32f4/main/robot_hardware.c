#include "robot_hardware.h"

#define MOTOR_INIT_ERR_STR              "motor init error"
#define IMU_INIT_ERR_STR                "imu init error"
#define RESOLVER_INIT_ERR_STR           "resolver init error"
#define MADGWICK_INIT_ERR_STR           "madgwick filter init error"

#define IMU_UPDATE_QUAT_ERR_STR         "imu update quaternion error"
#define IMU_GET_QUAT_ERR_STR            "imu get quaternion error"
#define IMU_GET_ACCEL_ERR_STR           "imu get accelerometer error"
#define IMU_GET_GYRO_ERR_STR            "imu get gyroscope error"

#define MOTORLEFT_START_ERR_STR         "motor left start error"
#define MOTORLEFT_STOP_ERR_STR          "motor left stop error"
#define MOTORLEFT_FORWARD_ERR_STR       "motor left forward error"
#define MOTORLEFT_BACKWARD_ERR_STR      "motor left backward error"
#define MOTORLEFT_SET_SPEED_ERR_STR     "motor left set speed error"
#define MOTORLEFT_GET_DIR_ERR_STR       "motor left get direction error"

#define MOTORRIGHT_START_ERR_STR        "motor right start error"
#define MOTORRIGHT_STOP_ERR_STR         "motor right stop error"
#define MOTORRIGHT_FORWARD_ERR_STR      "motor right forward error"
#define MOTORRIGHT_BACKWARD_ERR_STR     "motor right backward error"
#define MOTORRIGHT_SET_SPEED_ERR_STR    "motor right set speed error"
#define MOTORRIGHT_GET_DIR_ERR_STR      "motor right get direction error"

static const char* TAG = "ROBOT HARDWARE";
#define HARDWARE_CHECK(a, str, ret)  if(!(a)) {                                      \
        STM_LOGE(TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);        \
        return (ret);                                                                \
        }

stepmotor_handle_t motor_left, motor_right;
software_resolver_handle_t resolver_left, resolver_right;
mpu9250_handle_t mpu9250_handle;
madgwick_handle_t madgwick_handle;

stm_err_t robot_motor_init(void) 
{
    stepmotor_config_t motorleft_cfg;
    motorleft_cfg.dir_gpio_port = MOTORLEFT_DIR_GPIO_PORT;
    motorleft_cfg.dir_gpio_num = MOTORLEFT_DIR_GPIO_NUM;
    motorleft_cfg.pulse_timer_num = MOTORLEFT_PULSE_TIMER_NUM;
    motorleft_cfg.pulse_timer_pins_pack = MOTORLEFT_PULSE_TIMER_PINSPACK;
    motorleft_cfg.pulse_timer_chnl = MOTORLEFT_PULSE_TIMER_CHANNEL;
    motor_left = stepmotor_config(&motorleft_cfg);
    HARDWARE_CHECK(motor_left, MOTOR_INIT_ERR_STR, STM_FAIL);

    stepmotor_config_t motorright_cfg;
    motorright_cfg.dir_gpio_port = MOTORRIGHT_DIR_GPIO_PORT;
    motorright_cfg.dir_gpio_num = MOTORRIGHT_DIR_GPIO_NUM;
    motorright_cfg.pulse_timer_num = MOTORRIGHT_PULSE_TIMER_NUM;
    motorright_cfg.pulse_timer_pins_pack = MOTORRIGHT_PULSE_TIMER_PINSPACK;
    motorright_cfg.pulse_timer_chnl = MOTORRIGHT_PULSE_TIMER_CHANNEL;
    motor_right = stepmotor_config(&motorright_cfg);
    HARDWARE_CHECK(motor_right, MOTOR_INIT_ERR_STR, STM_FAIL);

    int ret;

    ret = stepmotor_set_pwm_duty(motor_left, STEP_DRIVER_PWM_DUTYCYCLE);
    HARDWARE_CHECK(!ret, MOTOR_INIT_ERR_STR, STM_FAIL);
    ret = stepmotor_set_pwm_freq(motor_left, 0);
    HARDWARE_CHECK(!ret, MOTOR_INIT_ERR_STR, STM_FAIL);
    ret = stepmotor_set_dir(motor_left, MOTORLEFT_DIR_FORWARD);
    HARDWARE_CHECK(!ret, MOTOR_INIT_ERR_STR, STM_FAIL);
    ret = stepmotor_start(motor_left);
    HARDWARE_CHECK(!ret, MOTOR_INIT_ERR_STR, STM_FAIL);

    ret = stepmotor_set_pwm_duty(motor_right, STEP_DRIVER_PWM_DUTYCYCLE);
    HARDWARE_CHECK(!ret, MOTOR_INIT_ERR_STR, STM_FAIL);
    ret = stepmotor_set_pwm_freq(motor_right, 0);
    HARDWARE_CHECK(!ret, MOTOR_INIT_ERR_STR, STM_FAIL);
    ret = stepmotor_set_dir(motor_right, MOTORRIGHT_DIR_FORWARD);
    HARDWARE_CHECK(!ret, MOTOR_INIT_ERR_STR, STM_FAIL);
    ret = stepmotor_start(motor_right);
    HARDWARE_CHECK(!ret, MOTOR_INIT_ERR_STR, STM_FAIL);

    STM_LOGD(TAG, "Configure motor success.");
    return STM_OK;
}

stm_err_t robot_imu_init(void)
{
    int ret;

    i2c_config_t i2c_cfg;
    i2c_cfg.i2c_num = IMU_I2C_NUM;
    i2c_cfg.i2c_pins_pack = IMU_I2C_PINSPACK;
    i2c_cfg.clk_speed = IMU_CLOCK_SPEED;
    ret = i2c_config(&i2c_cfg);
    HARDWARE_CHECK(!ret, IMU_INIT_ERR_STR, STM_FAIL);
        
    mpu9250_config_t mpu9250_cfg;
    mpu9250_cfg.afs_sel = MPU9250_AFS_SEL_8G;
    mpu9250_cfg.clksel = MPU9250_CLKSEL_AUTO;
    mpu9250_cfg.dlpf_cfg =  MPU9250_41ACEL_42GYRO_BW_HZ;
    mpu9250_cfg.fs_sel = MPU9250_FS_SEL_1000;
    mpu9250_cfg.sleep_mode = MPU9250_DISABLE_SLEEP_MODE;
    mpu9250_cfg.i2c_num = IMU_I2C_NUM;
    mpu9250_handle = mpu9250_config(&mpu9250_cfg);
    HARDWARE_CHECK(mpu9250_handle, IMU_INIT_ERR_STR, STM_FAIL);
    STM_LOGD(TAG, "Configure IMU success.");

    mpu9250_auto_calib(mpu9250_handle);
    STM_LOGD(TAG, "Calibrate IMU success");

    imu_bias_data_t accel_bias, gyro_bias;
    mpu9250_get_accel_bias(mpu9250_handle, &accel_bias);
    mpu9250_get_gyro_bias(mpu9250_handle, &gyro_bias);
    STM_LOGD(TAG, "MPU9250 bias value:");
    STM_LOGD(TAG, "x accel: %d\t y accel %d\t z accel %d", accel_bias.x_axis, accel_bias.y_axis, accel_bias.z_axis);
    STM_LOGD(TAG, "x gyro: %d\t y gyro %d\t z gyro %d", gyro_bias.x_axis, gyro_bias.y_axis, gyro_bias.z_axis);

    return STM_OK;
}

stm_err_t robot_madgwick_filter_init(void)
{
    madgwick_config_t madgwick_cfg;
    madgwick_cfg.beta = MADGWICK_BETA;
    madgwick_cfg.sample_freq = MADGWICK_SAMPLE_FREQ;
    madgwick_handle = madgwick_config(&madgwick_cfg);
    HARDWARE_CHECK(madgwick_handle, MADGWICK_INIT_ERR_STR, STM_FAIL);

    STM_LOGD(TAG, "Configure Madgwick filter success");
    return STM_OK;
}

stm_err_t robot_encoder_init(void)
{
    software_resolver_config_t resolver_left_cfg;
    resolver_left_cfg.timer_num = MOTORLEFT_TICK_TIMER_NUM;
    resolver_left_cfg.timer_pins_pack = MOTORLEFT_TICK_TIMER_PINSPACK;
    resolver_left_cfg.max_reload = 60000;
    resolver_left_cfg.counter_mode = TIMER_COUNTER_UP;
    resolver_left = software_resolver_config(&resolver_left_cfg);

    software_resolver_config_t resolver_right_cfg;
    resolver_right_cfg.timer_num = MOTORRIGHT_TICK_TIMER_NUM;
    resolver_right_cfg.timer_pins_pack = MOTORRIGHT_TICK_TIMER_PINSPACK;
    resolver_right_cfg.max_reload = 60000;
    resolver_right_cfg.counter_mode = TIMER_COUNTER_UP;
    resolver_right = software_resolver_config(&resolver_right_cfg);

    int ret;

    ret = software_resolver_start(resolver_left);
    HARDWARE_CHECK(!ret, RESOLVER_INIT_ERR_STR, STM_FAIL);
    ret = software_resolver_start(resolver_right);
    HARDWARE_CHECK(!ret, RESOLVER_INIT_ERR_STR, STM_FAIL);

    STM_LOGD(TAG, "Configure resolver success");
    return STM_OK;
}

stm_err_t robot_motor_left_start(void)
{
    int ret = stepmotor_start(motor_left);
    HARDWARE_CHECK(!ret, MOTORLEFT_START_ERR_STR, STM_FAIL);

    return STM_OK;
}

stm_err_t robot_motor_left_stop(void)
{
    int ret = stepmotor_stop(motor_left);
    HARDWARE_CHECK(!ret, MOTORLEFT_STOP_ERR_STR, STM_FAIL);

    return STM_OK;
}

stm_err_t robot_motor_left_forward(void)
{
    int ret = stepmotor_set_dir(motor_left, MOTORLEFT_DIR_FORWARD);
    HARDWARE_CHECK(!ret, MOTORLEFT_FORWARD_ERR_STR, STM_FAIL);
    software_resolver_set_mode(resolver_left, TIMER_COUNTER_UP);

    return STM_OK;
}

stm_err_t robot_motor_left_backward(void)
{
    int ret = stepmotor_set_dir(motor_left, MOTORLEFT_DIR_BACKWARD);
    HARDWARE_CHECK(!ret, MOTORLEFT_BACKWARD_ERR_STR, STM_FAIL);
    software_resolver_set_mode(resolver_left, TIMER_COUNTER_DOWN);

    return STM_OK;
}

stm_err_t robot_motor_left_set_speed(float speed)
{
    int ret = stepmotor_set_pwm_freq(motor_left, (uint32_t)(speed * VEL2FREQ));
    HARDWARE_CHECK(!ret, MOTORLEFT_SET_SPEED_ERR_STR, STM_FAIL);

    return STM_OK;
}

stm_err_t robot_motor_right_start(void)
{
    int ret = stepmotor_start(motor_right);
    HARDWARE_CHECK(!ret, MOTORRIGHT_START_ERR_STR, STM_FAIL);

    return STM_OK;
}

stm_err_t robot_motor_right_stop(void)
{
    int ret = stepmotor_stop(motor_right);
    HARDWARE_CHECK(!ret, MOTORRIGHT_STOP_ERR_STR, STM_FAIL);

    return STM_OK;
}

stm_err_t robot_motor_right_forward(void)
{
    int ret = stepmotor_set_dir(motor_right, MOTORRIGHT_DIR_FORWARD);
    HARDWARE_CHECK(!ret, MOTORRIGHT_FORWARD_ERR_STR, STM_FAIL);
    software_resolver_set_mode(resolver_right, TIMER_COUNTER_UP);

    return STM_OK;
}

stm_err_t robot_motor_right_backward(void)
{
    int ret = stepmotor_set_dir(motor_right, MOTORRIGHT_DIR_BACKWARD);
    HARDWARE_CHECK(!ret, MOTORRIGHT_BACKWARD_ERR_STR, STM_FAIL);
    software_resolver_set_mode(resolver_right, TIMER_COUNTER_DOWN);

    return STM_OK;
}

stm_err_t robot_motor_right_set_speed(float speed)
{
    int ret = stepmotor_set_pwm_freq(motor_right, (uint32_t)(speed * VEL2FREQ));
    HARDWARE_CHECK(!ret, MOTORRIGHT_SET_SPEED_ERR_STR, STM_FAIL);

    return STM_OK;
}

stm_err_t robot_imu_update_quat(void)
{
    int ret;
    imu_scale_data_t accel_scale, gyro_scale;

    ret = mpu9250_get_accel_scale(mpu9250_handle, &accel_scale);
    HARDWARE_CHECK(!ret, IMU_UPDATE_QUAT_ERR_STR, STM_FAIL);

    ret = mpu9250_get_gyro_scale(mpu9250_handle, &gyro_scale);
    HARDWARE_CHECK(!ret, IMU_UPDATE_QUAT_ERR_STR, STM_FAIL);

    madgwick_update_6dof(madgwick_handle,
                         DEG2RAD(gyro_scale.x_axis),
                         DEG2RAD(gyro_scale.y_axis),
                         DEG2RAD(gyro_scale.z_axis),
                         accel_scale.x_axis,
                         accel_scale.y_axis,
                         accel_scale.z_axis);

    return STM_OK;
}

stm_err_t robot_imu_get_quat(float *quat)
{
    imu_quat_data_t quat_data;
    madgwick_get_quaternion(madgwick_handle, &quat_data);

    quat[0] = quat_data.q0;
    quat[1] = quat_data.q1;
    quat[2] = quat_data.q2;
    quat[3] = quat_data.q3;

    return STM_OK;
}

stm_err_t robot_imu_get_accel(float *accel)
{
    imu_scale_data_t accel_data;
    int ret = mpu9250_get_accel_scale(mpu9250_handle, &accel_data);
    HARDWARE_CHECK(!ret, IMU_GET_ACCEL_ERR_STR, STM_FAIL);

    accel[0] = accel_data.x_axis;
    accel[1] = accel_data.y_axis;
    accel[2] = accel_data.z_axis;

    return STM_OK;
}

stm_err_t robot_imu_get_gyro(float *gyro)
{
    imu_scale_data_t gyro_data;
    int ret = mpu9250_get_gyro_scale(mpu9250_handle, &gyro_data);
    HARDWARE_CHECK(!ret, IMU_GET_GYRO_ERR_STR, STM_FAIL);

    gyro[0] = gyro_data.x_axis;
    gyro[1] = gyro_data.y_axis;
    gyro[2] = gyro_data.z_axis;

    return STM_OK;
}

stm_err_t robot_encoder_left_get_tick(uint32_t *left_tick)
{
    software_resolver_get_value(resolver_left, left_tick);
    return STM_OK;
}

stm_err_t robot_encoder_right_get_tick(uint32_t *right_tick)
{
    software_resolver_get_value(resolver_right, right_tick);
    return STM_OK;
}













