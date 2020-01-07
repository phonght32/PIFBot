/******************************** Include *********************************** */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "main.h"

#include "stdio.h"
#include "math.h"
#include "string.h"

#include "../robot/include/robot_core_config.h"

#include "../components/driver/include/timer.h"
#include "../components/driver/include/i2c.h"
#include "../components/driver/include/gpio.h"
#include "../components/driver/include/uart.h"
#include "../components/mpu6050/include/mpu6050.h"
#include "../components/step_driver/include/step_driver.h"

/*************************** ***** Define *********************************** */

#define MOTOR_LEFT_FORWARD(_handle_)	step_driver_set_dir(_handle_, 1);
#define MOTOR_LEFT_BACKWARD(_handle_)	step_driver_set_dir(_handle_, 0);
#define MOTOR_RIGHT_FORWARD(_handle_)	step_driver_set_dir(_handle_, 1);
#define MOTOR_RIGHT_BACKWARD(_handle_)	step_driver_set_dir(_handle_, 0);


/********************************** ROS ************************************* */
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
void robot_rosserial_init(void);

/********************************* MOTOR ************************************ */
step_driver_handle_t motor_left, motor_right;
void robot_motor_init(void);

/******************************** MPU6050 *********************************** */
mpu6050_handle_t mpu6050;
I2C_HandleTypeDef mpu6050_i2c;
void robot_mpu6050_init(void);


/****************************** STM32 System ******************************** */
TIM_HandleTypeDef htim5;
void timer_interval_init(void);

void SystemClock_Config(void);
void Error_Handler(void);

sensor_msgs::Imu getIMU(void);
void ros_setup(void);
void initIMU(void);
void calibrationGyro(void);
void getOrientation(float *orientation);
int constrain(int x, int a, int b);

uint32_t tickcount_ms;
uint32_t millis(void);
void led_cb( const std_msgs::UInt16& cmd_msg)
{

}
ros::Subscriber<std_msgs::UInt16> sub("led", led_cb);

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);


int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* ROS configuration */
    robot_rosserial_init();

    /* Motor configuration */
    robot_motor_init();

    /* MPU6050 configuration */
    robot_mpu6050_init();

    timer_interval_init();
    HAL_TIM_Base_Start_IT(&htim5);

//        ros_setup();

    // OK
    nh.initNode();
    nh.subscribe(sub);
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(reset_sub);
    nh.advertise(chatter);
    const char * hello = "Hello Phong!!";
    int chatter_interval = 1000.0 / 2;
    int chatter_last = HAL_GetTick();
//    nh.advertise(imu_pub);

//    	nh.advertise(imu_pub);
//    	nh.advertise(cmd_vel_motor_pub);
//    	nh.advertise(odom_pub);
//    	nh.advertise(joint_states_pub);
//    	nh.advertise(battery_state_pub);

    while (1)
    {
  	  if (nh.connected())
  	  {
  		  if(HAL_GetTick() - chatter_last > chatter_interval)
  		  {
  			  str_msg.data = hello;
  			  chatter.publish(&str_msg);
  			  chatter_last = HAL_GetTick();
  		  }
  	  }
  	  nh.spinOnce();

////    	 uint32_t t = millis();
////    	updateTime();
////    	updateVariable(nh.connected());
////    	updateTFPrefix(nh.connected());
//
//
//    	imu_msg =  getIMU();
////    	imu_pub.publish(&imu_msg);
////    	if ((t-tTime[3]) >= (1000 / IMU_PUBLISH_FREQUENCY))
////    	  {
////    	    publishImuMsg();
////
////    	    tTime[3] = t;
////    	  }
//    	HAL_Delay(10);
//    	nh.spinOnce();
////    	 waitForSerialLink(nh.connected());
    }
}

void ros_setup(void)
{
	nh.initNode();

	nh.subscribe(cmd_vel_sub);
	nh.subscribe(reset_sub);

	nh.advertise(imu_pub);
	nh.advertise(cmd_vel_motor_pub);
	nh.advertise(odom_pub);
	nh.advertise(joint_states_pub);
	nh.advertise(battery_state_pub);

	tf_broadcaster.init(nh);

	initOdom();

	initJointStates();

	prev_update_time = millis();

	setup_end = true;
}

void robot_mpu6050_init(void)
{
	i2c_config_t i2c_config;
	i2c_config.i2c_num = I2C_NUM_1;
	i2c_config.i2c_pins_pack = I2C_PINS_PACK_1;
	i2c_handle_t i2c_handle = i2c_init(&i2c_config);

	mpu6050_i2c = i2c_get_I2C_HandleTypeDef(i2c_handle);
	mpu6050_i2c_config(&mpu6050_i2c);

    mpu6050_config_t mpu6050_config;
    mpu6050_config.afs_sel = MPU6050_AFS_SEL_4G;
    mpu6050_config.clksel = MPU6050_CLKSEL_INTERNAL_8_MHZ;
    mpu6050_config.dlpf_cfg =  MPU6050_184ACCEL_188GYRO_BW_HZ;
    mpu6050_config.fs_sel = MPU6050_FS_SEL_500;
    mpu6050_config.sleep_mode = MPU6050_DISABLE_SLEEP_MODE;
    mpu6050 = mpu6050_init(&mpu6050_config);
}

void robot_motor_init(void)
{
    step_driver_config_t motor_left_config;
    motor_left_config.pin_clk.timer_num = TIMER_NUM_3;
    motor_left_config.pin_clk.timer_channel = TIMER_CHANNEL_2;
	motor_left_config.pin_clk.timer_pins_pack = TIMER_PINS_PACK_1;
	motor_left_config.pin_dir.gpio_port = GPIO_PORT_C;
	motor_left_config.pin_dir.gpio_num = GPIO_NUM_5;
	motor_left_config.pin_dir.gpio_mode = GPIO_OUTPUT;
	motor_left_config.pin_dir.gpio_reg_pull = GPIO_REG_PULL_NONE;
	motor_left = step_driver_init(&motor_left_config);

	step_driver_config_t motor_right_config;
	motor_right_config.pin_clk.timer_num = TIMER_NUM_2;
	motor_right_config.pin_clk.timer_channel = TIMER_CHANNEL_1;
	motor_right_config.pin_clk.timer_pins_pack = TIMER_PINS_PACK_2;
	motor_right_config.pin_dir.gpio_port = GPIO_PORT_A;
	motor_right_config.pin_dir.gpio_num = GPIO_NUM_3;
	motor_right_config.pin_dir.gpio_mode = GPIO_OUTPUT;
	motor_right_config.pin_dir.gpio_reg_pull = GPIO_REG_PULL_NONE;
	motor_right = step_driver_init(&motor_right_config);

	MOTOR_LEFT_FORWARD(motor_left);
	MOTOR_RIGHT_FORWARD(motor_right);
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

void robot_rosserial_init(void)
{
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart4.Instance = UART4;
    huart4.Init.BaudRate = 57600;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart4);

    hdma_uart4_rx.Instance = DMA1_Stream2;
    hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_NORMAL;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_uart4_rx);
    __HAL_LINKDMA(&huart4, hdmarx, hdma_uart4_rx);

    hdma_uart4_tx.Instance = DMA1_Stream4;
    hdma_uart4_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_tx.Init.Mode = DMA_NORMAL;
    hdma_uart4_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart4_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_uart4_tx);
    __HAL_LINKDMA(&huart4, hdmatx, hdma_uart4_tx);

    HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);

    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
    goal_velocity_from_cmd[LINEAR] = cmd_vel_msg.linear.x;
    goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

    goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

    tTime[6] = millis();
}

void resetCallback(const std_msgs::Empty &reset_msg)
{
    char log_msg[50];

    (void)(reset_msg);

    sprintf(log_msg, "Start Calibration of Gyro");
    nh.loginfo(log_msg);

    initOdom();

    sprintf(log_msg, "Reset Odometry");
    nh.loginfo(log_msg);
}

void publishCmdVelFromMotorMsg(void)
{
    cmd_vel_motor_msg.linear.x = goal_velocity_from_motor[LINEAR];
    cmd_vel_motor_msg.angular.z = goal_velocity_from_motor[ANGULAR];

    cmd_vel_motor_pub.publish(&cmd_vel_motor_msg);
}

void publishImuMsg(void)
{
    imu_msg = getIMU();

    imu_msg.header.stamp = rosNow();
    imu_msg.header.frame_id = imu_frame_id;

    imu_pub.publish(&imu_msg);
}

void publishDriveInformation(void)
{
    unsigned long time_now = millis();
    unsigned long step_time = time_now - prev_update_time;
    prev_update_time = time_now;
    ros::Time stamp_now = rosNow();

    calcOdometry((double)(step_time * 0.001));

    updateOdometry();
    odom.header.stamp = stamp_now;
    odom_pub.publish(&odom);

    updateTF(odom_tf);
    odom_tf.header.stamp = stamp_now;
    tf_broadcaster.sendTransform(odom_tf);

    updateJointStates();
    joint_states.header.stamp = stamp_now;
    joint_states_pub.publish(&joint_states);
}

void updateVariable(bool isConnected)
{
    static bool variable_flag = false;

    if (isConnected)
    {
        if (variable_flag == false)
        {
            initIMU();
            initOdom();

            variable_flag = true;
        }
    }
    else
    {
        variable_flag = false;
    }
}
void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
    int32_t current_tick = 0;
    static int32_t last_tick[WHEEL_NUM] = {0, 0};

    if (init_encoder)
    {
        for (int index = 0; index < WHEEL_NUM; index++)
        {
            last_diff_tick[index] = 0;
            last_tick[index]      = 0;
            last_rad[index]       = 0.0;

            last_velocity[index]  = 0.0;
        }

        last_tick[LEFT] = left_tick;
        last_tick[RIGHT] = right_tick;

        init_encoder = false;
        return;
    }

    current_tick = left_tick;

    last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
    last_tick[LEFT]      = current_tick;
    last_rad[LEFT]       += TICK2RAD * (double)last_diff_tick[LEFT];

    current_tick = right_tick;

    last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
    last_tick[RIGHT]      = current_tick;
    last_rad[RIGHT]       += TICK2RAD * (double)last_diff_tick[RIGHT];
}

void updateTime(void)
{
    current_offset = millis();
    current_time = nh.now();
}

ros::Time rosNow(void)
{
  return nh.now();
}

ros::Time addMicros(ros::Time & t, uint32_t _micros)
{
//  uint32_t sec, nsec;
//
//  sec  = _micros / 1000 + t.sec;
//  nsec = _micros % 1000000000 + t.nsec;
//
//  return ros::Time(sec, nsec);
}

void updateOdometry(void)
{
    odom.header.frame_id = odom_header_frame_id;
    odom.child_frame_id  = odom_child_frame_id;

    odom.pose.pose.position.x = odom_pose[0];
    odom.pose.pose.position.y = odom_pose[1];
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

    odom.twist.twist.linear.x  = odom_vel[0];
    odom.twist.twist.angular.z = odom_vel[2];
}

void updateJointStates(void)
{
    static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
    static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
    //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

    joint_states_pos[LEFT]  = last_rad[LEFT];
    joint_states_pos[RIGHT] = last_rad[RIGHT];

    joint_states_vel[LEFT]  = last_velocity[LEFT];
    joint_states_vel[RIGHT] = last_velocity[RIGHT];

    joint_states.position = joint_states_pos;
    joint_states.velocity = joint_states_vel;
}

void updateJoint(void)
{

}

void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
    odom_tf.header = odom.header;
    odom_tf.child_frame_id = odom.child_frame_id;
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

void updateGyroCali(bool isConnected)
{
    static bool isEnded = false;
    char log_msg[50];

    (void)(isConnected);

    if (nh.connected())
    {
        if (isEnded == false)
        {
            sprintf(log_msg, "Start Calibration of Gyro");
            nh.loginfo(log_msg);

            calibrationGyro();

            sprintf(log_msg, "Calibration End");
            nh.loginfo(log_msg);

            isEnded = true;
        }
    }
    else
    {
        isEnded = false;
    }
}

void updateGoalVelocity(void)
{
    goal_velocity[LINEAR]  = goal_velocity_from_button[LINEAR]  + goal_velocity_from_cmd[LINEAR]  + goal_velocity_from_motor[LINEAR];
    goal_velocity[ANGULAR] = goal_velocity_from_button[ANGULAR] + goal_velocity_from_cmd[ANGULAR] + goal_velocity_from_motor[ANGULAR];
}

void updateTFPrefix(bool isConnected)
{
    static bool isChecked = false;
    char log_msg[50];

    if (isConnected)
    {
        if (isChecked == false)
        {
            nh.getParam("~tf_prefix", &get_tf_prefix);

            if (!strcmp(get_tf_prefix, ""))
            {
                sprintf(odom_header_frame_id, "odom");
                sprintf(odom_child_frame_id, "base_footprint");

                sprintf(imu_frame_id, "imu_link");
                sprintf(joint_state_header_frame_id, "base_link");
            }
            else
            {
                strcpy(odom_header_frame_id, get_tf_prefix);
                strcpy(odom_child_frame_id, get_tf_prefix);

                strcpy(imu_frame_id, get_tf_prefix);
                strcpy(joint_state_header_frame_id, get_tf_prefix);

                strcat(odom_header_frame_id, "/odom");
                strcat(odom_child_frame_id, "/base_footprint");

                strcat(imu_frame_id, "/imu_link");
                strcat(joint_state_header_frame_id, "/base_link");
            }

            sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
            nh.loginfo(log_msg);

            sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
            nh.loginfo(log_msg);


            sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
            nh.loginfo(log_msg);

            isChecked = true;
        }
    }
    else
    {
        isChecked = false;
    }
}

void initOdom(void)
{
    init_encoder = true;

    for (int index = 0; index < 3; index++)
    {
        odom_pose[index] = 0.0;
        odom_vel[index]  = 0.0;
    }

    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 0.0;

    odom.twist.twist.linear.x  = 0.0;
    odom.twist.twist.angular.z = 0.0;
}

void initJointStates(void)
{
    static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

    joint_states.header.frame_id = joint_state_header_frame_id;
    joint_states.name            = joint_states_name;

    joint_states.name_length     = WHEEL_NUM;
    joint_states.position_length = WHEEL_NUM;
    joint_states.velocity_length = WHEEL_NUM;
    joint_states.effort_length   = WHEEL_NUM;
}

bool calcOdometry(double diff_time)
{
    float* orientation;
    double wheel_l, wheel_r;      // rotation value of wheel [rad]
    double delta_s, theta, delta_theta;
    static double last_theta = 0.0;
    double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
    double step_time;

    wheel_l = wheel_r = 0.0;
    delta_s = delta_theta = theta = 0.0;
    v = w = 0.0;
    step_time = 0.0;

    step_time = diff_time;

    if (step_time == 0)
        return false;

    wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
    wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

    if (isnan(wheel_l))
        wheel_l = 0.0;

    if (isnan(wheel_r))
        wheel_r = 0.0;

    delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
    // theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
    getOrientation(orientation);
    theta       = atan2f(orientation[1] * orientation[2] + orientation[0] * orientation[3],
                         0.5f - orientation[2] * orientation[2] - orientation[3] * orientation[3]);

    delta_theta = theta - last_theta;

    // compute odometric pose
    odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[2] += delta_theta;

    // compute odometric instantaneouse velocity

    v = delta_s / step_time;
    w = delta_theta / step_time;

    odom_vel[0] = v;
    odom_vel[1] = 0.0;
    odom_vel[2] = w;

    last_velocity[LEFT]  = wheel_l / step_time;
    last_velocity[RIGHT] = wheel_r / step_time;
    last_theta = theta;

    return true;
}

void sendLogMsg(void)
{
    static bool log_flag = false;
    char log_msg[100];

//    String name             = NAME;
//    String firmware_version = FIRMWARE_VER;
//    String bringup_log      = "This core(v" + firmware_version + ") is compatible with TB3 " + name;
//
//    const char* init_log_data = bringup_log.c_str();

    if (nh.connected())
    {
        if (log_flag == false)
        {
            sprintf(log_msg, "--------------------------");
            nh.loginfo(log_msg);

            sprintf(log_msg, "Connected to OpenCR board!");
            nh.loginfo(log_msg);

//        sprintf(log_msg, init_log_data);
//        nh.loginfo(log_msg);

            sprintf(log_msg, "--------------------------");
            nh.loginfo(log_msg);

            log_flag = true;
        }
    }
    else
    {
        log_flag = false;
    }
}

void waitForSerialLink(bool isConnected)
{
    static bool wait_flag = false;

    if (isConnected)
    {
        if (wait_flag == false)
        {
            HAL_Delay(10);

            wait_flag = true;
        }
    }
    else
    {
        wait_flag = false;
    }
}

uint32_t millis(void)
{
    return tickcount_ms;
}

sensor_msgs::Imu getIMU(void)
{
	mpu6050_scaled_data_t accel_scale;
	mpu6050_scaled_data_t gyro_scale;
	mpu6050_quat_data_t quat;
	mpu6050_get_accel_scale(&accel_scale);
	mpu6050_get_gyro_scale(&gyro_scale);
	mpu6050_get_quat(&quat);

	sensor_msgs::Imu imu_msg_;

	  imu_msg_.angular_velocity.x = gyro_scale.x_axis;
	  imu_msg_.angular_velocity.y = gyro_scale.y_axis;
	  imu_msg_.angular_velocity.z = gyro_scale.z_axis;
	  imu_msg_.angular_velocity_covariance[1] = 0;
	  imu_msg_.angular_velocity_covariance[2] = 0;
	  imu_msg_.angular_velocity_covariance[3] = 0;
	  imu_msg_.angular_velocity_covariance[4] = 0.02;
	  imu_msg_.angular_velocity_covariance[5] = 0;
	  imu_msg_.angular_velocity_covariance[6] = 0;
	  imu_msg_.angular_velocity_covariance[7] = 0;
	  imu_msg_.angular_velocity_covariance[8] = 0.02;

	  imu_msg_.linear_acceleration.x = accel_scale.x_axis;
	  imu_msg_.linear_acceleration.y = accel_scale.y_axis;
	  imu_msg_.linear_acceleration.z = accel_scale.z_axis;

	  imu_msg_.linear_acceleration_covariance[0] = 0.04;
	  imu_msg_.linear_acceleration_covariance[1] = 0;
	  imu_msg_.linear_acceleration_covariance[2] = 0;
	  imu_msg_.linear_acceleration_covariance[3] = 0;
	  imu_msg_.linear_acceleration_covariance[4] = 0.04;
	  imu_msg_.linear_acceleration_covariance[5] = 0;
	  imu_msg_.linear_acceleration_covariance[6] = 0;
	  imu_msg_.linear_acceleration_covariance[7] = 0;
	  imu_msg_.linear_acceleration_covariance[8] = 0.04;

	  imu_msg_.orientation.w = quat.q0;
	  imu_msg_.orientation.x = quat.q1;
	  imu_msg_.orientation.y = quat.q2;
	  imu_msg_.orientation.z = quat.q3;

	  imu_msg_.orientation_covariance[0] = 0.0025;
	  imu_msg_.orientation_covariance[1] = 0;
	  imu_msg_.orientation_covariance[2] = 0;
	  imu_msg_.orientation_covariance[3] = 0;
	  imu_msg_.orientation_covariance[4] = 0.0025;
	  imu_msg_.orientation_covariance[5] = 0;
	  imu_msg_.orientation_covariance[6] = 0;
	  imu_msg_.orientation_covariance[7] = 0;
	  imu_msg_.orientation_covariance[8] = 0.0025;

	  return imu_msg_;
}

int constrain(int x, int a, int b)
{
	int value;
	if(x>a)
	{
		value = a;
	}
	else if(x<b)
	{
		value = b;
	}
	else
	{
		value = x;
	}
	return value;
}

void initIMU(void)
{

}

void getOrientation(float *orientation)
{
	mpu6050_quat_data_t quat;
	mpu6050_get_quat(&quat);
	orientation[0] = quat.q0;
	orientation[1] = quat.q1;
	orientation[2] = quat.q2;
	orientation[3] = quat.q3;


}

void calibrationGyro(void)
{

}

void timer_interval_init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  /* USER CODE BEGIN TIM5_Init 1 */

	  /* USER CODE END TIM5_Init 1 */
	  htim5.Instance = TIM5;
	  htim5.Init.Prescaler = 83;
	  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim5.Init.Period = 999;
	  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }




}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance ==  htim5.Instance)
	{
		tickcount_ms++;
	}
}

void Error_Handler(void)
{

}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */

