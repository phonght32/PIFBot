#include "stm32f4xx_hal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "stm_log.h"

#include "robot_hardware.h"
#include "robot_utils.h"
#include "robot_config.h"

// static const char *TAG = "APP_MAIN";


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

}

int main(void)
{
    HAL_Init();
    system_clock_init();
    stm_log_init();

    stm_log_level_set("*", STM_LOG_NONE);
    stm_log_level_set("APP_MAIN", STM_LOG_INFO);

    xTaskCreate(example_task, "example_task", 512, NULL, 5, NULL);
    vTaskStartScheduler();
}


void ros_setup(void)
{
    nh.initNode();                      /*!< Init ROS node handle */

    nh.subscribe(cmd_vel_sub);          /*!< Subscribe "cmd_vel" topic to get motor cmd */
    nh.subscribe(reset_sub);            /*!< Subscribe "reset" topic */

    nh.advertise(imu_pub);              /*!< Register the publisher to "imu" topic */
    nh.advertise(cmd_vel_motor_pub);    /*!< Register the publisher to "cmd_vel_motor" topic */
    nh.advertise(odom_pub);             /*!< Register the publisher to "odom" topic */
    nh.advertise(joint_states_pub);     /*!< Register the publisher to "joint_states" topic */
    nh.advertise(battery_state_pub);    /*!< Register the publisher to "battery_state" topic */

    tf_broadcaster.init(nh);            /*!< Init TransformBroadcaster */
    initOdom();                         /*!< Init odometry value */
    initJointStates();                  /*!< Init joint state */

    prev_update_time = millis();        /*!< Update time */
    setup_end = true;                   /*!< Flag for setup completed */
}

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
    /* Get goal velocity */
    goal_velocity_from_cmd[LINEAR] = cmd_vel_msg.linear.x;
    goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

    /* Constrain velocity */
    goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

    /* Update time */
    tTime[CONTROL_MOTOR_TIMEOUT_TIME_INDEX] = millis();
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
    /* Get motor velocity */
    cmd_vel_motor_msg.linear.x = goal_velocity_from_motor[LINEAR];
    cmd_vel_motor_msg.angular.z = goal_velocity_from_motor[ANGULAR];

    /* Publish veloctiy to "cmd_vel_motor" topic */
    cmd_vel_motor_pub.publish(&cmd_vel_motor_msg);
}

void publishImuMsg(void)
{
    /* Get IMU data (accelerometer, gyroscope, quaternion and variance ) */
    imu_msg = getIMU();

    imu_msg.header.stamp = rosNow();
    imu_msg.header.frame_id = imu_frame_id;

    /* Publish IMU messages */
    imu_pub.publish(&imu_msg);
}

void publishDriveInformation(void)
{
    /* Update time */
    unsigned long time_now = millis();
    unsigned long step_time = time_now - prev_update_time;
    prev_update_time = time_now;
    ros::Time stamp_now = rosNow();

    /* Calculate odometry */
    calcOdometry((float)(step_time * 0.001f));

    /* Publish odometry message */
    updateOdometry();
    odom.header.stamp = stamp_now;
    odom_pub.publish(&odom);

    /* Publish TF message */
    updateTF(odom_tf);
    odom_tf.header.stamp = stamp_now;
    tf_broadcaster.sendTransform(odom_tf);

    /* Publish jointStates message */
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
//            initIMU();
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
            last_rad[index]       = 0.0f;

            last_velocity[index]  = 0.0f;
        }

        last_tick[LEFT] = left_tick;
        last_tick[RIGHT] = right_tick;

        init_encoder = false;
        return;
    }

    current_tick = left_tick;

    last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
    last_tick[LEFT]      = current_tick;
    last_rad[LEFT]       += TICK2RAD * (float)last_diff_tick[LEFT];

    current_tick = right_tick;

    last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
    last_tick[RIGHT]      = current_tick;
    last_rad[RIGHT]       += TICK2RAD * (float)last_diff_tick[RIGHT];
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
    uint32_t sec, nsec;

    sec  = _micros / 1000 + t.sec;
    nsec = _micros % 1000000000 + t.nsec;

    return ros::Time(sec, nsec);
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

    joint_states.position = (double *)joint_states_pos;
    joint_states.velocity = (double *)joint_states_vel;
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

            //calibrationGyro();

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
    goal_velocity[LINEAR]  = goal_velocity_from_cmd[LINEAR];
    goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
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

bool calcOdometry(float diff_time)
{
    float wheel_l, wheel_r;      // rotation value of wheel [rad]
    float delta_s, theta, delta_theta;
    static float last_theta = 0.0f;
    float v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
    float step_time;

    wheel_l = wheel_r = 0.0f;
    delta_s = delta_theta = theta = 0.0f;
    v = w = 0.0f;
    step_time = 0.0f;

    step_time = diff_time;

    if (step_time == 0)
        return false;

    wheel_l = TICK2RAD * (float)last_diff_tick[LEFT];
    wheel_r = TICK2RAD * (float)last_diff_tick[RIGHT];

    if (isnan(wheel_l))
        wheel_l = 0.0f;

    if (isnan(wheel_r))
        wheel_r = 0.0f;

    delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0f;
//     theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;

    float quat_data[4];
    robot_imu_get_quat(quat_data);
    theta = atan2f(quat_data[0] * quat_data[3] + quat_data[1] * quat_data[2], 0.5f - quat_data[2] * quat_data[2] - quat_data[3] * quat_data[3]);
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

    if (nh.connected())
    {
        if (log_flag == false)
        {
            sprintf(log_msg, "--------------------------");
            nh.loginfo(log_msg);

            sprintf(log_msg, "Connected to limo-board v1.0");
            nh.loginfo(log_msg);

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

sensor_msgs::Imu getIMU(void)
{
    float accel[3], gyro[3], quaternion[4];
    robot_imu_get_accel(accel);
    robot_imu_get_gyro(gyro);
    robot_imu_get_quat(quaternion);

    sensor_msgs::Imu imu_msg_;

    imu_msg_.angular_velocity.x = gyro[0];
    imu_msg_.angular_velocity.y = gyro[1];
    imu_msg_.angular_velocity.z = gyro[2];

    imu_msg_.linear_acceleration.x = accel[0];
    imu_msg_.linear_acceleration.y = accel[1];
    imu_msg_.linear_acceleration.z = accel[2];

    imu_msg_.orientation.w = quaternion[0];
    imu_msg_.orientation.x = quaternion[1];
    imu_msg_.orientation.y = quaternion[2];
    imu_msg_.orientation.z = quaternion[3];

    return imu_msg_;
}

void initIMUCovariance(void)
{
    imu_msg.angular_velocity_covariance[1] = 0;
    imu_msg.angular_velocity_covariance[2] = 0;
    imu_msg.angular_velocity_covariance[3] = 0;
    imu_msg.angular_velocity_covariance[4] = 0.02;
    imu_msg.angular_velocity_covariance[5] = 0;
    imu_msg.angular_velocity_covariance[6] = 0;
    imu_msg.angular_velocity_covariance[7] = 0;
    imu_msg.angular_velocity_covariance[8] = 0.02;

    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[1] = 0;
    imu_msg.linear_acceleration_covariance[2] = 0;
    imu_msg.linear_acceleration_covariance[3] = 0;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[5] = 0;
    imu_msg.linear_acceleration_covariance[6] = 0;
    imu_msg.linear_acceleration_covariance[7] = 0;
    imu_msg.linear_acceleration_covariance[8] = 0.04;

    imu_msg.orientation_covariance[0] = 0.0025;
    imu_msg.orientation_covariance[1] = 0;
    imu_msg.orientation_covariance[2] = 0;
    imu_msg.orientation_covariance[3] = 0;
    imu_msg.orientation_covariance[4] = 0.0025;
    imu_msg.orientation_covariance[5] = 0;
    imu_msg.orientation_covariance[6] = 0;
    imu_msg.orientation_covariance[7] = 0;
    imu_msg.orientation_covariance[8] = 0.0025;
}

void controlMotor(float *goal_vel)
{
    float wheel_velocity_cmd[2];

    float lin_vel = goal_vel[LEFT];
    float ang_vel = goal_vel[RIGHT];

    wheel_velocity_cmd[LEFT]  = lin_vel - (ang_vel * WHEEL_SEPARATION / 2);
    wheel_velocity_cmd[RIGHT] = lin_vel + (ang_vel * WHEEL_SEPARATION / 2);

    wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT], MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT], MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);

    if (wheel_velocity_cmd[LEFT] < 0)
    {
        robot_motor_left_backward();
        robot_motor_left_set_speed(-wheel_velocity_cmd[LEFT]);
    }
    else
    {
        robot_motor_left_forward();
        robot_motor_left_set_speed(wheel_velocity_cmd[LEFT]);
    }

    if (wheel_velocity_cmd[RIGHT] < 0)
    {
        robot_motor_right_backward();
        robot_motor_right_set_speed(-wheel_velocity_cmd[RIGHT]);
    }
    else
    {
        robot_motor_right_forward();
        robot_motor_right_set_speed(wheel_velocity_cmd[RIGHT]);
    }

    if (wheel_velocity_cmd[LEFT])
    {
        robot_motor_left_start();
    }
    else
    {
        robot_motor_left_stop();
    }
    if (wheel_velocity_cmd[RIGHT])
    {
        robot_motor_right_start();
    }
    else
    {
        robot_motor_right_stop();
    }
}

void getMotorSpeed(float *vel)
{
    goal_velocity_from_motor[LINEAR] = goal_velocity_from_cmd[LINEAR];
    goal_velocity_from_motor[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
}