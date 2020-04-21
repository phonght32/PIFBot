// MIT License

// Copyright (c) 2020 thanhphong98

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef _ROBOT_ROS_CONFIG_H_
#define _ROBOT_ROS_CONFIG_H_

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


/* Time update index */
#define CONTROL_MOTOR_TIME_INDEX                0       /*!< Time index control motor */
#define CMD_VEL_PUBLISH_TIME_INDEX              1       /*!< Time index publish velocity */
#define DRIVE_INFORMATION_PUBLISH_TIME_INDEX    2       /*!< Time index publish drive information */
#define IMU_PUBLISH_TIME_INDEX                  3       /*!< Time index publish IMU information */
#define LOG_PUBLISH_TIME_INDEX                  5
#define CONTROL_MOTOR_TIMEOUT_TIME_INDEX        6       /*!< Time index control motor timeout */

/* Frequency of publish/subscribe */
#define CONTROL_MOTOR_SPEED_FREQUENCY          10       /*!< Frequency in Hz to control motor */
#define CONTROL_MOTOR_TIMEOUT                  500      /*!< Period in ms to check control motor timeout */
#define IMU_PUBLISH_FREQUENCY                  15      	/*!< Frequency in Hz to publish IMU information */
#define CMD_VEL_PUBLISH_FREQUENCY              5       	/*!< Frequency in Hz to publish robot velocity */
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    5       	/*!< Frequency in Hz to publish drive information */
#define DEBUG_LOG_FREQUENCY                    10       /*!< Frequency in Hz to send log debug messages */

/* Linear & Angular velocity index */
#define WHEEL_NUM       2                       /*!< Num wheel */

#define LEFT            0                       /*!< Left wheel index */
#define RIGHT           1                       /*!< Right wheel index */

#define LINEAR          0                       /*!< Linear velocity index */
#define ANGULAR         1                       /*!< Angular velocity index */


/*
 * @brief   This function called when receive any message from "cmd_vel" topic.
 *          Geometry messages content linear and angular velocity.
 * @param   cmd_vel_msg Geometry message.
 * @return  None.
 */
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);

/*
 * @brief   This function called when receive any message from "reset" topic.
 * @param   reset_msg Reset message.
 * @return  None.
 */
void resetCallback(const std_msgs::Empty &reset_msg);

/*
 * @brief   Publish linear and angular velocity to "cmd_vel_motor" topic.
 * @param   None.
 * @return  None.
 */
void publishCmdVelFromMotorMsg(void);

/*
 * @brief   Publish IMU information.
 * @param   None.
 * @return  None.
 */
void publishImuMsg(void);

/*
 * @brief   Publish drive information.
 * @param   None.
 * @return  None.
 */
void publishDriveInformation(void);

/*
 * @brief   Get ROS time current.
 * @param   None.
 * @return  ROS time.
 */
ros::Time rosNow(void);

/*
 * @brief   Add microsecond to ROS time.
 * @param   t Ros time.
 * @param   _micros Time add.
 * @return  ROS time calibration.
 */
ros::Time addMicros(ros::Time &t, uint32_t _micros);

/*
 * @brief   Update variable (Odometry and IMU parameters).
 * @param   isConnected Check rosserial connect.
 * @return  None.
 */
void updateVariable(bool isConnected);

/*
 * @brief   Update motor information.
 * @param   left_tick
 * @pram    right_tick
 * @return  None.
 */
void updateMotorInfo(int32_t left_tick, int32_t right_tick);

/*
 * @brief   Update ROS time from system time.
 * @param   None.
 * @return  None.
 */
void updateTime(void);

/*
 * @brief   Update Odometry.
 * @param   None.
 * @return  None.
 */
void updateOdometry(void);

/*
 * @brief   Update Joint.
 * @param   None.
 * @return  None.
 */
void updateJoint(void);

/*
 * @brief   Update JointStates.
 * @param   None.
 * @return  None.
 */
void updateJointStates(void);

/*
 * @brief   Update Transform.
 * @param   odom_tf Geometry messages content tf information.
 * @return  None.
 */
void updateTF(geometry_msgs::TransformStamped& odom_tf);

/*
 * @brief   Update IMU bias value.
 * @param   isConnected Check rosserial connect.
 * @return  None.
 */
void updateGyroCali(bool isConnected);

/*
 * @brief   Update goal velocity to control motor.
 * @param   None.
 * @return  None.
 */
void updateGoalVelocity(void);

/*
 * @brief   Update TFPrefix.
 * @param   None.
 * @return  None.
 */
void updateTFPrefix(bool isConnected);

/*
 * @brief   Initialize Odometry.
 * @param   None.
 * @return  None.
 */
void initOdom(void);

/*
 * @brief   Initialize JointStates
 * @param   None.
 * @return  None.
 */
void initJointStates(void);

/*
 * @brief   Calculate Odometry.
 * @param   diff_time
 * @return  None.
 */
bool calcOdometry(float diff_time);

/*
 * @brief   Send Log messages.
 * @param   None.
 * @return  None.
 */
void sendLogMsg(void);

/*
 * @brief   Wait rosserial connection.
 * @param   isConnected Check rosserial connect.
 * @return  None.
 */
void waitForSerialLink(bool isConnected);

/*
 * @brief   ROS setup node handle, rosserial connection, ...
 * @param   None.
 * @return  None.
 */
void ros_setup(void);

/*
 * @brief   Control robot adapt to linear and angular velocity.
 * @param   goal_vel Linear and angular velocity.
 * @return  None.
 */
void controlMotor(float *goal_vel);

/*
 * @brief   Get motor speed.
 * @param   vel Pointer data.
 * @return  None.
 */
void getMotorSpeed(float *vel);

/*
 * @brief   Get quaternion components.
 * @param   None.
 * @return  IMU message type sensor_msg::Imu.
 */
sensor_msgs::Imu getIMU(void);

/*
 * Log message buffer.
 */
char log_msg[100];

/*
 * ROS NodeHandle.
 */
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*
 * ROS parameters.
 */
char get_prefix[10];
char *get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];

char joint_state_header_frame_id[30];

/*
 * Subscribers.
 */
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);
ros::Subscriber<std_msgs::Empty> reset_sub("reset", resetCallback);

/*
 * Publishers.
 */
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

geometry_msgs::Twist cmd_vel_motor_msg;
ros::Publisher cmd_vel_motor_pub("cmd_vel_motor", &cmd_vel_motor_msg);

nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

sensor_msgs::BatteryState battery_state_msg;
ros::Publisher battery_state_pub("battery_state", &battery_state_msg);

/*
 * Transform Broadcaster.
 */
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/*
 * Software Timer of Robot.
 */
static uint32_t tTime[10];

/*
 * Calculation for odometry.
 */
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
float last_rad[WHEEL_NUM] = {0.0, 0.0};

/*
 * Update Joint State.
 */
float  last_velocity[WHEEL_NUM]  = {0.0, 0.0};

/*
 * Declaration for controllers.
 */
float zero_velocity[WHEEL_NUM] = {0.0, 0.0};            /*!< Velocity to stop motor                 */
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};            /*!< Velocity to control motor              */
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};   /*!< Velocity receive from "cmd_vel" topic  */
float goal_velocity_from_motor[WHEEL_NUM] = {0.0, 0.0}; /*!< Velocity read from encoder             */

/*
 * Declaration for SLAM and Navigation.
 */
unsigned long prev_update_time;
float odom_pose[3];
float odom_vel[3];

/*
 * Decalration for battery.
 */
uint8_t battery_state = 0;

/*
 * Declaration for setup flag.
 */
bool setup_end        = false;

#endif /* _ROBOT_ROS_CONFIG_H_ */