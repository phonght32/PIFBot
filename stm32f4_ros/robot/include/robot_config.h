#ifndef _ROBOT_CORE_CONFIG_H_
#define _ROBOT_CORE_CONFIG_H_


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
#define CONTROL_MOTOR_TIME_INDEX                0
#define CMD_VEL_PUBLISH_TIME_INDEX              1
#define DRIVE_INFORMATION_PUBLISH_TIME_INDEX    2
#define IMU_PUBLISH_TIME_INDEX                  3
#define CONTROL_MOTOR_TIMEOUT_TIME_INDEX        6

/* Frequency of publish/subscribe */
#define CONTROL_MOTOR_SPEED_FREQUENCY          30   /*!< Hz */
#define CONTROL_MOTOR_TIMEOUT                  500  /*!< ms */
#define IMU_PUBLISH_FREQUENCY                  200  /*!< Hz */
#define CMD_VEL_PUBLISH_FREQUENCY              30   /*!< Hz */
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   /*!< Hz */
#define DEBUG_LOG_FREQUENCY                    10   /*!< Hz */


/* Linear & Angular velocity index */
#define WHEEL_NUM       2

#define LEFT            0
#define RIGHT           1

#define LINEAR          0
#define ANGULAR         1

/* Convert constant */
#define DEG2RAD(x)      (x * 0.01745329252)     /*!< pi/180 */
#define RAD2DEG(x)      (x * 57.2957795131)     /*!< 180/pi */

#define TICK2RAD        0.001533981             /*!< 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f */

/*
 * @brief  This function called when receive any message from "cmd_vel" topic.
 *         Geometry messages content linear and angular velocity.
 * @param  cmd_vel_msg Geometry message.
 * @return None.
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
bool calcOdometry(double diff_time);

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


/* ROS node handle */
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/* ROS parameter */
char get_prefix[10];
char *get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];

char joint_state_header_frame_id[30];

/* Subscriber */
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);
ros::Subscriber<std_msgs::Empty> reset_sub("reset", resetCallback);

/* Publisher */
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


/* Transform Broadcaster */
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/* SoftwareTimer of Robot */
static uint32_t tTime[10];

/* Calculation for odometry */
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
double last_rad[WHEEL_NUM] = {0.0, 0.0};

/* Update Joint State */
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};

/* Declaration for controllers */
float zero_velocity[WHEEL_NUM] = {0.0, 0.0};            /*!< Velocity to stop motor                 */
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};            /*!< Velocity to control motor              */
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};   /*!< Velocity receive from "cmd_vel" topic  */
float goal_velocity_from_motor[WHEEL_NUM] = {0.0, 0.0}; /*!< Velocity read from encoder             */

/* Declaration for SLAM and navigation */
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];

/* Declaration for Battery */
bool setup_end        = false;
uint8_t battery_state = 0;


#endif /* _ROBOT_CORE_CONFIG_H_ */
