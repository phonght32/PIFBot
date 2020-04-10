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

#ifndef _ROBOT_CONFIG_H_
#define _ROBOT_CONFIG_H_

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
#define CONTROL_MOTOR_SPEED_FREQUENCY          30       /*!< Frequency in Hz to control motor */
#define CONTROL_MOTOR_TIMEOUT                  500      /*!< Period in ms to check control motor timeout */
#define IMU_PUBLISH_FREQUENCY                  200      /*!< Frequency in Hz to publish IMU information */
#define CMD_VEL_PUBLISH_FREQUENCY              30       /*!< Frequency in Hz to publish robot velocity */
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30       /*!< Frequency in Hz to publish drive information */
#define DEBUG_LOG_FREQUENCY                    10       /*!< Frequency in Hz to send log debug messages */

/* Linear & Angular velocity index */
#define WHEEL_NUM       2                       /*!< Num wheel */

#define LEFT            0                       /*!< Left wheel index */
#define RIGHT           1                       /*!< Right wheel index */

#define LINEAR          0                       /*!< Linear velocity index */
#define ANGULAR         1                       /*!< Angular velocity index */




#endif /* _ROBOT_CONFIG_H_ */