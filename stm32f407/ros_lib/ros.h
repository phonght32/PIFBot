/*
 * ros.h
 *
 *  Created on: Aug 15, 2018
 *      Author: yusaku
 */

#ifndef ROSSERIAL_CLIENT_SRC_ROS_LIB_ROS_H_
#define ROSSERIAL_CLIENT_SRC_ROS_LIB_ROS_H_

#include "ros/node_handle.h"
#include "STM32Hardware.h"

namespace ros
{
typedef NodeHandle_<STM32Hardware, 25, 25, BUF_SIZE, BUF_SIZE> NodeHandle;
}

#endif /* ROSSERIAL_CLIENT_SRC_ROS_LIB_ROS_H_ */
