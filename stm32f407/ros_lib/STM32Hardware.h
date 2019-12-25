/*
 * STM32Hardware.h
 *
 *  Created on: Aug 14, 2018
 *      Author: yusaku
 */

#ifndef ROSSERIAL_CLIENT_SRC_ROS_LIB_STM32HARDWARE_H_
#define ROSSERIAL_CLIENT_SRC_ROS_LIB_STM32HARDWARE_H_

#include "SerialClass.h"

class STM32Hardware
{
public:
	STM32Hardware() :
			com(&serial)
	{

	}

	void init()
	{
		com->start_dma();
	}

	int read()
	{
		return com->read();
	}

	void write(uint8_t* data, int length)
	{
		com->write(data, length);
	}

	unsigned long time()
	{
		return HAL_GetTick();
	}

protected:
	SerialClass* com;
	//long baud_;
};

#endif

