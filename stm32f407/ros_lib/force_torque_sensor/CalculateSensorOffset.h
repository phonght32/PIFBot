#ifndef _ROS_SERVICE_CalculateSensorOffset_h
#define _ROS_SERVICE_CalculateSensorOffset_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Wrench.h"

namespace force_torque_sensor
{

static const char CALCULATESENSOROFFSET[] = "force_torque_sensor/CalculateSensorOffset";

  class CalculateSensorOffsetRequest : public ros::Msg
  {
    public:
      typedef bool _apply_after_calculation_type;
      _apply_after_calculation_type apply_after_calculation;

    CalculateSensorOffsetRequest():
      apply_after_calculation(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_apply_after_calculation;
      u_apply_after_calculation.real = this->apply_after_calculation;
      *(outbuffer + offset + 0) = (u_apply_after_calculation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->apply_after_calculation);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_apply_after_calculation;
      u_apply_after_calculation.base = 0;
      u_apply_after_calculation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->apply_after_calculation = u_apply_after_calculation.real;
      offset += sizeof(this->apply_after_calculation);
     return offset;
    }

    const char * getType(){ return CALCULATESENSOROFFSET; };
    const char * getMD5(){ return "cd1023ae75b839b2ab353031c0d5c50d"; };

  };

  class CalculateSensorOffsetResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _message_type;
      _message_type message;
      typedef geometry_msgs::Wrench _offset_type;
      _offset_type offset;

    CalculateSensorOffsetResponse():
      success(0),
      message(""),
      offset()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      offset += this->offset.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
      offset += this->offset.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return CALCULATESENSOROFFSET; };
    const char * getMD5(){ return "8493d26f08650809172e95bd62163dc4"; };

  };

  class CalculateSensorOffset {
    public:
    typedef CalculateSensorOffsetRequest Request;
    typedef CalculateSensorOffsetResponse Response;
  };

}
#endif
