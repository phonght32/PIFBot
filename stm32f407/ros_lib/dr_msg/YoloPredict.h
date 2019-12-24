#ifndef _ROS_dr_msg_YoloPredict_h
#define _ROS_dr_msg_YoloPredict_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "dr_msg/Rect2d.h"

namespace dr_msg
{

  class YoloPredict : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef dr_msg::Rect2d _bbox_type;
      _bbox_type bbox;
      typedef int16_t _objClass_type;
      _objClass_type objClass;
      typedef float _prob_type;
      _prob_type prob;
      typedef const char* _name_type;
      _name_type name;

    YoloPredict():
      header(),
      bbox(),
      objClass(0),
      prob(0),
      name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->bbox.serialize(outbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_objClass;
      u_objClass.real = this->objClass;
      *(outbuffer + offset + 0) = (u_objClass.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_objClass.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->objClass);
      union {
        float real;
        uint32_t base;
      } u_prob;
      u_prob.real = this->prob;
      *(outbuffer + offset + 0) = (u_prob.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_prob.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_prob.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_prob.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->prob);
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->bbox.deserialize(inbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_objClass;
      u_objClass.base = 0;
      u_objClass.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_objClass.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->objClass = u_objClass.real;
      offset += sizeof(this->objClass);
      union {
        float real;
        uint32_t base;
      } u_prob;
      u_prob.base = 0;
      u_prob.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_prob.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_prob.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_prob.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->prob = u_prob.real;
      offset += sizeof(this->prob);
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
     return offset;
    }

    const char * getType(){ return "dr_msg/YoloPredict"; };
    const char * getMD5(){ return "7d0f34ee2190b9b92b4b33974215aec9"; };

  };

}
#endif
