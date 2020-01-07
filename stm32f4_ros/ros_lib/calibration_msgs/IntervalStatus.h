#ifndef _ROS_calibration_msgs_IntervalStatus_h
#define _ROS_calibration_msgs_IntervalStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace calibration_msgs
{

  class IntervalStatus : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t names_length;
      typedef char* _names_type;
      _names_type st_names;
      _names_type * names;
      uint32_t yeild_rates_length;
      typedef float _yeild_rates_type;
      _yeild_rates_type st_yeild_rates;
      _yeild_rates_type * yeild_rates;

    IntervalStatus():
      header(),
      names_length(0), names(NULL),
      yeild_rates_length(0), yeild_rates(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->names_length);
      for( uint32_t i = 0; i < names_length; i++){
      uint32_t length_namesi = strlen(this->names[i]);
      varToArr(outbuffer + offset, length_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->names[i], length_namesi);
      offset += length_namesi;
      }
      *(outbuffer + offset + 0) = (this->yeild_rates_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->yeild_rates_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->yeild_rates_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->yeild_rates_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yeild_rates_length);
      for( uint32_t i = 0; i < yeild_rates_length; i++){
      union {
        float real;
        uint32_t base;
      } u_yeild_ratesi;
      u_yeild_ratesi.real = this->yeild_rates[i];
      *(outbuffer + offset + 0) = (u_yeild_ratesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yeild_ratesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yeild_ratesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yeild_ratesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yeild_rates[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->names_length);
      if(names_lengthT > names_length)
        this->names = (char**)realloc(this->names, names_lengthT * sizeof(char*));
      names_length = names_lengthT;
      for( uint32_t i = 0; i < names_length; i++){
      uint32_t length_st_names;
      arrToVar(length_st_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_names-1]=0;
      this->st_names = (char *)(inbuffer + offset-1);
      offset += length_st_names;
        memcpy( &(this->names[i]), &(this->st_names), sizeof(char*));
      }
      uint32_t yeild_rates_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      yeild_rates_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      yeild_rates_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      yeild_rates_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->yeild_rates_length);
      if(yeild_rates_lengthT > yeild_rates_length)
        this->yeild_rates = (float*)realloc(this->yeild_rates, yeild_rates_lengthT * sizeof(float));
      yeild_rates_length = yeild_rates_lengthT;
      for( uint32_t i = 0; i < yeild_rates_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_yeild_rates;
      u_st_yeild_rates.base = 0;
      u_st_yeild_rates.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_yeild_rates.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_yeild_rates.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_yeild_rates.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_yeild_rates = u_st_yeild_rates.real;
      offset += sizeof(this->st_yeild_rates);
        memcpy( &(this->yeild_rates[i]), &(this->st_yeild_rates), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "calibration_msgs/IntervalStatus"; };
    const char * getMD5(){ return "277fe87e9a44a07ab27c97b6b37d5898"; };

  };

}
#endif
