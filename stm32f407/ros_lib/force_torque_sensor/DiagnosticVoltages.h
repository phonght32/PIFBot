#ifndef _ROS_SERVICE_DiagnosticVoltages_h
#define _ROS_SERVICE_DiagnosticVoltages_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace force_torque_sensor
{

static const char DIAGNOSTICVOLTAGES[] = "force_torque_sensor/DiagnosticVoltages";

  class DiagnosticVoltagesRequest : public ros::Msg
  {
    public:
      typedef int8_t _index_type;
      _index_type index;

    DiagnosticVoltagesRequest():
      index(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_index;
      u_index.real = this->index;
      *(outbuffer + offset + 0) = (u_index.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->index);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_index;
      u_index.base = 0;
      u_index.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->index = u_index.real;
      offset += sizeof(this->index);
     return offset;
    }

    const char * getType(){ return DIAGNOSTICVOLTAGES; };
    const char * getMD5(){ return "6d2a262aadb2b043bfcb1714fc57440a"; };

  };

  class DiagnosticVoltagesResponse : public ros::Msg
  {
    public:
      typedef int16_t _adc_value_type;
      _adc_value_type adc_value;

    DiagnosticVoltagesResponse():
      adc_value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_adc_value;
      u_adc_value.real = this->adc_value;
      *(outbuffer + offset + 0) = (u_adc_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_adc_value.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->adc_value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_adc_value;
      u_adc_value.base = 0;
      u_adc_value.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_adc_value.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->adc_value = u_adc_value.real;
      offset += sizeof(this->adc_value);
     return offset;
    }

    const char * getType(){ return DIAGNOSTICVOLTAGES; };
    const char * getMD5(){ return "121d2634960410861dbe13a3e771c9db"; };

  };

  class DiagnosticVoltages {
    public:
    typedef DiagnosticVoltagesRequest Request;
    typedef DiagnosticVoltagesResponse Response;
  };

}
#endif
