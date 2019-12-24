#ifndef _ROS_SERVICE_CalculateAverageMasurement_h
#define _ROS_SERVICE_CalculateAverageMasurement_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Wrench.h"

namespace force_torque_sensor
{

static const char CALCULATEAVERAGEMASUREMENT[] = "force_torque_sensor/CalculateAverageMasurement";

  class CalculateAverageMasurementRequest : public ros::Msg
  {
    public:
      typedef uint32_t _N_measurements_type;
      _N_measurements_type N_measurements;
      typedef float _T_between_meas_type;
      _T_between_meas_type T_between_meas;
      typedef const char* _frame_id_type;
      _frame_id_type frame_id;

    CalculateAverageMasurementRequest():
      N_measurements(0),
      T_between_meas(0),
      frame_id("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->N_measurements >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->N_measurements >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->N_measurements >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->N_measurements >> (8 * 3)) & 0xFF;
      offset += sizeof(this->N_measurements);
      offset += serializeAvrFloat64(outbuffer + offset, this->T_between_meas);
      uint32_t length_frame_id = strlen(this->frame_id);
      varToArr(outbuffer + offset, length_frame_id);
      offset += 4;
      memcpy(outbuffer + offset, this->frame_id, length_frame_id);
      offset += length_frame_id;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->N_measurements =  ((uint32_t) (*(inbuffer + offset)));
      this->N_measurements |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->N_measurements |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->N_measurements |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->N_measurements);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->T_between_meas));
      uint32_t length_frame_id;
      arrToVar(length_frame_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_frame_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_frame_id-1]=0;
      this->frame_id = (char *)(inbuffer + offset-1);
      offset += length_frame_id;
     return offset;
    }

    const char * getType(){ return CALCULATEAVERAGEMASUREMENT; };
    const char * getMD5(){ return "5080a2ffd70f20ca445070733a6acac3"; };

  };

  class CalculateAverageMasurementResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _message_type;
      _message_type message;
      typedef geometry_msgs::Wrench _measurement_type;
      _measurement_type measurement;

    CalculateAverageMasurementResponse():
      success(0),
      message(""),
      measurement()
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
      offset += this->measurement.serialize(outbuffer + offset);
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
      offset += this->measurement.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return CALCULATEAVERAGEMASUREMENT; };
    const char * getMD5(){ return "72d003f8cbc134e90e0ad06460fe9557"; };

  };

  class CalculateAverageMasurement {
    public:
    typedef CalculateAverageMasurementRequest Request;
    typedef CalculateAverageMasurementResponse Response;
  };

}
#endif
