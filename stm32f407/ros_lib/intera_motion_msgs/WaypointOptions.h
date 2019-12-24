#ifndef _ROS_intera_motion_msgs_WaypointOptions_h
#define _ROS_intera_motion_msgs_WaypointOptions_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace intera_motion_msgs
{

  class WaypointOptions : public ros::Msg
  {
    public:
      typedef const char* _label_type;
      _label_type label;
      typedef float _max_joint_speed_ratio_type;
      _max_joint_speed_ratio_type max_joint_speed_ratio;
      uint32_t joint_tolerances_length;
      typedef float _joint_tolerances_type;
      _joint_tolerances_type st_joint_tolerances;
      _joint_tolerances_type * joint_tolerances;
      uint32_t max_joint_accel_length;
      typedef float _max_joint_accel_type;
      _max_joint_accel_type st_max_joint_accel;
      _max_joint_accel_type * max_joint_accel;
      typedef float _max_linear_speed_type;
      _max_linear_speed_type max_linear_speed;
      typedef float _max_linear_accel_type;
      _max_linear_accel_type max_linear_accel;
      typedef float _max_rotational_speed_type;
      _max_rotational_speed_type max_rotational_speed;
      typedef float _max_rotational_accel_type;
      _max_rotational_accel_type max_rotational_accel;
      typedef float _corner_distance_type;
      _corner_distance_type corner_distance;

    WaypointOptions():
      label(""),
      max_joint_speed_ratio(0),
      joint_tolerances_length(0), joint_tolerances(NULL),
      max_joint_accel_length(0), max_joint_accel(NULL),
      max_linear_speed(0),
      max_linear_accel(0),
      max_rotational_speed(0),
      max_rotational_accel(0),
      corner_distance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_label = strlen(this->label);
      varToArr(outbuffer + offset, length_label);
      offset += 4;
      memcpy(outbuffer + offset, this->label, length_label);
      offset += length_label;
      offset += serializeAvrFloat64(outbuffer + offset, this->max_joint_speed_ratio);
      *(outbuffer + offset + 0) = (this->joint_tolerances_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_tolerances_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_tolerances_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_tolerances_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_tolerances_length);
      for( uint32_t i = 0; i < joint_tolerances_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_tolerances[i]);
      }
      *(outbuffer + offset + 0) = (this->max_joint_accel_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->max_joint_accel_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->max_joint_accel_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->max_joint_accel_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_joint_accel_length);
      for( uint32_t i = 0; i < max_joint_accel_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->max_joint_accel[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->max_linear_speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_linear_accel);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_rotational_speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_rotational_accel);
      offset += serializeAvrFloat64(outbuffer + offset, this->corner_distance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_label;
      arrToVar(length_label, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_label; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_label-1]=0;
      this->label = (char *)(inbuffer + offset-1);
      offset += length_label;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_joint_speed_ratio));
      uint32_t joint_tolerances_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_tolerances_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_tolerances_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_tolerances_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_tolerances_length);
      if(joint_tolerances_lengthT > joint_tolerances_length)
        this->joint_tolerances = (float*)realloc(this->joint_tolerances, joint_tolerances_lengthT * sizeof(float));
      joint_tolerances_length = joint_tolerances_lengthT;
      for( uint32_t i = 0; i < joint_tolerances_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_joint_tolerances));
        memcpy( &(this->joint_tolerances[i]), &(this->st_joint_tolerances), sizeof(float));
      }
      uint32_t max_joint_accel_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      max_joint_accel_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      max_joint_accel_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      max_joint_accel_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->max_joint_accel_length);
      if(max_joint_accel_lengthT > max_joint_accel_length)
        this->max_joint_accel = (float*)realloc(this->max_joint_accel, max_joint_accel_lengthT * sizeof(float));
      max_joint_accel_length = max_joint_accel_lengthT;
      for( uint32_t i = 0; i < max_joint_accel_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_max_joint_accel));
        memcpy( &(this->max_joint_accel[i]), &(this->st_max_joint_accel), sizeof(float));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_linear_speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_linear_accel));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_rotational_speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_rotational_accel));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->corner_distance));
     return offset;
    }

    const char * getType(){ return "intera_motion_msgs/WaypointOptions"; };
    const char * getMD5(){ return "1b4687d4e536269b06e629169723339f"; };

  };

}
#endif
