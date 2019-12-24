#ifndef _ROS_intera_core_msgs_JointLimits_h
#define _ROS_intera_core_msgs_JointLimits_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace intera_core_msgs
{

  class JointLimits : public ros::Msg
  {
    public:
      uint32_t joint_names_length;
      typedef char* _joint_names_type;
      _joint_names_type st_joint_names;
      _joint_names_type * joint_names;
      uint32_t position_lower_length;
      typedef float _position_lower_type;
      _position_lower_type st_position_lower;
      _position_lower_type * position_lower;
      uint32_t position_upper_length;
      typedef float _position_upper_type;
      _position_upper_type st_position_upper;
      _position_upper_type * position_upper;
      uint32_t velocity_length;
      typedef float _velocity_type;
      _velocity_type st_velocity;
      _velocity_type * velocity;
      uint32_t accel_length;
      typedef float _accel_type;
      _accel_type st_accel;
      _accel_type * accel;
      uint32_t effort_length;
      typedef float _effort_type;
      _effort_type st_effort;
      _effort_type * effort;

    JointLimits():
      joint_names_length(0), joint_names(NULL),
      position_lower_length(0), position_lower(NULL),
      position_upper_length(0), position_upper(NULL),
      velocity_length(0), velocity(NULL),
      accel_length(0), accel(NULL),
      effort_length(0), effort(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->joint_names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_names_length);
      for( uint32_t i = 0; i < joint_names_length; i++){
      uint32_t length_joint_namesi = strlen(this->joint_names[i]);
      varToArr(outbuffer + offset, length_joint_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_names[i], length_joint_namesi);
      offset += length_joint_namesi;
      }
      *(outbuffer + offset + 0) = (this->position_lower_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->position_lower_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->position_lower_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->position_lower_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_lower_length);
      for( uint32_t i = 0; i < position_lower_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->position_lower[i]);
      }
      *(outbuffer + offset + 0) = (this->position_upper_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->position_upper_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->position_upper_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->position_upper_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_upper_length);
      for( uint32_t i = 0; i < position_upper_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->position_upper[i]);
      }
      *(outbuffer + offset + 0) = (this->velocity_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocity_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->velocity_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->velocity_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_length);
      for( uint32_t i = 0; i < velocity_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->velocity[i]);
      }
      *(outbuffer + offset + 0) = (this->accel_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->accel_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->accel_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->accel_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accel_length);
      for( uint32_t i = 0; i < accel_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->accel[i]);
      }
      *(outbuffer + offset + 0) = (this->effort_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->effort_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->effort_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->effort_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->effort_length);
      for( uint32_t i = 0; i < effort_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->effort[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t joint_names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_names_length);
      if(joint_names_lengthT > joint_names_length)
        this->joint_names = (char**)realloc(this->joint_names, joint_names_lengthT * sizeof(char*));
      joint_names_length = joint_names_lengthT;
      for( uint32_t i = 0; i < joint_names_length; i++){
      uint32_t length_st_joint_names;
      arrToVar(length_st_joint_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_joint_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_joint_names-1]=0;
      this->st_joint_names = (char *)(inbuffer + offset-1);
      offset += length_st_joint_names;
        memcpy( &(this->joint_names[i]), &(this->st_joint_names), sizeof(char*));
      }
      uint32_t position_lower_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      position_lower_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      position_lower_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      position_lower_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->position_lower_length);
      if(position_lower_lengthT > position_lower_length)
        this->position_lower = (float*)realloc(this->position_lower, position_lower_lengthT * sizeof(float));
      position_lower_length = position_lower_lengthT;
      for( uint32_t i = 0; i < position_lower_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_position_lower));
        memcpy( &(this->position_lower[i]), &(this->st_position_lower), sizeof(float));
      }
      uint32_t position_upper_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      position_upper_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      position_upper_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      position_upper_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->position_upper_length);
      if(position_upper_lengthT > position_upper_length)
        this->position_upper = (float*)realloc(this->position_upper, position_upper_lengthT * sizeof(float));
      position_upper_length = position_upper_lengthT;
      for( uint32_t i = 0; i < position_upper_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_position_upper));
        memcpy( &(this->position_upper[i]), &(this->st_position_upper), sizeof(float));
      }
      uint32_t velocity_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->velocity_length);
      if(velocity_lengthT > velocity_length)
        this->velocity = (float*)realloc(this->velocity, velocity_lengthT * sizeof(float));
      velocity_length = velocity_lengthT;
      for( uint32_t i = 0; i < velocity_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_velocity));
        memcpy( &(this->velocity[i]), &(this->st_velocity), sizeof(float));
      }
      uint32_t accel_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      accel_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      accel_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      accel_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->accel_length);
      if(accel_lengthT > accel_length)
        this->accel = (float*)realloc(this->accel, accel_lengthT * sizeof(float));
      accel_length = accel_lengthT;
      for( uint32_t i = 0; i < accel_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_accel));
        memcpy( &(this->accel[i]), &(this->st_accel), sizeof(float));
      }
      uint32_t effort_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->effort_length);
      if(effort_lengthT > effort_length)
        this->effort = (float*)realloc(this->effort, effort_lengthT * sizeof(float));
      effort_length = effort_lengthT;
      for( uint32_t i = 0; i < effort_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_effort));
        memcpy( &(this->effort[i]), &(this->st_effort), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/JointLimits"; };
    const char * getMD5(){ return "c4c445eb2c9324525a704c84ca1e7598"; };

  };

}
#endif
