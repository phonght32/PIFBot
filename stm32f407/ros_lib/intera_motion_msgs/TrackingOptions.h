#ifndef _ROS_intera_motion_msgs_TrackingOptions_h
#define _ROS_intera_motion_msgs_TrackingOptions_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace intera_motion_msgs
{

  class TrackingOptions : public ros::Msg
  {
    public:
      typedef bool _use_min_time_rate_type;
      _use_min_time_rate_type use_min_time_rate;
      typedef float _min_time_rate_type;
      _min_time_rate_type min_time_rate;
      typedef bool _use_max_time_rate_type;
      _use_max_time_rate_type use_max_time_rate;
      typedef float _max_time_rate_type;
      _max_time_rate_type max_time_rate;
      uint32_t goal_joint_tolerance_length;
      typedef float _goal_joint_tolerance_type;
      _goal_joint_tolerance_type st_goal_joint_tolerance;
      _goal_joint_tolerance_type * goal_joint_tolerance;
      typedef bool _use_settling_time_at_goal_type;
      _use_settling_time_at_goal_type use_settling_time_at_goal;
      typedef float _settling_time_at_goal_type;
      _settling_time_at_goal_type settling_time_at_goal;

    TrackingOptions():
      use_min_time_rate(0),
      min_time_rate(0),
      use_max_time_rate(0),
      max_time_rate(0),
      goal_joint_tolerance_length(0), goal_joint_tolerance(NULL),
      use_settling_time_at_goal(0),
      settling_time_at_goal(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_use_min_time_rate;
      u_use_min_time_rate.real = this->use_min_time_rate;
      *(outbuffer + offset + 0) = (u_use_min_time_rate.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->use_min_time_rate);
      offset += serializeAvrFloat64(outbuffer + offset, this->min_time_rate);
      union {
        bool real;
        uint8_t base;
      } u_use_max_time_rate;
      u_use_max_time_rate.real = this->use_max_time_rate;
      *(outbuffer + offset + 0) = (u_use_max_time_rate.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->use_max_time_rate);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_time_rate);
      *(outbuffer + offset + 0) = (this->goal_joint_tolerance_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->goal_joint_tolerance_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->goal_joint_tolerance_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->goal_joint_tolerance_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->goal_joint_tolerance_length);
      for( uint32_t i = 0; i < goal_joint_tolerance_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->goal_joint_tolerance[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_use_settling_time_at_goal;
      u_use_settling_time_at_goal.real = this->use_settling_time_at_goal;
      *(outbuffer + offset + 0) = (u_use_settling_time_at_goal.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->use_settling_time_at_goal);
      offset += serializeAvrFloat64(outbuffer + offset, this->settling_time_at_goal);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_use_min_time_rate;
      u_use_min_time_rate.base = 0;
      u_use_min_time_rate.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->use_min_time_rate = u_use_min_time_rate.real;
      offset += sizeof(this->use_min_time_rate);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->min_time_rate));
      union {
        bool real;
        uint8_t base;
      } u_use_max_time_rate;
      u_use_max_time_rate.base = 0;
      u_use_max_time_rate.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->use_max_time_rate = u_use_max_time_rate.real;
      offset += sizeof(this->use_max_time_rate);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_time_rate));
      uint32_t goal_joint_tolerance_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      goal_joint_tolerance_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      goal_joint_tolerance_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      goal_joint_tolerance_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->goal_joint_tolerance_length);
      if(goal_joint_tolerance_lengthT > goal_joint_tolerance_length)
        this->goal_joint_tolerance = (float*)realloc(this->goal_joint_tolerance, goal_joint_tolerance_lengthT * sizeof(float));
      goal_joint_tolerance_length = goal_joint_tolerance_lengthT;
      for( uint32_t i = 0; i < goal_joint_tolerance_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_goal_joint_tolerance));
        memcpy( &(this->goal_joint_tolerance[i]), &(this->st_goal_joint_tolerance), sizeof(float));
      }
      union {
        bool real;
        uint8_t base;
      } u_use_settling_time_at_goal;
      u_use_settling_time_at_goal.base = 0;
      u_use_settling_time_at_goal.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->use_settling_time_at_goal = u_use_settling_time_at_goal.real;
      offset += sizeof(this->use_settling_time_at_goal);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->settling_time_at_goal));
     return offset;
    }

    const char * getType(){ return "intera_motion_msgs/TrackingOptions"; };
    const char * getMD5(){ return "e848e8a266b514c3bde707d0e1859055"; };

  };

}
#endif
