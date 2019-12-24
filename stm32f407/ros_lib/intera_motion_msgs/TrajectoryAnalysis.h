#ifndef _ROS_intera_motion_msgs_TrajectoryAnalysis_h
#define _ROS_intera_motion_msgs_TrajectoryAnalysis_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace intera_motion_msgs
{

  class TrajectoryAnalysis : public ros::Msg
  {
    public:
      typedef float _planned_duration_type;
      _planned_duration_type planned_duration;
      typedef float _measured_duration_type;
      _measured_duration_type measured_duration;
      uint32_t min_angle_command_length;
      typedef float _min_angle_command_type;
      _min_angle_command_type st_min_angle_command;
      _min_angle_command_type * min_angle_command;
      uint32_t max_angle_command_length;
      typedef float _max_angle_command_type;
      _max_angle_command_type st_max_angle_command;
      _max_angle_command_type * max_angle_command;
      uint32_t peak_speed_command_length;
      typedef float _peak_speed_command_type;
      _peak_speed_command_type st_peak_speed_command;
      _peak_speed_command_type * peak_speed_command;
      uint32_t peak_accel_command_length;
      typedef float _peak_accel_command_type;
      _peak_accel_command_type st_peak_accel_command;
      _peak_accel_command_type * peak_accel_command;
      uint32_t peak_jerk_command_length;
      typedef float _peak_jerk_command_type;
      _peak_jerk_command_type st_peak_jerk_command;
      _peak_jerk_command_type * peak_jerk_command;
      typedef float _min_time_rate_type;
      _min_time_rate_type min_time_rate;
      typedef float _max_time_rate_type;
      _max_time_rate_type max_time_rate;
      uint32_t max_position_error_length;
      typedef float _max_position_error_type;
      _max_position_error_type st_max_position_error;
      _max_position_error_type * max_position_error;
      uint32_t max_velocity_error_length;
      typedef float _max_velocity_error_type;
      _max_velocity_error_type st_max_velocity_error;
      _max_velocity_error_type * max_velocity_error;

    TrajectoryAnalysis():
      planned_duration(0),
      measured_duration(0),
      min_angle_command_length(0), min_angle_command(NULL),
      max_angle_command_length(0), max_angle_command(NULL),
      peak_speed_command_length(0), peak_speed_command(NULL),
      peak_accel_command_length(0), peak_accel_command(NULL),
      peak_jerk_command_length(0), peak_jerk_command(NULL),
      min_time_rate(0),
      max_time_rate(0),
      max_position_error_length(0), max_position_error(NULL),
      max_velocity_error_length(0), max_velocity_error(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->planned_duration);
      offset += serializeAvrFloat64(outbuffer + offset, this->measured_duration);
      *(outbuffer + offset + 0) = (this->min_angle_command_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->min_angle_command_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->min_angle_command_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->min_angle_command_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_angle_command_length);
      for( uint32_t i = 0; i < min_angle_command_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->min_angle_command[i]);
      }
      *(outbuffer + offset + 0) = (this->max_angle_command_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->max_angle_command_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->max_angle_command_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->max_angle_command_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_angle_command_length);
      for( uint32_t i = 0; i < max_angle_command_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->max_angle_command[i]);
      }
      *(outbuffer + offset + 0) = (this->peak_speed_command_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->peak_speed_command_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->peak_speed_command_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->peak_speed_command_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->peak_speed_command_length);
      for( uint32_t i = 0; i < peak_speed_command_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->peak_speed_command[i]);
      }
      *(outbuffer + offset + 0) = (this->peak_accel_command_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->peak_accel_command_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->peak_accel_command_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->peak_accel_command_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->peak_accel_command_length);
      for( uint32_t i = 0; i < peak_accel_command_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->peak_accel_command[i]);
      }
      *(outbuffer + offset + 0) = (this->peak_jerk_command_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->peak_jerk_command_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->peak_jerk_command_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->peak_jerk_command_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->peak_jerk_command_length);
      for( uint32_t i = 0; i < peak_jerk_command_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->peak_jerk_command[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->min_time_rate);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_time_rate);
      *(outbuffer + offset + 0) = (this->max_position_error_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->max_position_error_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->max_position_error_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->max_position_error_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_position_error_length);
      for( uint32_t i = 0; i < max_position_error_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->max_position_error[i]);
      }
      *(outbuffer + offset + 0) = (this->max_velocity_error_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->max_velocity_error_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->max_velocity_error_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->max_velocity_error_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_velocity_error_length);
      for( uint32_t i = 0; i < max_velocity_error_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->max_velocity_error[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->planned_duration));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->measured_duration));
      uint32_t min_angle_command_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      min_angle_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      min_angle_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      min_angle_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->min_angle_command_length);
      if(min_angle_command_lengthT > min_angle_command_length)
        this->min_angle_command = (float*)realloc(this->min_angle_command, min_angle_command_lengthT * sizeof(float));
      min_angle_command_length = min_angle_command_lengthT;
      for( uint32_t i = 0; i < min_angle_command_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_min_angle_command));
        memcpy( &(this->min_angle_command[i]), &(this->st_min_angle_command), sizeof(float));
      }
      uint32_t max_angle_command_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      max_angle_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      max_angle_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      max_angle_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->max_angle_command_length);
      if(max_angle_command_lengthT > max_angle_command_length)
        this->max_angle_command = (float*)realloc(this->max_angle_command, max_angle_command_lengthT * sizeof(float));
      max_angle_command_length = max_angle_command_lengthT;
      for( uint32_t i = 0; i < max_angle_command_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_max_angle_command));
        memcpy( &(this->max_angle_command[i]), &(this->st_max_angle_command), sizeof(float));
      }
      uint32_t peak_speed_command_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      peak_speed_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      peak_speed_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      peak_speed_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->peak_speed_command_length);
      if(peak_speed_command_lengthT > peak_speed_command_length)
        this->peak_speed_command = (float*)realloc(this->peak_speed_command, peak_speed_command_lengthT * sizeof(float));
      peak_speed_command_length = peak_speed_command_lengthT;
      for( uint32_t i = 0; i < peak_speed_command_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_peak_speed_command));
        memcpy( &(this->peak_speed_command[i]), &(this->st_peak_speed_command), sizeof(float));
      }
      uint32_t peak_accel_command_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      peak_accel_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      peak_accel_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      peak_accel_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->peak_accel_command_length);
      if(peak_accel_command_lengthT > peak_accel_command_length)
        this->peak_accel_command = (float*)realloc(this->peak_accel_command, peak_accel_command_lengthT * sizeof(float));
      peak_accel_command_length = peak_accel_command_lengthT;
      for( uint32_t i = 0; i < peak_accel_command_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_peak_accel_command));
        memcpy( &(this->peak_accel_command[i]), &(this->st_peak_accel_command), sizeof(float));
      }
      uint32_t peak_jerk_command_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      peak_jerk_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      peak_jerk_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      peak_jerk_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->peak_jerk_command_length);
      if(peak_jerk_command_lengthT > peak_jerk_command_length)
        this->peak_jerk_command = (float*)realloc(this->peak_jerk_command, peak_jerk_command_lengthT * sizeof(float));
      peak_jerk_command_length = peak_jerk_command_lengthT;
      for( uint32_t i = 0; i < peak_jerk_command_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_peak_jerk_command));
        memcpy( &(this->peak_jerk_command[i]), &(this->st_peak_jerk_command), sizeof(float));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->min_time_rate));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_time_rate));
      uint32_t max_position_error_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      max_position_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      max_position_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      max_position_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->max_position_error_length);
      if(max_position_error_lengthT > max_position_error_length)
        this->max_position_error = (float*)realloc(this->max_position_error, max_position_error_lengthT * sizeof(float));
      max_position_error_length = max_position_error_lengthT;
      for( uint32_t i = 0; i < max_position_error_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_max_position_error));
        memcpy( &(this->max_position_error[i]), &(this->st_max_position_error), sizeof(float));
      }
      uint32_t max_velocity_error_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      max_velocity_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      max_velocity_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      max_velocity_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->max_velocity_error_length);
      if(max_velocity_error_lengthT > max_velocity_error_length)
        this->max_velocity_error = (float*)realloc(this->max_velocity_error, max_velocity_error_lengthT * sizeof(float));
      max_velocity_error_length = max_velocity_error_lengthT;
      for( uint32_t i = 0; i < max_velocity_error_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_max_velocity_error));
        memcpy( &(this->max_velocity_error[i]), &(this->st_max_velocity_error), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "intera_motion_msgs/TrajectoryAnalysis"; };
    const char * getMD5(){ return "f30ec541413b4eb2cecc0d0af7d30ad4"; };

  };

}
#endif
