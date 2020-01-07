#ifndef _ROS_calibration_msgs_JointStateCalibrationPattern_h
#define _ROS_calibration_msgs_JointStateCalibrationPattern_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/JointState.h"

namespace calibration_msgs
{

  class JointStateCalibrationPattern : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t object_points_length;
      typedef geometry_msgs::Point _object_points_type;
      _object_points_type st_object_points;
      _object_points_type * object_points;
      uint32_t joint_points_length;
      typedef sensor_msgs::JointState _joint_points_type;
      _joint_points_type st_joint_points;
      _joint_points_type * joint_points;

    JointStateCalibrationPattern():
      header(),
      object_points_length(0), object_points(NULL),
      joint_points_length(0), joint_points(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->object_points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->object_points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->object_points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->object_points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->object_points_length);
      for( uint32_t i = 0; i < object_points_length; i++){
      offset += this->object_points[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->joint_points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_points_length);
      for( uint32_t i = 0; i < joint_points_length; i++){
      offset += this->joint_points[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t object_points_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      object_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      object_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      object_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->object_points_length);
      if(object_points_lengthT > object_points_length)
        this->object_points = (geometry_msgs::Point*)realloc(this->object_points, object_points_lengthT * sizeof(geometry_msgs::Point));
      object_points_length = object_points_lengthT;
      for( uint32_t i = 0; i < object_points_length; i++){
      offset += this->st_object_points.deserialize(inbuffer + offset);
        memcpy( &(this->object_points[i]), &(this->st_object_points), sizeof(geometry_msgs::Point));
      }
      uint32_t joint_points_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_points_length);
      if(joint_points_lengthT > joint_points_length)
        this->joint_points = (sensor_msgs::JointState*)realloc(this->joint_points, joint_points_lengthT * sizeof(sensor_msgs::JointState));
      joint_points_length = joint_points_lengthT;
      for( uint32_t i = 0; i < joint_points_length; i++){
      offset += this->st_joint_points.deserialize(inbuffer + offset);
        memcpy( &(this->joint_points[i]), &(this->st_joint_points), sizeof(sensor_msgs::JointState));
      }
     return offset;
    }

    const char * getType(){ return "calibration_msgs/JointStateCalibrationPattern"; };
    const char * getMD5(){ return "c80e9cf8e7942eba44a6d32e3c75bf59"; };

  };

}
#endif
