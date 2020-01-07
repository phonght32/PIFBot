#ifndef _ROS_calibration_msgs_LaserMeasurement_h
#define _ROS_calibration_msgs_LaserMeasurement_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/JointState.h"
#include "calibration_msgs/DenseLaserSnapshot.h"
#include "sensor_msgs/Image.h"
#include "calibration_msgs/CalibrationPattern.h"
#include "calibration_msgs/JointStateCalibrationPattern.h"

namespace calibration_msgs
{

  class LaserMeasurement : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _laser_id_type;
      _laser_id_type laser_id;
      uint32_t joint_points_length;
      typedef sensor_msgs::JointState _joint_points_type;
      _joint_points_type st_joint_points;
      _joint_points_type * joint_points;
      typedef bool _verbose_type;
      _verbose_type verbose;
      typedef calibration_msgs::DenseLaserSnapshot _snapshot_type;
      _snapshot_type snapshot;
      typedef sensor_msgs::Image _laser_image_type;
      _laser_image_type laser_image;
      typedef calibration_msgs::CalibrationPattern _image_features_type;
      _image_features_type image_features;
      typedef calibration_msgs::JointStateCalibrationPattern _joint_features_type;
      _joint_features_type joint_features;

    LaserMeasurement():
      header(),
      laser_id(""),
      joint_points_length(0), joint_points(NULL),
      verbose(0),
      snapshot(),
      laser_image(),
      image_features(),
      joint_features()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_laser_id = strlen(this->laser_id);
      varToArr(outbuffer + offset, length_laser_id);
      offset += 4;
      memcpy(outbuffer + offset, this->laser_id, length_laser_id);
      offset += length_laser_id;
      *(outbuffer + offset + 0) = (this->joint_points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_points_length);
      for( uint32_t i = 0; i < joint_points_length; i++){
      offset += this->joint_points[i].serialize(outbuffer + offset);
      }
      union {
        bool real;
        uint8_t base;
      } u_verbose;
      u_verbose.real = this->verbose;
      *(outbuffer + offset + 0) = (u_verbose.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->verbose);
      offset += this->snapshot.serialize(outbuffer + offset);
      offset += this->laser_image.serialize(outbuffer + offset);
      offset += this->image_features.serialize(outbuffer + offset);
      offset += this->joint_features.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_laser_id;
      arrToVar(length_laser_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_laser_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_laser_id-1]=0;
      this->laser_id = (char *)(inbuffer + offset-1);
      offset += length_laser_id;
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
      union {
        bool real;
        uint8_t base;
      } u_verbose;
      u_verbose.base = 0;
      u_verbose.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->verbose = u_verbose.real;
      offset += sizeof(this->verbose);
      offset += this->snapshot.deserialize(inbuffer + offset);
      offset += this->laser_image.deserialize(inbuffer + offset);
      offset += this->image_features.deserialize(inbuffer + offset);
      offset += this->joint_features.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "calibration_msgs/LaserMeasurement"; };
    const char * getMD5(){ return "7fa7e818b1234a443aa5d8e315175d17"; };

  };

}
#endif
