#ifndef _ROS_calibration_msgs_DenseLaserObjectFeatures_h
#define _ROS_calibration_msgs_DenseLaserObjectFeatures_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "calibration_msgs/DenseLaserPoint.h"
#include "geometry_msgs/Point.h"

namespace calibration_msgs
{

  class DenseLaserObjectFeatures : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t dense_laser_points_length;
      typedef calibration_msgs::DenseLaserPoint _dense_laser_points_type;
      _dense_laser_points_type st_dense_laser_points;
      _dense_laser_points_type * dense_laser_points;
      uint32_t object_points_length;
      typedef geometry_msgs::Point _object_points_type;
      _object_points_type st_object_points;
      _object_points_type * object_points;
      typedef uint8_t _success_type;
      _success_type success;

    DenseLaserObjectFeatures():
      header(),
      dense_laser_points_length(0), dense_laser_points(NULL),
      object_points_length(0), object_points(NULL),
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->dense_laser_points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dense_laser_points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dense_laser_points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dense_laser_points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dense_laser_points_length);
      for( uint32_t i = 0; i < dense_laser_points_length; i++){
      offset += this->dense_laser_points[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->object_points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->object_points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->object_points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->object_points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->object_points_length);
      for( uint32_t i = 0; i < object_points_length; i++){
      offset += this->object_points[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->success >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t dense_laser_points_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      dense_laser_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      dense_laser_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      dense_laser_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->dense_laser_points_length);
      if(dense_laser_points_lengthT > dense_laser_points_length)
        this->dense_laser_points = (calibration_msgs::DenseLaserPoint*)realloc(this->dense_laser_points, dense_laser_points_lengthT * sizeof(calibration_msgs::DenseLaserPoint));
      dense_laser_points_length = dense_laser_points_lengthT;
      for( uint32_t i = 0; i < dense_laser_points_length; i++){
      offset += this->st_dense_laser_points.deserialize(inbuffer + offset);
        memcpy( &(this->dense_laser_points[i]), &(this->st_dense_laser_points), sizeof(calibration_msgs::DenseLaserPoint));
      }
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
      this->success =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return "calibration_msgs/DenseLaserObjectFeatures"; };
    const char * getMD5(){ return "b642d46e47d54e00f98a3d98b02b5cc6"; };

  };

}
#endif
