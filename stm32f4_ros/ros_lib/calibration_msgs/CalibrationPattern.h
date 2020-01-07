#ifndef _ROS_calibration_msgs_CalibrationPattern_h
#define _ROS_calibration_msgs_CalibrationPattern_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"

namespace calibration_msgs
{

  class CalibrationPattern : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t object_points_length;
      typedef geometry_msgs::Point _object_points_type;
      _object_points_type st_object_points;
      _object_points_type * object_points;
      uint32_t image_points_length;
      typedef geometry_msgs::Point _image_points_type;
      _image_points_type st_image_points;
      _image_points_type * image_points;
      typedef uint8_t _success_type;
      _success_type success;

    CalibrationPattern():
      header(),
      object_points_length(0), object_points(NULL),
      image_points_length(0), image_points(NULL),
      success(0)
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
      *(outbuffer + offset + 0) = (this->image_points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->image_points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->image_points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->image_points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->image_points_length);
      for( uint32_t i = 0; i < image_points_length; i++){
      offset += this->image_points[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->success >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
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
      uint32_t image_points_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      image_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      image_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      image_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->image_points_length);
      if(image_points_lengthT > image_points_length)
        this->image_points = (geometry_msgs::Point*)realloc(this->image_points, image_points_lengthT * sizeof(geometry_msgs::Point));
      image_points_length = image_points_lengthT;
      for( uint32_t i = 0; i < image_points_length; i++){
      offset += this->st_image_points.deserialize(inbuffer + offset);
        memcpy( &(this->image_points[i]), &(this->st_image_points), sizeof(geometry_msgs::Point));
      }
      this->success =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return "calibration_msgs/CalibrationPattern"; };
    const char * getMD5(){ return "5854af5462e19a169f68917c875a6238"; };

  };

}
#endif
