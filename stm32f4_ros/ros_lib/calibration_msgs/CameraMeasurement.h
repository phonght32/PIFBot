#ifndef _ROS_calibration_msgs_CameraMeasurement_h
#define _ROS_calibration_msgs_CameraMeasurement_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "calibration_msgs/CalibrationPattern.h"

namespace calibration_msgs
{

  class CameraMeasurement : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _camera_id_type;
      _camera_id_type camera_id;
      uint32_t image_points_length;
      typedef geometry_msgs::Point _image_points_type;
      _image_points_type st_image_points;
      _image_points_type * image_points;
      typedef sensor_msgs::CameraInfo _cam_info_type;
      _cam_info_type cam_info;
      typedef bool _verbose_type;
      _verbose_type verbose;
      typedef sensor_msgs::Image _image_type;
      _image_type image;
      typedef sensor_msgs::Image _image_rect_type;
      _image_rect_type image_rect;
      typedef calibration_msgs::CalibrationPattern _features_type;
      _features_type features;

    CameraMeasurement():
      header(),
      camera_id(""),
      image_points_length(0), image_points(NULL),
      cam_info(),
      verbose(0),
      image(),
      image_rect(),
      features()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_camera_id = strlen(this->camera_id);
      varToArr(outbuffer + offset, length_camera_id);
      offset += 4;
      memcpy(outbuffer + offset, this->camera_id, length_camera_id);
      offset += length_camera_id;
      *(outbuffer + offset + 0) = (this->image_points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->image_points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->image_points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->image_points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->image_points_length);
      for( uint32_t i = 0; i < image_points_length; i++){
      offset += this->image_points[i].serialize(outbuffer + offset);
      }
      offset += this->cam_info.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_verbose;
      u_verbose.real = this->verbose;
      *(outbuffer + offset + 0) = (u_verbose.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->verbose);
      offset += this->image.serialize(outbuffer + offset);
      offset += this->image_rect.serialize(outbuffer + offset);
      offset += this->features.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_camera_id;
      arrToVar(length_camera_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_camera_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_camera_id-1]=0;
      this->camera_id = (char *)(inbuffer + offset-1);
      offset += length_camera_id;
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
      offset += this->cam_info.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_verbose;
      u_verbose.base = 0;
      u_verbose.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->verbose = u_verbose.real;
      offset += sizeof(this->verbose);
      offset += this->image.deserialize(inbuffer + offset);
      offset += this->image_rect.deserialize(inbuffer + offset);
      offset += this->features.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "calibration_msgs/CameraMeasurement"; };
    const char * getMD5(){ return "f7a0cca96cdd8e17d1424338e086252f"; };

  };

}
#endif
