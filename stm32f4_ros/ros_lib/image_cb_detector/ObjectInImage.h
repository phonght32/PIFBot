#ifndef _ROS_image_cb_detector_ObjectInImage_h
#define _ROS_image_cb_detector_ObjectInImage_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"
#include "image_cb_detector/ImagePoint.h"

namespace image_cb_detector
{

  class ObjectInImage : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t model_points_length;
      typedef geometry_msgs::Point _model_points_type;
      _model_points_type st_model_points;
      _model_points_type * model_points;
      uint32_t image_points_length;
      typedef image_cb_detector::ImagePoint _image_points_type;
      _image_points_type st_image_points;
      _image_points_type * image_points;

    ObjectInImage():
      header(),
      model_points_length(0), model_points(NULL),
      image_points_length(0), image_points(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->model_points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->model_points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->model_points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->model_points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->model_points_length);
      for( uint32_t i = 0; i < model_points_length; i++){
      offset += this->model_points[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->image_points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->image_points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->image_points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->image_points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->image_points_length);
      for( uint32_t i = 0; i < image_points_length; i++){
      offset += this->image_points[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t model_points_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      model_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      model_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      model_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->model_points_length);
      if(model_points_lengthT > model_points_length)
        this->model_points = (geometry_msgs::Point*)realloc(this->model_points, model_points_lengthT * sizeof(geometry_msgs::Point));
      model_points_length = model_points_lengthT;
      for( uint32_t i = 0; i < model_points_length; i++){
      offset += this->st_model_points.deserialize(inbuffer + offset);
        memcpy( &(this->model_points[i]), &(this->st_model_points), sizeof(geometry_msgs::Point));
      }
      uint32_t image_points_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      image_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      image_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      image_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->image_points_length);
      if(image_points_lengthT > image_points_length)
        this->image_points = (image_cb_detector::ImagePoint*)realloc(this->image_points, image_points_lengthT * sizeof(image_cb_detector::ImagePoint));
      image_points_length = image_points_lengthT;
      for( uint32_t i = 0; i < image_points_length; i++){
      offset += this->st_image_points.deserialize(inbuffer + offset);
        memcpy( &(this->image_points[i]), &(this->st_image_points), sizeof(image_cb_detector::ImagePoint));
      }
     return offset;
    }

    const char * getType(){ return "image_cb_detector/ObjectInImage"; };
    const char * getMD5(){ return "0996b0d8499882526b533fe6e96aa418"; };

  };

}
#endif
