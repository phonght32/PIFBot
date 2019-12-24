#ifndef _ROS_pcl_msg_Line_h
#define _ROS_pcl_msg_Line_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pcl_msg/Point.h"

namespace pcl_msg
{

  class Line : public ros::Msg
  {
    public:
      typedef uint16_t _numpoints_type;
      _numpoints_type numpoints;
      uint32_t points_length;
      typedef pcl_msg::Point _points_type;
      _points_type st_points;
      _points_type * points;

    Line():
      numpoints(0),
      points_length(0), points(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->numpoints >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->numpoints >> (8 * 1)) & 0xFF;
      offset += sizeof(this->numpoints);
      *(outbuffer + offset + 0) = (this->points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->points_length);
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->points[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->numpoints =  ((uint16_t) (*(inbuffer + offset)));
      this->numpoints |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->numpoints);
      uint32_t points_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->points_length);
      if(points_lengthT > points_length)
        this->points = (pcl_msg::Point*)realloc(this->points, points_lengthT * sizeof(pcl_msg::Point));
      points_length = points_lengthT;
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->st_points.deserialize(inbuffer + offset);
        memcpy( &(this->points[i]), &(this->st_points), sizeof(pcl_msg::Point));
      }
     return offset;
    }

    const char * getType(){ return "pcl_msg/Line"; };
    const char * getMD5(){ return "ce61593350d0c9ffeeb7e9c8155aae2b"; };

  };

}
#endif
