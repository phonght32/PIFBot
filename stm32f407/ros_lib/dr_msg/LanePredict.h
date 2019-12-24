#ifndef _ROS_dr_msg_LanePredict_h
#define _ROS_dr_msg_LanePredict_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "dr_msg/Line.h"

namespace dr_msg
{

  class LanePredict : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _numlines_type;
      _numlines_type numlines;
      uint32_t lines_length;
      typedef dr_msg::Line _lines_type;
      _lines_type st_lines;
      _lines_type * lines;

    LanePredict():
      header(),
      numlines(0),
      lines_length(0), lines(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->numlines >> (8 * 0)) & 0xFF;
      offset += sizeof(this->numlines);
      *(outbuffer + offset + 0) = (this->lines_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->lines_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->lines_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->lines_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lines_length);
      for( uint32_t i = 0; i < lines_length; i++){
      offset += this->lines[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->numlines =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->numlines);
      uint32_t lines_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      lines_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      lines_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      lines_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->lines_length);
      if(lines_lengthT > lines_length)
        this->lines = (dr_msg::Line*)realloc(this->lines, lines_lengthT * sizeof(dr_msg::Line));
      lines_length = lines_lengthT;
      for( uint32_t i = 0; i < lines_length; i++){
      offset += this->st_lines.deserialize(inbuffer + offset);
        memcpy( &(this->lines[i]), &(this->st_lines), sizeof(dr_msg::Line));
      }
     return offset;
    }

    const char * getType(){ return "dr_msg/LanePredict"; };
    const char * getMD5(){ return "55bcba72a234f1eec4107aa58d0c0d22"; };

  };

}
#endif
