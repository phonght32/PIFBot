#ifndef _ROS_SERVICE_GetBinaryFile_h
#define _ROS_SERVICE_GetBinaryFile_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace file_server
{

static const char GETBINARYFILE[] = "file_server/GetBinaryFile";

  class GetBinaryFileRequest : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;

    GetBinaryFileRequest():
      name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
     return offset;
    }

    const char * getType(){ return GETBINARYFILE; };
    const char * getMD5(){ return "c1f3d28f1b044c871e6eff2e9fc3c667"; };

  };

  class GetBinaryFileResponse : public ros::Msg
  {
    public:
      uint32_t value_length;
      typedef uint8_t _value_type;
      _value_type st_value;
      _value_type * value;

    GetBinaryFileResponse():
      value_length(0), value(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->value_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->value_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->value_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->value_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value_length);
      for( uint32_t i = 0; i < value_length; i++){
      *(outbuffer + offset + 0) = (this->value[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->value[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t value_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      value_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      value_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      value_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->value_length);
      if(value_lengthT > value_length)
        this->value = (uint8_t*)realloc(this->value, value_lengthT * sizeof(uint8_t));
      value_length = value_lengthT;
      for( uint32_t i = 0; i < value_length; i++){
      this->st_value =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_value);
        memcpy( &(this->value[i]), &(this->st_value), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return GETBINARYFILE; };
    const char * getMD5(){ return "ceebcd87b651833e04cec03ceb23abb1"; };

  };

  class GetBinaryFile {
    public:
    typedef GetBinaryFileRequest Request;
    typedef GetBinaryFileResponse Response;
  };

}
#endif
