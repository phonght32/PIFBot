#ifndef _ROS_calibration_msgs_DenseLaserPoint_h
#define _ROS_calibration_msgs_DenseLaserPoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace calibration_msgs
{

  class DenseLaserPoint : public ros::Msg
  {
    public:
      typedef float _scan_type;
      _scan_type scan;
      typedef float _ray_type;
      _ray_type ray;

    DenseLaserPoint():
      scan(0),
      ray(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->scan);
      offset += serializeAvrFloat64(outbuffer + offset, this->ray);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->scan));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ray));
     return offset;
    }

    const char * getType(){ return "calibration_msgs/DenseLaserPoint"; };
    const char * getMD5(){ return "12821677bc3daf8fabbb485d5b0cc027"; };

  };

}
#endif
