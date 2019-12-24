#ifndef _ROS_intera_motion_msgs_Waypoint_h
#define _ROS_intera_motion_msgs_Waypoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"
#include "intera_motion_msgs/WaypointOptions.h"

namespace intera_motion_msgs
{

  class Waypoint : public ros::Msg
  {
    public:
      uint32_t joint_positions_length;
      typedef float _joint_positions_type;
      _joint_positions_type st_joint_positions;
      _joint_positions_type * joint_positions;
      typedef const char* _active_endpoint_type;
      _active_endpoint_type active_endpoint;
      typedef geometry_msgs::PoseStamped _pose_type;
      _pose_type pose;
      typedef intera_motion_msgs::WaypointOptions _options_type;
      _options_type options;

    Waypoint():
      joint_positions_length(0), joint_positions(NULL),
      active_endpoint(""),
      pose(),
      options()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->joint_positions_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_positions_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_positions_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_positions_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_positions_length);
      for( uint32_t i = 0; i < joint_positions_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_positions[i]);
      }
      uint32_t length_active_endpoint = strlen(this->active_endpoint);
      varToArr(outbuffer + offset, length_active_endpoint);
      offset += 4;
      memcpy(outbuffer + offset, this->active_endpoint, length_active_endpoint);
      offset += length_active_endpoint;
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->options.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t joint_positions_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_positions_length);
      if(joint_positions_lengthT > joint_positions_length)
        this->joint_positions = (float*)realloc(this->joint_positions, joint_positions_lengthT * sizeof(float));
      joint_positions_length = joint_positions_lengthT;
      for( uint32_t i = 0; i < joint_positions_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_joint_positions));
        memcpy( &(this->joint_positions[i]), &(this->st_joint_positions), sizeof(float));
      }
      uint32_t length_active_endpoint;
      arrToVar(length_active_endpoint, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_active_endpoint; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_active_endpoint-1]=0;
      this->active_endpoint = (char *)(inbuffer + offset-1);
      offset += length_active_endpoint;
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->options.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "intera_motion_msgs/Waypoint"; };
    const char * getMD5(){ return "8284b290b22204acc5e4d8000467b033"; };

  };

}
#endif
