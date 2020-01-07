#ifndef _ROS_calibration_msgs_RobotMeasurement_h
#define _ROS_calibration_msgs_RobotMeasurement_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "calibration_msgs/CameraMeasurement.h"
#include "calibration_msgs/LaserMeasurement.h"
#include "calibration_msgs/ChainMeasurement.h"

namespace calibration_msgs
{

  class RobotMeasurement : public ros::Msg
  {
    public:
      typedef const char* _sample_id_type;
      _sample_id_type sample_id;
      typedef const char* _target_id_type;
      _target_id_type target_id;
      typedef const char* _chain_id_type;
      _chain_id_type chain_id;
      uint32_t M_cam_length;
      typedef calibration_msgs::CameraMeasurement _M_cam_type;
      _M_cam_type st_M_cam;
      _M_cam_type * M_cam;
      uint32_t M_laser_length;
      typedef calibration_msgs::LaserMeasurement _M_laser_type;
      _M_laser_type st_M_laser;
      _M_laser_type * M_laser;
      uint32_t M_chain_length;
      typedef calibration_msgs::ChainMeasurement _M_chain_type;
      _M_chain_type st_M_chain;
      _M_chain_type * M_chain;

    RobotMeasurement():
      sample_id(""),
      target_id(""),
      chain_id(""),
      M_cam_length(0), M_cam(NULL),
      M_laser_length(0), M_laser(NULL),
      M_chain_length(0), M_chain(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_sample_id = strlen(this->sample_id);
      varToArr(outbuffer + offset, length_sample_id);
      offset += 4;
      memcpy(outbuffer + offset, this->sample_id, length_sample_id);
      offset += length_sample_id;
      uint32_t length_target_id = strlen(this->target_id);
      varToArr(outbuffer + offset, length_target_id);
      offset += 4;
      memcpy(outbuffer + offset, this->target_id, length_target_id);
      offset += length_target_id;
      uint32_t length_chain_id = strlen(this->chain_id);
      varToArr(outbuffer + offset, length_chain_id);
      offset += 4;
      memcpy(outbuffer + offset, this->chain_id, length_chain_id);
      offset += length_chain_id;
      *(outbuffer + offset + 0) = (this->M_cam_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->M_cam_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->M_cam_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->M_cam_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->M_cam_length);
      for( uint32_t i = 0; i < M_cam_length; i++){
      offset += this->M_cam[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->M_laser_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->M_laser_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->M_laser_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->M_laser_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->M_laser_length);
      for( uint32_t i = 0; i < M_laser_length; i++){
      offset += this->M_laser[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->M_chain_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->M_chain_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->M_chain_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->M_chain_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->M_chain_length);
      for( uint32_t i = 0; i < M_chain_length; i++){
      offset += this->M_chain[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_sample_id;
      arrToVar(length_sample_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_sample_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_sample_id-1]=0;
      this->sample_id = (char *)(inbuffer + offset-1);
      offset += length_sample_id;
      uint32_t length_target_id;
      arrToVar(length_target_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_target_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_target_id-1]=0;
      this->target_id = (char *)(inbuffer + offset-1);
      offset += length_target_id;
      uint32_t length_chain_id;
      arrToVar(length_chain_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_chain_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_chain_id-1]=0;
      this->chain_id = (char *)(inbuffer + offset-1);
      offset += length_chain_id;
      uint32_t M_cam_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      M_cam_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      M_cam_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      M_cam_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->M_cam_length);
      if(M_cam_lengthT > M_cam_length)
        this->M_cam = (calibration_msgs::CameraMeasurement*)realloc(this->M_cam, M_cam_lengthT * sizeof(calibration_msgs::CameraMeasurement));
      M_cam_length = M_cam_lengthT;
      for( uint32_t i = 0; i < M_cam_length; i++){
      offset += this->st_M_cam.deserialize(inbuffer + offset);
        memcpy( &(this->M_cam[i]), &(this->st_M_cam), sizeof(calibration_msgs::CameraMeasurement));
      }
      uint32_t M_laser_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      M_laser_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      M_laser_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      M_laser_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->M_laser_length);
      if(M_laser_lengthT > M_laser_length)
        this->M_laser = (calibration_msgs::LaserMeasurement*)realloc(this->M_laser, M_laser_lengthT * sizeof(calibration_msgs::LaserMeasurement));
      M_laser_length = M_laser_lengthT;
      for( uint32_t i = 0; i < M_laser_length; i++){
      offset += this->st_M_laser.deserialize(inbuffer + offset);
        memcpy( &(this->M_laser[i]), &(this->st_M_laser), sizeof(calibration_msgs::LaserMeasurement));
      }
      uint32_t M_chain_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      M_chain_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      M_chain_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      M_chain_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->M_chain_length);
      if(M_chain_lengthT > M_chain_length)
        this->M_chain = (calibration_msgs::ChainMeasurement*)realloc(this->M_chain, M_chain_lengthT * sizeof(calibration_msgs::ChainMeasurement));
      M_chain_length = M_chain_lengthT;
      for( uint32_t i = 0; i < M_chain_length; i++){
      offset += this->st_M_chain.deserialize(inbuffer + offset);
        memcpy( &(this->M_chain[i]), &(this->st_M_chain), sizeof(calibration_msgs::ChainMeasurement));
      }
     return offset;
    }

    const char * getType(){ return "calibration_msgs/RobotMeasurement"; };
    const char * getMD5(){ return "fe22486c078efbf7892430dd0b99305c"; };

  };

}
#endif
