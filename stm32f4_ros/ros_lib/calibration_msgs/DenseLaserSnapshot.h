#ifndef _ROS_calibration_msgs_DenseLaserSnapshot_h
#define _ROS_calibration_msgs_DenseLaserSnapshot_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/time.h"

namespace calibration_msgs
{

  class DenseLaserSnapshot : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _angle_min_type;
      _angle_min_type angle_min;
      typedef float _angle_max_type;
      _angle_max_type angle_max;
      typedef float _angle_increment_type;
      _angle_increment_type angle_increment;
      typedef float _time_increment_type;
      _time_increment_type time_increment;
      typedef float _range_min_type;
      _range_min_type range_min;
      typedef float _range_max_type;
      _range_max_type range_max;
      typedef uint32_t _readings_per_scan_type;
      _readings_per_scan_type readings_per_scan;
      typedef uint32_t _num_scans_type;
      _num_scans_type num_scans;
      uint32_t ranges_length;
      typedef float _ranges_type;
      _ranges_type st_ranges;
      _ranges_type * ranges;
      uint32_t intensities_length;
      typedef float _intensities_type;
      _intensities_type st_intensities;
      _intensities_type * intensities;
      uint32_t scan_start_length;
      typedef ros::Time _scan_start_type;
      _scan_start_type st_scan_start;
      _scan_start_type * scan_start;

    DenseLaserSnapshot():
      header(),
      angle_min(0),
      angle_max(0),
      angle_increment(0),
      time_increment(0),
      range_min(0),
      range_max(0),
      readings_per_scan(0),
      num_scans(0),
      ranges_length(0), ranges(NULL),
      intensities_length(0), intensities(NULL),
      scan_start_length(0), scan_start(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_angle_min;
      u_angle_min.real = this->angle_min;
      *(outbuffer + offset + 0) = (u_angle_min.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_min.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_min.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_min.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_min);
      union {
        float real;
        uint32_t base;
      } u_angle_max;
      u_angle_max.real = this->angle_max;
      *(outbuffer + offset + 0) = (u_angle_max.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_max.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_max.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_max.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_max);
      union {
        float real;
        uint32_t base;
      } u_angle_increment;
      u_angle_increment.real = this->angle_increment;
      *(outbuffer + offset + 0) = (u_angle_increment.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_increment.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_increment.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_increment.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_increment);
      union {
        float real;
        uint32_t base;
      } u_time_increment;
      u_time_increment.real = this->time_increment;
      *(outbuffer + offset + 0) = (u_time_increment.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_time_increment.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_time_increment.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_time_increment.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_increment);
      union {
        float real;
        uint32_t base;
      } u_range_min;
      u_range_min.real = this->range_min;
      *(outbuffer + offset + 0) = (u_range_min.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_range_min.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_range_min.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_range_min.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->range_min);
      union {
        float real;
        uint32_t base;
      } u_range_max;
      u_range_max.real = this->range_max;
      *(outbuffer + offset + 0) = (u_range_max.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_range_max.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_range_max.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_range_max.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->range_max);
      *(outbuffer + offset + 0) = (this->readings_per_scan >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->readings_per_scan >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->readings_per_scan >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->readings_per_scan >> (8 * 3)) & 0xFF;
      offset += sizeof(this->readings_per_scan);
      *(outbuffer + offset + 0) = (this->num_scans >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->num_scans >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->num_scans >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->num_scans >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_scans);
      *(outbuffer + offset + 0) = (this->ranges_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ranges_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ranges_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ranges_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ranges_length);
      for( uint32_t i = 0; i < ranges_length; i++){
      union {
        float real;
        uint32_t base;
      } u_rangesi;
      u_rangesi.real = this->ranges[i];
      *(outbuffer + offset + 0) = (u_rangesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rangesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rangesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rangesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ranges[i]);
      }
      *(outbuffer + offset + 0) = (this->intensities_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->intensities_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->intensities_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->intensities_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->intensities_length);
      for( uint32_t i = 0; i < intensities_length; i++){
      union {
        float real;
        uint32_t base;
      } u_intensitiesi;
      u_intensitiesi.real = this->intensities[i];
      *(outbuffer + offset + 0) = (u_intensitiesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_intensitiesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_intensitiesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_intensitiesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->intensities[i]);
      }
      *(outbuffer + offset + 0) = (this->scan_start_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->scan_start_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->scan_start_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->scan_start_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->scan_start_length);
      for( uint32_t i = 0; i < scan_start_length; i++){
      *(outbuffer + offset + 0) = (this->scan_start[i].sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->scan_start[i].sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->scan_start[i].sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->scan_start[i].sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->scan_start[i].sec);
      *(outbuffer + offset + 0) = (this->scan_start[i].nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->scan_start[i].nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->scan_start[i].nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->scan_start[i].nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->scan_start[i].nsec);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_angle_min;
      u_angle_min.base = 0;
      u_angle_min.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_min.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_min.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_min.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle_min = u_angle_min.real;
      offset += sizeof(this->angle_min);
      union {
        float real;
        uint32_t base;
      } u_angle_max;
      u_angle_max.base = 0;
      u_angle_max.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_max.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_max.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_max.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle_max = u_angle_max.real;
      offset += sizeof(this->angle_max);
      union {
        float real;
        uint32_t base;
      } u_angle_increment;
      u_angle_increment.base = 0;
      u_angle_increment.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_increment.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_increment.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_increment.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle_increment = u_angle_increment.real;
      offset += sizeof(this->angle_increment);
      union {
        float real;
        uint32_t base;
      } u_time_increment;
      u_time_increment.base = 0;
      u_time_increment.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_time_increment.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_time_increment.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_time_increment.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->time_increment = u_time_increment.real;
      offset += sizeof(this->time_increment);
      union {
        float real;
        uint32_t base;
      } u_range_min;
      u_range_min.base = 0;
      u_range_min.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_range_min.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_range_min.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_range_min.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->range_min = u_range_min.real;
      offset += sizeof(this->range_min);
      union {
        float real;
        uint32_t base;
      } u_range_max;
      u_range_max.base = 0;
      u_range_max.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_range_max.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_range_max.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_range_max.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->range_max = u_range_max.real;
      offset += sizeof(this->range_max);
      this->readings_per_scan =  ((uint32_t) (*(inbuffer + offset)));
      this->readings_per_scan |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->readings_per_scan |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->readings_per_scan |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->readings_per_scan);
      this->num_scans =  ((uint32_t) (*(inbuffer + offset)));
      this->num_scans |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->num_scans |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->num_scans |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->num_scans);
      uint32_t ranges_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ranges_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ranges_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ranges_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ranges_length);
      if(ranges_lengthT > ranges_length)
        this->ranges = (float*)realloc(this->ranges, ranges_lengthT * sizeof(float));
      ranges_length = ranges_lengthT;
      for( uint32_t i = 0; i < ranges_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_ranges;
      u_st_ranges.base = 0;
      u_st_ranges.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_ranges.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_ranges.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_ranges.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_ranges = u_st_ranges.real;
      offset += sizeof(this->st_ranges);
        memcpy( &(this->ranges[i]), &(this->st_ranges), sizeof(float));
      }
      uint32_t intensities_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      intensities_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      intensities_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      intensities_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->intensities_length);
      if(intensities_lengthT > intensities_length)
        this->intensities = (float*)realloc(this->intensities, intensities_lengthT * sizeof(float));
      intensities_length = intensities_lengthT;
      for( uint32_t i = 0; i < intensities_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_intensities;
      u_st_intensities.base = 0;
      u_st_intensities.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_intensities.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_intensities.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_intensities.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_intensities = u_st_intensities.real;
      offset += sizeof(this->st_intensities);
        memcpy( &(this->intensities[i]), &(this->st_intensities), sizeof(float));
      }
      uint32_t scan_start_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      scan_start_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      scan_start_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      scan_start_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->scan_start_length);
      if(scan_start_lengthT > scan_start_length)
        this->scan_start = (ros::Time*)realloc(this->scan_start, scan_start_lengthT * sizeof(ros::Time));
      scan_start_length = scan_start_lengthT;
      for( uint32_t i = 0; i < scan_start_length; i++){
      this->st_scan_start.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->st_scan_start.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_scan_start.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_scan_start.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_scan_start.sec);
      this->st_scan_start.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->st_scan_start.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_scan_start.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_scan_start.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_scan_start.nsec);
        memcpy( &(this->scan_start[i]), &(this->st_scan_start), sizeof(ros::Time));
      }
     return offset;
    }

    const char * getType(){ return "calibration_msgs/DenseLaserSnapshot"; };
    const char * getMD5(){ return "deb2810d3530db3484f886a81243195d"; };

  };

}
#endif
