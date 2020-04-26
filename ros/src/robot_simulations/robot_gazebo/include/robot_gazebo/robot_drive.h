#ifndef ROBOT_DRIVE_H_
#define ROBOT_DRIVE_H_

#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#define DEG2RAD (M_PI/180)
#define RAD2DEF (180/M_PI)

#define CENTER  0
#define LEFT    1
#define RIGHT   2

#define LINEAR_VELOCITY   0.3
#define ANGULAR_VELOCITY  1.5

#define GET_ROBOT_DIRECTION   0
#define ROBOT_DRIVE_FORWARD   1
#define ROBOT_RIGHT_TURN      2
#define ROBOT_LEFT_TURN       3

class RobotDrive
{
public:
    RobotDrive();
    ~RobotDrive();
    bool init();
    bool controlLoop();

private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;

    ros::Publisher cmd_vel_pub;

    ros::Subscriber laser_scan_sub;
    ros::Subscriber odom_sub;

    double escape_range;
    double check_forward_dist;
    double check_side_dist;

    double scan_data[3] = {0.0, 0.0, 0.0};

    double robot_pose;
    double prev_robot_pose;

    void updateCommandVelocity(double linear, double angular);
    void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
    void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
};



#endif /* ROBOT_DRIVE_H_ */