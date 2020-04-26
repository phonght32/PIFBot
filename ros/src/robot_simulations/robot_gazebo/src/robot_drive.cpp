#include "robot_gazebo/robot_drive.h"

RobotDrive::RobotDrive() : nh_priv("~")
{
    ROS_INFO("robot simulation node init");
    ROS_ASSERT(init());
}

RobotDrive::~RobotDrive()
{
    updateCommandVelocity(0.0, 0.0);
    ros::shutdown();
}

bool RobotDrive::init()
{
    std::string cmd_vel_topic_name = nh.param<std::string>("cmd_vel_topic_name","");

    escape_range = 30.0*DEG2RAD;
    check_forward_dist = 0.7;
    check_side_dist = 0.6;

    robot_pose = 0.0;
    prev_robot_pose = 0.0;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);
    laser_scan_sub = nh.subscribe("scan", 10, &RobotDrive::laserScanMsgCallBack, this);
    odom_sub = nh.subscribe("odom", 10, &RobotDrive::odomMsgCallBack, this);

    return true;
}

void RobotDrive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

    robot_pose = atan2(siny, cosy);
}

void RobotDrive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    uint16_t scan_angle[3] = {0, 30, 330};
    for (int num = 0; num < 3; num++)
    {
        if (std::isinf(msg->ranges.at(scan_angle[num])))
        {
            scan_data[num] = msg->range_max;
        }
        else
        {
            scan_data[num] = msg->ranges.at(scan_angle[num]);
        }
    }
}

void RobotDrive::updateCommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub.publish(cmd_vel);
}

bool RobotDrive::controlLoop()
{
    static uint8_t robot_state_num = 0;

    switch(robot_state_num)
    {
        case GET_ROBOT_DIRECTION:
            if (scan_data[CENTER] > check_forward_dist)
            {
                if(scan_data[LEFT] < check_side_dist)
                {
                    prev_robot_pose = robot_pose;
                    robot_state_num = ROBOT_RIGHT_TURN;
                }
                else if (scan_data[RIGHT] < check_side_dist)
                {
                    prev_robot_pose = robot_pose;
                    robot_state_num = ROBOT_LEFT_TURN;
                }
                else 
                {
                    robot_state_num = ROBOT_DRIVE_FORWARD;
                }
            }

            if (scan_data[CENTER] < check_forward_dist)
            {
                prev_robot_pose = robot_pose;
                robot_state_num = ROBOT_RIGHT_TURN;
            }
            break;
        case ROBOT_DRIVE_FORWARD:
            updateCommandVelocity(LINEAR_VELOCITY, 0.0);
            robot_state_num = GET_ROBOT_DIRECTION;
            break;

        case ROBOT_RIGHT_TURN:
            if(fabs(prev_robot_pose - robot_pose) >= escape_range)
                robot_state_num = GET_ROBOT_DIRECTION;
            else
                updateCommandVelocity(0.0, -1*ANGULAR_VELOCITY);
            break;
        case ROBOT_LEFT_TURN:
            if(fabs(prev_robot_pose - robot_pose) >= escape_range)
                robot_state_num = GET_ROBOT_DIRECTION;
            else
                updateCommandVelocity(0.0, ANGULAR_VELOCITY);
            break;
        default:
            robot_state_num = GET_ROBOT_DIRECTION;
            break;
    }

    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_drive");
    RobotDrive robot_drive;

    ros::Rate loop_rate(125);

    while(ros::ok())
    {
        robot_drive.controlLoop();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}