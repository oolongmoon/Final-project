#ifndef CATKIN_WS_GLOBAL_H
#define CATKIN_WS_GLOBAL_H

#include "ros/forwards.h"
#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <gazebo_msgs/GetModelState.h>
#include <sensor_msgs/LaserScan.h>
#include "tf/transform_datatypes.h"
#include "std_srvs/Empty.h"
#include <string>
#include <turtlebot3_navigation/info.h>

#define PI 3.14159265358979323846

class Global
{
public:
    geometry_msgs::Point goal;
    geometry_msgs::Point pos;
    Global();
    bool callback(turtlebot3_navigation::info::Request &req,
                      turtlebot3_navigation::info::Response &res);
//    void goal_counterCallback(const move_base_msgs::MoveBaseGoal &msg);
//    void pos_counterCallback(const geometry_msgs::PoseWithCovariance &msg);
//    geometry_msgs::Vector3 oriTrans(const geometry_msgs::Quaternion msg);
    void calculate();
    ros::NodeHandle nh;
    float ang;
    float dis;
};

#endif