//
// Created by oolongmoon on 05/08/2022.
//

#ifndef CATKIN_WS_AVOID_H
#define CATKIN_WS_AVOID_H

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <turtlebot3_navigation/info.h>
#define PI 3.14159265358979323846
#define LIMIT 2 // the danger range
#define WAFFLE_MAX_LIN_VEL 0.26
#define WAFFLE_MAX_ANG_VEL 1.82
#define scan_time 2
#define expand_coe 100
#define Vcomf 0.2
// the parameters for algorithm
#define a 1.5
#define b 0.6
#define c 1.5
#define t2 3

class Avoid {
public:
  Avoid(char const *num);
  void moving();
  void angular_control();
  void velocity_control();
  void global_control();
  void get_basics();
  void get_obstacle_info();
  void calculate(const sensor_msgs::LaserScan::ConstPtr &msg);
  void counterCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
  double oriTrans(const geometry_msgs::Quaternion msg);
  struct obstacle {
    float subscript;
    float low;
  } ob[2];
  int count;
  int detect;
  float td;  // time_deriviative
  float tti; // time to interact
  float t1;  // threshold
  float dis; // obstacle distance
  float ang; // angle with obstacle
  std::string topic;
  geometry_msgs::Twist vel;
  sensor_msgs::LaserScan scan;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::ServiceClient state_client;
  ros::ServiceClient global_client;
  ros::NodeHandle nh;
};

#endif