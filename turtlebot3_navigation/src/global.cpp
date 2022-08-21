#include "global.h"

Global::Global() {
  /*
    ros::Subscriber sub_goal = nh.subscribe("/move_base_simple/goal", 1000,
                                            &Global::goal_counterCallback,
    this); ros::Subscriber sub_pos = nh.subscribe("/amcl_pose", 1000,
    &Global::pos_counterCallback, this);
          */
  ros::ServiceServer gv =
      nh.advertiseService("/Global", &Global::callback,
                          this); // create the Service called // my_service
                                 // the defined // callback
  ros::spin();
}

void Global::calculate() {
  float x1 = pos.x;
  float y1 = pos.y;
  float z1 = pos.z + PI;
  float x2 = goal.x;
  float y2 = goal.y;
  ROS_INFO("x1: %f, x2: %f, y1: %f, y2: %f, z1: %f", x1, x2, y1, y2, z1);
  dis = pow(pow(y2 - y1, 2) + pow(x2 - x1, 2), 0.5);

  ang = atan(fabs(x2 - x1) / fabs(y2 - y1));
  ROS_INFO("before trans: %f", ang);
  if (x2 > x1) {
    if (y2 > y1) {
      ang = PI * 3 / 2 - ang;
    } else if (y2 < y1) {
      ang = PI / 2 + ang;
    } else {
      ang = PI;
    }
  } else if (x2 < x1) {
    if (y2 > y1) {
      ang = PI * 3 / 2 + ang;
    } else if (y2 < y1) {
      ang = PI / 2 - ang;
    } else {
      ang = PI * 2;
    }
  } else {
    if (y2 > y1) {
      ang = PI * 3 / 2;
    } else {
      ang = PI / 2;
    }
  }
  ROS_INFO("after trans: %f", ang);
  if (fabs(z1 - ang) > PI) { // the angle will be smaller turn another way
    if ((z1 - ang) > 0) {    // turn left
      ang = 2 * PI - fabs(z1 - ang);
    } else { // turn right
      ang = fabs(z1 - ang) - 2 * PI;
    }
  } else {
    if ((z1 - ang) > 0) { // turn right
      ang = -fabs(z1 - ang);
    } else { // turn left
      ang = fabs(z1 - ang);
    }
  }
  ROS_INFO("angle difference: %f", ang);
}

bool Global::callback(turtlebot3_navigation::info::Request &req,
                      turtlebot3_navigation::info::Response &res) {
  // res.some_variable = req.some_variable + req.other_variable;
  ROS_INFO("The angle is %f, the distance is %f", ang, dis);
  goal = req.goal;
  pos = req.pos;
  calculate();
  res.distance = dis;
  res.angle = ang;
  ROS_INFO("The angle is %f, the distance is %f", ang,
           dis); // We print an string whenever the
                 // Service gets called
  return true;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "global_server");
  Global global;
  return 0;
}

/*
void Global::goal_counterCallback(const move_base_msgs::MoveBaseGoal &msg) {
  goal = msg;
}
void Global::pos_counterCallback(const geometry_msgs::PoseWithCovariance &msg) {
  pos = msg;
  ROS_INFO("%f", msg.pose.position.x);
}
geometry_msgs::Vector3 Global::oriTrans(const geometry_msgs::Quaternion msg) {
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  geometry_msgs::Vector3 ori;
  ori.x = roll;
  ori.y = pitch;
  ori.z = yaw;
  return ori;
}
void Global::calculate() {
  float x1 = pos.pose.position.x;
  float y1 = pos.pose.position.y;
  float x2 = goal.target_pose.pose.position.x;
  float y2 = goal.target_pose.pose.position.y;
  ROS_INFO("x1: %f, x2: %f, y1: %f, y2: %f", x1, x2, y1, y2);
  dis = pow(pow(y2 - y1, 2) + pow(x2 - x1, 2), 0.5);

  geometry_msgs::Vector3 pos_ori = oriTrans(pos.pose.orientation);
  float ang = atan(fabs(y2 - y1) / fabs(x2 - x1));
  if (x2 > x1) {
    if (y2 > y1) {
      ang = PI / 2 + ang;
    } else if (y2 < y1) {
      ang = PI / 2 - ang;
    } else {
      ang = PI / 2;
    }
  } else if (x2 < x1) {
    if (y2 > y1) {
      ang = PI * 3 / 2 - ang;
    } else if (y2 < y1) {
      ang = PI * 3 / 2 + ang;
    } else {
      ang = PI * 3 / 2;
    }
  } else {
    if (y2 > y1) {
      ang = PI;
    } else {
      ang = 2 * PI;
    }
  }
  if (fabs(pos_ori.z + PI - ang) >
      PI) { // the angle will be smaller turn another way
    if ((pos_ori.z + PI - ang) > 0) { // turn left
      ang = 2 * PI - fabs(pos_ori.z + PI - ang);
    } else { // turn right
      ang = fabs(pos_ori.z + PI - ang) - 2 * PI;
    }
  } else {
    if ((pos_ori.z + PI - ang) > 0) { // turn right
      ang = -fabs(pos_ori.z + PI - ang);
    } else { // turn left
      ang = fabs(pos_ori.z + PI - ang);
    }
  }
}
*/