#include "avoid.h"

Avoid::Avoid(char const *num) {
  count = 0;
  topic = "/tb3_" + std::string(num);
  std::cout << topic << std::endl;
  ros::service::waitForService("/Global"); // wait until service started
  ros::ServiceClient global_client =
      nh.serviceClient<std_srvs::Empty>("/Global");
  ros::service::waitForService(
      "/gazebo/get_model_state"); // wait until service started
  ros::ServiceClient state_client =
      nh.serviceClient<std_srvs::Empty>("/gazebo/get_model_state");
  sub = nh.subscribe(topic + "/scan", 1000, &Avoid::counterCallback, this);
  pub = nh.advertise<geometry_msgs::Twist>(topic + "/cmd_vel", 1000);
  ROS_INFO("start");
}
void Avoid::moving() {
  ros::Rate loop_rate(10);
  pub.publish(vel);
  while (ros::ok()) {
    ros::spinOnce();
    // pub.publish(vel);
    loop_rate.sleep();
  }
}

void Avoid::get_obstacle_info() {
  gazebo_msgs::GetModelState msg;
  msg.request.model_name = "turtlebot3_waffle_pi_1";
  geometry_msgs::Point goal;
  goal.x = 10.0;
  goal.y = 10.0;
  goal.z = 0.0;
  if (state_client.call(msg)) {
    ROS_INFO("Successfully call robot state");
  }
  if (global_client.call(goal, msg)) {
    ROS_INFO("Successfully get obstacle state");
  }
}

void Avoid::angular_control() {
  float phi = td - t1;
  float theta;
  float trans = PI / 180;
  if (td > 0) { // turn right
    if (td > phi) {
      theta = td;
    } else {
      theta = phi;
    }
    vel.angular.z -= WAFFLE_MAX_ANG_VEL;
  } else { // turn left
    if (td < phi) {
      theta = td;
    } else {
      theta = phi;
    }
    vel.angular.z += fmod((double)theta, (PI / 2)); // turn left is positive
  }
  pub.publish(vel);
  ROS_INFO("%f", theta);
  // ros::Duration(theta/WAFFLE_MAX_ANG_VEL).sleep();
  // vel.angular.z = 0;
  // pub.publish(vel);
}
void Avoid::velocity_control() {
  vel.linear.x *= 1 - exp(-0.5 * pow(tti, 2));
  pub.publish(vel);
}
void Avoid::get_basics(const sensor_msgs::LaserScan::ConstPtr &msg) {
  // td = (ob[2].subscript - ob[0].subscript) / 2 * msg->time_increment;
  float a1 = ob[1].subscript;
  float a2 = ob[0].subscript;
  float trans = PI / 180;
  // increase the distance for convinience
  float x = ob[1].low * expand_coe;
  float y = ob[0].low * expand_coe;
  float distance_r = vel.linear.x * scan_time *
                     expand_coe; // instant moving distance for robot
  ROS_INFO("angle1: %f, angle2: %f, x: %f, y: %f, moving: %f", a1, a2, x, y,
           distance_r);
  float z2 = pow(x, 2) + pow(distance_r, 2) -
             2 * x * distance_r * cos((180 - a1) * trans); // square z
  float t = distance_r * sin(a2 * trans) - cos((90 - a2 + a1) * trans) * x;
  float distance_o = pow(pow(t, 2) + pow(y - x * sin((90 - a2 + a1) * trans) -
                                             distance_r * cos(a2 * trans),
                                         2),
                         0.5); // instant moving distance for obstacle
  float vo = distance_o / scan_time / expand_coe;
  float a3 = asin(t / distance_o) / trans;
  float a4 = a2 + a3 - a1;
  float vconv = vo * cos(a4 * trans) + vel.linear.x * cos(a2 * trans);
  float vorth = vo * sin(a4 * trans) - vel.linear.x * sin(a2 * trans);
  ROS_INFO("%f, %f, %f, %f, %f, %f", z2, t, distance_o, vo, vconv, vorth);
  tti = x / expand_coe / fabs(vconv);
  ROS_INFO("the moment tti is %f", tti);
  td = atan(fabs(vorth) / (x / expand_coe - fabs(vconv)));
  ROS_INFO("the moment td is %f", td);
  if (td < 0) {
    t1 = a - b * pow(tti, -c);
  } else {
    t1 = a + b * pow(tti, -c);
  }
}
void Avoid::calculate(const sensor_msgs::LaserScan::ConstPtr &msg) {
  ob[count].low = 10000;
  for (int i = 0; i < msg->ranges.size(); i++) {
    // ROS_INFO("the moment distance is :%f", msg->ranges[i]);
    // ROS_INFO("%d: %f",i,msg->ranges[i]);
    if (msg->ranges[i] <= ob[count].low) {
      ob[count].low = msg->ranges[i];
      // ROS_INFO("the narrowed distance is :%f", ob[count].low);
      ob[count].subscript =
          ((float)i - 1) /
          4; // the sending range is different with the coordinate system
             // ROS_INFO("the moment angle is :%f", ob[count].subscript);
    }
  }
  // ROS_INFO("the range_max is %f", msg->range_max);
  ROS_INFO("the lowest distance is :%f", ob[count].low);
}
void Avoid::counterCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  if (count < 2) {
    calculate(msg);
    count++;
    if (ob[count - 1].low >=
        LIMIT) { // robot is away from the obstacle; the lowest range
      // is smaller than the dange range
      count = 0;
      memset(&ob, 0, sizeof(struct obstacle));
      ROS_INFO("no need to avoid, obstacle is too far away");
      vel.linear.x = 0.5;
      vel.angular.z = 0;
      pub.publish(vel);
    }
    if (count == 1) {
      ros::Duration(scan_time).sleep();
    } // wait on second to receive the next obstacle pos
  } else if (count == 2) {
    if (ob[0].low <= ob[1].low) { // since they are Uniform linear motion, they
                                  // would not collise in this situation
      count = 0;
      memset(&ob, 0, sizeof(struct obstacle));
      ROS_INFO("no need to avoid, obstacles start to go far away");
      vel.linear.x = 0.5;
      vel.angular.z = 0;
      pub.publish(vel);
    } else {
      ROS_INFO("Prepared to measure the ttd & ti");
      get_basics(msg); // to get the tti and td
      if (td < t1 &&
          tti > 0) { // a walker detects a risk of future collision when ˙α is
        // low and ttii > 0.
        ROS_INFO("angular_control");
        angular_control();
      }
      if (tti < t2 && tti > 0 &&
          td < 0) { // The imminence of a collision is detected when
        // ttii is positive but lower than a threshold τ2
        ROS_INFO("velocity_control");
        velocity_control();
      }
      count = 0;
      memset(&ob, 0, sizeof(struct obstacle));
      ROS_INFO("linear_vel: %f, angular_vel: %f", vel.linear.x, vel.angular.z);
      ROS_INFO("finish all control");
    }
  }
}
