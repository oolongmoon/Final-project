#include "avoid.h"
#include <cmath>

Avoid::Avoid(char const *num) {
  count = 0;
  detect = 1;
  topic = "/tb3_" + std::string(num);
  std::cout << topic << std::endl;
  ros::service::waitForService("/Global"); // wait until service started
  global_client = nh.serviceClient<turtlebot3_navigation::info>("/Global");
  ros::service::waitForService(
      "/gazebo/get_model_state"); // wait until service started
  state_client =
      nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  sub = nh.subscribe(topic + "/scan", 1000, &Avoid::counterCallback, this);
  pub = nh.advertise<geometry_msgs::Twist>(topic + "/cmd_vel", 1000);
  pub.publish(vel); // initial the Publisher
  ros::Duration(1).sleep();
  ROS_INFO("start");
}
void Avoid::moving() {
  ros::Rate loop_rate(10);
  // global_control();
  while (ros::ok()) {
    ros::spinOnce();
    pub.publish(vel);
    loop_rate.sleep();
  }
}

void Avoid::get_obstacle_info() {
  gazebo_msgs::GetModelState msg;
  msg.request.model_name = "tb3_0";
  turtlebot3_navigation::info info;
  info.request.goal.x = 10.0;
  info.request.goal.y = 0.0;
  info.request.goal.z = 0.0;
  if (state_client.call(msg)) {
    // ROS_INFO("Successfully call robot state");
    info.request.pos = msg.response.pose.position;
    info.request.pos.z = oriTrans(msg.response.pose.orientation); // the yaw
  }
  if (global_client.call(info)) {
    // ROS_INFO("Successfully get obstacle state");
    ang = info.response.angle;
    dis = info.response.distance;
    // ROS_INFO("angle: %f, distance: %f", ang, dis);
  }
}
void Avoid::global_control() {
  detect = 0;
  vel.linear.x = 0;
  get_obstacle_info();
  if (ang > 0) {
    ROS_INFO("Start turn left");
    vel.angular.z = 0.2;
    float time = ang / vel.angular.z + 0.5;
    ROS_INFO("time to totate: %f", time);
    pub.publish(vel);
    ros::Duration(time).sleep();
    ROS_INFO("Rotate finish");
  } else {
    ROS_INFO("Start turn right");
    vel.angular.z = -0.2;
    float time = ang / vel.angular.z;
    ROS_INFO("time to totate: %f", time);
    pub.publish(vel);
    ros::Duration(time).sleep();
    ROS_INFO("Rotate finish");
  }
  ROS_INFO("Start move forward");
  vel.linear.x = Vcomf;
  vel.angular.z = 0;
  pub.publish(vel);
  /*
  if (detect == 1) {
    ros::Duration(dis / vel.linear.x).sleep();
    ROS_INFO("Navigation finished");
    vel.linear.x = 0;
    pub.publish(vel);
  }
  */
}

void Avoid::get_basics() { // get tti and td
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
  ROS_INFO("threshold 1: %f", t1);
}
void Avoid::calculate(const sensor_msgs::LaserScan::ConstPtr
                          &msg) { // get obstacle distance and angle
  ob[count].low = 10000;
  for (int i = 0; i < 360; i++) {

    if (msg->ranges[i] <= ob[count].low) {
      ob[count].low = msg->ranges[i];
      // ROS_INFO("the narrowed distance is :%f", ob[count].low);
      ob[count].subscript = ((float)i + 1) / 4;
      ; // the sending range is different with the coordinate system
        // ROS_INFO("the moment angle is :%f", ob[count].subscript);
    }
  }
  // if (ob[count].subscript > 180 && ob[count].subscript <= 360) {
  //  ob[count].subscript = 360 - ob[count].subscript;
  //}

  // ROS_INFO("the range_max is %f", msg->range_max);
  // ROS_INFO("the lowest distance is :%f", ob[count].low);
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
      count = 0;
      if (detect == 1) {
        global_control();
      } // wait on second to receive the next obstacle pos
    }
    if (count == 1) {
      ros::Duration(scan_time).sleep();
    }
  } else if (count == 2) {
    ROS_INFO("Detect a object may interact");
    ROS_INFO("Prepared to measure the ttd & ti");
    get_basics();
    if (ob[0].low > ob[1].low) {

      if (tti <= 0) { // since they are Uniform linear motion,
                      // they would not collise in this situation
        count = 0;
        memset(&ob, 0, sizeof(struct obstacle));
        ROS_INFO("no need do angular control");
        if (detect == 1) {
          global_control();
        }
      } else {
        // to get the tti and td
        detect = 1;
        float phi =
            fmod((td - t1), WAFFLE_MAX_ANG_VEL); // a walker detects a risk of
                                                 // future collision when ˙α is
        // low and ttii > 0.
        ROS_INFO("angular_control");
        /*
        get_obstacle_info();
        float td_g = atan(fabs(vel.linear.x * sin(ang)) / dis -
                          (fabs(vel.linear.x * cos(ang))));
        if (((td < 0 && td_g < phi) || (td > 0 && td_g > phi)) &&
            (fabs(td_g) > 0.1)) {
          vel.angular.z = fmod(td_g,2);
          ROS_INFO("td_g, angular_vel: %f", vel.angular.z);
        } else {
        */
        vel.angular.z = phi;

        ROS_INFO("ob_g, angular_vel: %f", vel.angular.z);
        /*
      }
      */
        vel.linear.x = 0.15;
        /*
        if (tti < t2) { // The imminence of a collision is detected when
          // ttii is positive but lower than a threshold τ2
          ROS_INFO("velocity_control");
          vel.linear.x *= 1 - exp(-0.5 * pow(tti, 2));
        } else {
          vel.linear.x = Vcomf;
          ROS_INFO("no need do linear velocity control");
        }
        */
        ROS_INFO("linear_vel: %f", vel.linear.x);
        count = 0;
        memset(&ob, 0, sizeof(struct obstacle));
        ROS_INFO("finish all control");
        pub.publish(vel);
      }
    } else {
      ROS_INFO("no need to avoid, obstacles start to go far away");
      memset(&ob, 0, sizeof(struct obstacle));
      count = 0;
      if (detect == 1) {
        global_control();
      }
    }
  }
}
//}

double Avoid::oriTrans(const geometry_msgs::Quaternion msg) {
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  geometry_msgs::Vector3 ori;
  ori.x = roll;
  ori.y = pitch;
  ori.z = yaw;
  return ori.z;
}
/*void Avoid::angular_control() {
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
}*/