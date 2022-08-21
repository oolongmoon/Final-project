//
// Created by oolongmoon on 03/08/2022.
//

#ifndef MSC_WS_DWA_PLANNER_ROS_H
#define MSC_WS_DWA_PLANNER_ROS_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf2_ros/buffer.h>

#include <dynamic_reconfigure/server.h>
#include </opt/ros/noetic/include/dwa_local_planner/DWAPlannerConfig.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

#include "dwa_planner.h"

namespace local_planner {
    class DWAPlannerROS : public nav_core::BaseLocalPlanner {
    public:
        DWAPlannerROS();

        void initialize(std::string name, tf2_ros::Buffer* tf,
                        costmap_2d::Costmap2DROS* costmap_ros);

        ~DWAPlannerROS();

        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);


        bool dwaComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& cmd_vel);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

        bool isGoalReached();



        bool isInitialized() {
            return initialized_;
        }

    private:
        void reconfigureCB(DWAPlannerConfig &config, uint32_t level);

        void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

        void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

        tf2_ros::Buffer* tf_;

        // for visualisation, publishers of global and local plan
        ros::Publisher g_plan_pub_, l_plan_pub_;

        base_local_planner::LocalPlannerUtil planner_util_;

        boost::shared_ptr<DWAPlanner> dp_;

        costmap_2d::Costmap2DROS* costmap_ros_;

        dynamic_reconfigure::Server<DWAPlannerConfig> *dsrv_;
        dwa_local_planner::DWAPlannerConfig default_config_;
        bool setup_;
        geometry_msgs::PoseStamped current_pose_;

        base_local_planner::LatchedStopRotateController latchedStopRotateController_;


        bool initialized_;


        base_local_planner::OdometryHelperRos odom_helper_;
        std::string odom_topic_;
    };
};

#endif //MSC_WS_DWA_PLANNER_ROS_H
