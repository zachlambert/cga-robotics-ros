#ifndef NODES_CONTROLLER_H
#define NODES_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/server/simple_action_server.h>
#include <cga_robotics_ros/TrajectoryAction.h>

#include "cga_robotics_ros/TrajectoryFeedback.h"
#include "joint_group_publisher.h"

#include "cbot/cbot.h"

class ControllerNode {
public:
    ControllerNode(ros::NodeHandle &n, cbot::Robot *robot);

private:
    void trajectory_callback(const cga_robotics_ros::TrajectoryGoalConstPtr &goal);
    void ee_twist_cmd_callback(const geometry_msgs::TwistStamped &ee_twist_cmd);
    void loop_velocity(const ros::TimerEvent &timer);

    std::unique_ptr<cbot::Robot> robot;
    JointGroupPublisher joint_publisher;
    JointGroupPublisher gripper_publisher;

    actionlib::SimpleActionServer<cga_robotics_ros::TrajectoryAction> trajectory_server;

    ros::Subscriber ee_twist_cmd_sub;
    cbot::Twist ee_twist_cmd;

    ros::Timer velocity_timer;
    bool skip_velocity_timer;
};

#endif
