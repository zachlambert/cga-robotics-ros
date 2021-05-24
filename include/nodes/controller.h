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
#include <cga_robotics_ros/GripperAction.h>
#include <cga_robotics_ros/GripperFeedback.h>
#include "joint_group_publisher.h"

#include "cbot/cbot.h"

class ControllerNode {
public:
    ControllerNode(ros::NodeHandle &n, cbot::Robot *robot);

private:
    // Robot model

    std::unique_ptr<cbot::Robot> robot;

    // Main joint publisher

    JointGroupPublisher joint_publisher;

    void ee_twist_cmd_callback(const geometry_msgs::TwistStamped &ee_twist_cmd);
    ros::Subscriber ee_twist_cmd_sub;
    cbot::Twist ee_twist_cmd;

    actionlib::SimpleActionServer<cga_robotics_ros::TrajectoryAction> trajectory_server;
    void trajectory_callback(const cga_robotics_ros::TrajectoryGoalConstPtr &goal);

    // Gripper joint publisher

    JointGroupPublisher gripper_publisher;

    void gripper_cmd_callback(std_msgs::Float64 gripper_cmd);
    ros::Subscriber gripper_cmd_sub;
    double gripper_cmd;

    actionlib::SimpleActionServer<cga_robotics_ros::GripperAction> gripper_server;
    void gripper_callback(const cga_robotics_ros::GripperGoalConstPtr &goal);

    // Loop

    void loop_velocity(const ros::TimerEvent &timer);

    ros::Timer velocity_timer;
    bool skip_velocity_timer;
};

#endif
