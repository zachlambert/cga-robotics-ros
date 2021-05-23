#ifndef CBOT_CONVERSIONS_H
#define CBOT_CONVERSIONS_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "cbot/types.h"

namespace cbot {

geometry_msgs::Pose to_msg(const Pose &pose)
{
    geometry_msgs::Pose msg;
    msg.position.x = pose.position.x;
    msg.position.y = pose.position.y;
    msg.position.z = pose.position.z;
    msg.orientation.w = pose.orientation.w;
    msg.orientation.x = pose.orientation.x;
    msg.orientation.y = pose.orientation.y;
    msg.orientation.z = pose.orientation.z;
    return msg;
}

Pose from_msg(const geometry_msgs::Pose &msg)
{
    Pose pose;
    pose.position.x = msg.position.x;
    pose.position.y = msg.position.y;
    pose.position.z = msg.position.z;
    pose.orientation.w = msg.orientation.w;
    pose.orientation.x = msg.orientation.x;
    pose.orientation.y = msg.orientation.y;
    pose.orientation.z = msg.orientation.z;
    return pose;
}

geometry_msgs::Twist to_msg(const Twist &twist)
{
    geometry_msgs::Twist msg;
    msg.linear.x = twist.linear.x;
    msg.linear.y = twist.linear.y;
    msg.linear.z = twist.linear.z;
    msg.angular.x = twist.angular.x;
    msg.angular.y = twist.angular.y;
    msg.angular.z = twist.angular.z;
    return msg;
}

Twist from_msg(const geometry_msgs::Twist &msg)
{
    Twist twist;
    twist.linear.x = msg.linear.x;
    twist.linear.y = msg.linear.y;
    twist.linear.z = msg.linear.z;
    twist.angular.x = msg.angular.x;
    twist.angular.y = msg.angular.y;
    twist.angular.z = msg.angular.z;
    return twist;
}

geometry_msgs::Wrench to_msg(const Wrench &wrench)
{
    geometry_msgs::Wrench msg;
    msg.force.x = wrench.force.x;
    msg.force.y = wrench.force.y;
    msg.force.z = wrench.force.z;
    msg.torque.x = wrench.torque.x;
    msg.torque.y = wrench.torque.y;
    msg.torque.z = wrench.torque.z;
    return msg;
}

Wrench from_msg(const geometry_msgs::Wrench &msg)
{
    Wrench wrench;
    wrench.force.x = msg.force.x;
    wrench.force.y = msg.force.y;
    wrench.force.z = msg.force.z;
    wrench.torque.x = msg.torque.x;
    wrench.torque.y = msg.torque.y;
    wrench.torque.z = msg.torque.z;
    return wrench;
}

trajectory_msgs::JointTrajectory to_msg(const JointTrajectory &trajectory)
{
    trajectory_msgs::JointTrajectory msg;
    msg.joint_names = trajectory.names;
    std::size_t num_joints = trajectory.names.size();
    msg.header.stamp = ros::Time::now();
    msg.points = std::vector<trajectory_msgs::JointTrajectoryPoint>(trajectory.points.size());
    for (std::size_t i = 0; i < msg.points.size(); i++) {
        msg.points[i].positions = trajectory.points[i].positions;
        msg.points[i].velocities.resize(num_joints, 0.0);
        msg.points[i].accelerations.resize(num_joints, 0.0);
        msg.points[i].effort.resize(num_joints, 0.0);
        msg.points[i].time_from_start = ros::Duration(trajectory.points[i].time);
    }
    return msg;
}

JointTrajectory from_msg(const trajectory_msgs::JointTrajectory &msg)
{
    JointTrajectory trajectory;
    trajectory.names = msg.joint_names;
    trajectory.points = std::vector<JointTrajectoryPoint>(msg.points.size());
    for (std::size_t i = 0; i < trajectory.points.size(); i++) {
        trajectory.points[i].positions = msg.points[i].positions;
        trajectory.points[i].time = msg.points[i].time_from_start.toSec();
    }
    return trajectory;
}

} // namespace cbot

#endif
