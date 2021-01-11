#ifndef CBOT_CONVERSIONS_H
#define CBOT_CONVERSIONS_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
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
};

#endif
