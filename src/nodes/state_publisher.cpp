#include "nodes/state_publisher.h"

#include "cbot/conversions.h"

StatePublisherNode::StatePublisherNode(ros::NodeHandle &n, cbot::Robot *robot):
    robot(robot)
{
    if (robot->is_parallel()) {
        joint_states_in_sub = n.subscribe(
            "joint_states_in", 1, &StatePublisherNode::joint_states_callback, this
        );
        joint_states_out_pub = n.advertise<sensor_msgs::JointState>(
            "joint_states_out", 1
        );
    } else {
        joint_states_in_sub = n.subscribe(
            "joint_states", 1, &StatePublisherNode::joint_states_callback, this
        );
        // joint_states_out_pub unused
    }

    ee_pose_pub = n.advertise<geometry_msgs::PoseStamped>(
        "ee_pose", 1
    );
    ee_twist_pub = n.advertise<geometry_msgs::TwistStamped>(
        "ee_twist", 1
    );
}

void StatePublisherNode::joint_states_callback(const sensor_msgs::JointState &joint_states_in)
{
    for (std::size_t i = 0; i < joint_states_in.name.size(); i++) {
        // If the delta model doesn't use a joint provided, it is ignored
        robot->set_joint_position(joint_states_in.name[i], joint_states_in.position[i]);
        robot->set_joint_velocity(joint_states_in.name[i], joint_states_in.velocity[i]);
    }

    if (!robot->update_pose()) {
        ROS_ERROR("Failed to do FK for pose");
        return;
    }
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose = cbot::to_msg(robot->get_pose());
    pose_msg.header.frame_id="base_link";
    pose_msg.header.stamp = joint_states_in.header.stamp;
    ee_pose_pub.publish(pose_msg);

    if (!robot->update_twist()) {
        ROS_ERROR("Failed to do FK for twist.");
        return;
    }
    geometry_msgs::TwistStamped twist_msg;
    twist_msg.twist = cbot::to_msg(robot->get_twist());
    twist_msg.header.frame_id="base_link";
    twist_msg.header.stamp = joint_states_in.header.stamp;
    ee_twist_pub.publish(twist_msg);

    if (!robot->is_parallel()) return;

    if (!robot->update_dependent_joints()) {
        ROS_ERROR("Failed to calculate dependent joints.");
    }
    sensor_msgs::JointState joint_states(joint_states_in);
    auto dependent_joints = robot->get_dependent_joint_names();
    for (std::size_t i = 0; i < dependent_joints.size(); i++) {
        joint_states.name.push_back(dependent_joints[i]);
        joint_states.position.push_back(robot->get_joint_position(dependent_joints[i]));
    }
    joint_states.velocity.resize(joint_states.name.size(), 0);
    joint_states.effort.resize(joint_states.name.size(), 0);
    joint_states_out_pub.publish(joint_states);
}
