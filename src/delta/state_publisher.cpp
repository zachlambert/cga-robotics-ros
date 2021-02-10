#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "cbot/delta.h"

#include "cbot/conversions.h"

class Node {
public:
    Node(
        ros::NodeHandle &n,
        const cbot::Delta::Dimensions &dim,
        const cbot::Delta::JointNames &joint_names):
        delta(dim, joint_names)
    {
        joint_states_in_sub = n.subscribe(
            "joint_states_in", 1, &Node::joint_states_callback, this
        );
        joint_states_out_pub = n.advertise<sensor_msgs::JointState>(
            "joint_states_out", 1
        );

        ee_pose_pub = n.advertise<geometry_msgs::PoseStamped>(
            "ee_pose", 1
        );
        ee_twist_pub = n.advertise<geometry_msgs::TwistStamped>(
            "ee_twist", 1
        );
    }

    void joint_states_callback(const sensor_msgs::JointState &joint_states_in)
    {
        for (std::size_t i = 0; i < joint_states_in.name.size(); i++) {
            // If the delta model doesn't use a joint provided, it is ignored
            delta.set_joint_position(joint_states_in.name[i], joint_states_in.position[i]);
            delta.set_joint_velocity(joint_states_in.name[i], joint_states_in.velocity[i]);
        }

        if (!delta.update_pose()) {
            ROS_ERROR("Failed to do FK for pose");
            return;
        }
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.pose = cbot::to_msg(delta.get_pose());
        pose_msg.header.frame_id="base";
        pose_msg.header.stamp = joint_states_in.header.stamp;
        ee_pose_pub.publish(pose_msg);

        if (!delta.update_twist()) {
            ROS_ERROR("Failed to do FK for twist.");
            return;
        }
        geometry_msgs::TwistStamped twist_msg;
        twist_msg.twist = cbot::to_msg(delta.get_twist());
        twist_msg.header.frame_id="base";
        twist_msg.header.stamp = joint_states_in.header.stamp;
        ee_twist_pub.publish(twist_msg);

        if (!delta.update_dependent_joints()) {
            ROS_ERROR("Failed to calculate dependent joints.");
        }
        sensor_msgs::JointState joint_states(joint_states_in);
        auto dependent_joints = delta.get_dependent_joint_names();
        for (std::size_t i = 0; i < dependent_joints.size(); i++) {
            joint_states.name.push_back(dependent_joints[i]);
            joint_states.position.push_back(delta.get_joints().at(dependent_joints[i]).position);
        }
        joint_states.velocity.resize(joint_states.name.size(), 0);
        joint_states.effort.resize(joint_states.name.size(), 0);
        joint_states_out_pub.publish(joint_states);
    }

private:
    cbot::Delta delta;

    ros::Subscriber joint_states_in_sub;
    ros::Publisher joint_states_out_pub;
    ros::Publisher ee_pose_pub;
    ros::Publisher ee_twist_pub;

    std::string theta_names[3];
    std::string alpha_names[3];
    std::string beta_names[3];
    std::string gamma_names[3];
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delta_fk");
    ros::NodeHandle n;

    cbot::Delta::Dimensions dim;
    ros::NodeHandle n_local("~");
    n_local.getParam("base_radius", dim.r_base);
    n_local.getParam("ee_radius", dim.r_ee);
    n_local.getParam("upper_length", dim.l_upper);
    n_local.getParam("lower_length", dim.l_lower);

    cbot::Delta::JointNames joint_names;
    // TODO: Tidy this up
    joint_names.theta.push_back("theta_1");
    joint_names.theta.push_back("theta_2");
    joint_names.theta.push_back("theta_3");
    joint_names.alpha.push_back("alpha_1");
    joint_names.alpha.push_back("alpha_2");
    joint_names.alpha.push_back("alpha_3");
    joint_names.beta.push_back("beta_1");
    joint_names.beta.push_back("beta_2");
    joint_names.beta.push_back("beta_3");
    joint_names.gamma.push_back("gamma_1");
    joint_names.gamma.push_back("gamma_2");
    joint_names.gamma.push_back("gamma_3");

    Node node(n, dim, joint_names);
    ros::spin();
}
