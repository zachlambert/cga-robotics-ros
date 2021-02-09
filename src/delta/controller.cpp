#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "cbot/delta.h"
// namespace cbot { using namespace cga_impl; }
namespace cbot { using namespace linalg_impl; }

#include "cga_robotics_ros/RobotCommand.h"
#include "cbot/conversions.h"
#include "joint_publisher.h"

class Node {
public:
    Node(ros::NodeHandle &n, cbot::Delta::Config config): delta(config)
    {
        joint_states_in_sub = n.subscribe(
            "joint_states_in", 1, &Node::joint_states_callback, this
        );
        joint_states_out_pub = n.advertise<sensor_msgs::JointState>(
            "joint_states_out", 1
        );

        std::stringstream ss;
        theta_names.resize(3);
        alpha_names.resize(3);
        beta_names.resize(3);
        gamma_names.resize(3);
        for (int i = 0; i < 3; i++) {
            ss.str("");
            ss << "theta_" << (i+1);
            theta_names[i] = ss.str();
            ss.str("");
            ss << "alpha_" << (i+1);
            alpha_names[i] = ss.str();
            ss.str("");
            ss << "beta_" << (i+1);
            beta_names[i] = ss.str();
            ss.str("");
            ss << "gamma_" << (i+1);
            gamma_names[i] = ss.str();
        }

        joint_publisher = std::unique_ptr<JointPublisher>(new JointPublisher(n, theta_names));

        ee_pose_pub = n.advertise<geometry_msgs::PoseStamped>(
            "ee_pose", 1
        );

        robot_command_server = n.advertiseService(
            "robot_command", &Node::robot_command_callback, this
        );
    }

    void joint_states_callback(const sensor_msgs::JointState &joint_states_in)
    {
        // Read theta joints in. Don't rely on the joints being provided
        // in any specific order, so have to search joint names.
        cbot::Delta::Joints joints_pos;
        for (int i = 0; i < 3; i++) {
            for (std::size_t j = 0; j < joint_states_in.name.size(); j++) {
                if (theta_names[i] == joint_states_in.name[j]) {
                    joints_pos.theta[i] = joint_states_in.position[j];
                    break;
                }
            }
        }

        // Get the end effector pose and dependent joint values
        cbot::Pose pose;
        cbot::Delta::JointsDep joints_dep_pos;
        if (!delta.fk_pose(joints_pos, &joints_dep_pos, pose)) {
            ROS_ERROR("Failed to do FK");
            return;
        }

        // Copy the joint_states_in message, then append the dependent
        // joints, which shouldn't be present in the original message

        sensor_msgs::JointState joint_states(joint_states_in);

        for (int i = 0; i < 3; i++) {
            joint_states.name.push_back(alpha_names[i]);
            joint_states.position.push_back(joints_dep_pos.alpha[i]);
        }
        for (int i = 0; i < 3; i++) {
            joint_states.name.push_back(beta_names[i]);
            joint_states.position.push_back(joints_dep_pos.beta[i]);
        }
        for (int i = 0; i < 3; i++) {
            joint_states.name.push_back(gamma_names[i]);
            joint_states.position.push_back(joints_dep_pos.gamma[i]);
        }
        joint_states.velocity.resize(joint_states.name.size());
        joint_states.effort.resize(joint_states.name.size());

        joint_states_out_pub.publish(joint_states);

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.pose = cbot::to_msg(pose);
        pose_msg.header.frame_id="base";
        pose_msg.header.stamp = joint_states_in.header.stamp;
        ee_pose_pub.publish(pose_msg);
    }

    bool robot_command_callback(
        cga_robotics_ros::RobotCommand::Request &req,
        cga_robotics_ros::RobotCommand::Request &res)
    {
        return true;
    }

private:
    cbot::Delta delta;
    std::unique_ptr<JointPublisher> joint_publisher;

    ros::Subscriber joint_states_in_sub;
    ros::Subscriber ee_twist_sub;
    ros::Publisher joint_states_out_pub;
    ros::Publisher ee_pose_pub;

    ros::ServiceServer robot_command_server;

    std::vector<std::string> theta_names;
    std::vector<std::string> alpha_names;
    std::vector<std::string> beta_names;
    std::vector<std::string> gamma_names;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delta_fk");
    ros::NodeHandle n;

    cbot::Delta::Config config;
    ros::NodeHandle n_local("~");
    n_local.getParam("base_radius", config.r_base);
    n_local.getParam("ee_radius", config.r_ee);
    n_local.getParam("upper_length", config.l_upper);
    n_local.getParam("lower_length", config.l_lower);

    Node node(n, config);
    ros::spin();
}
