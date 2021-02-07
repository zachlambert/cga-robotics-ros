#include <sstream>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include "cbot/delta.h"
namespace cbot { using namespace linalg_impl; }
#include "cbot/conversions.h"

class Node {
public:
    Node(ros::NodeHandle &n, cbot::Delta::Config config): delta(config)
    {
        ee_twist_cmd_sub = n.subscribe(
            "ee_twist_cmd", 1, &Node::ee_twist_cmd_callback, this
        );
        joint_states_sub = n.subscribe(
            "joint_states", 1, &Node::joint_states_callback, this
        );
        joint_vel_pub = n.advertise<std_msgs::Float64MultiArray>(
            "joint_velocities", 1
        );

        joint_vel_msg.layout.data_offset = 0;
        joint_vel_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        joint_vel_msg.layout.dim[0].size = 3;
        joint_vel_msg.layout.dim[0].stride = 1;
        joint_vel_msg.layout.dim[0].label = "joint";
        joint_vel_msg.data.resize(3);

        joint_names.push_back("theta_1");
        joint_names.push_back("theta_2");
        joint_names.push_back("theta_3");
    }

    void ee_twist_cmd_callback(const geometry_msgs::TwistStamped msg)
    {
        twist_cmd = cbot::from_msg(msg.twist);
    }

    void joint_states_callback(const sensor_msgs::JointState &joint_states)
    {
        cbot::Delta::Joints joints_pos;
        // Don't rely on indexes in joint_states message remaining
        // consistent, so search through each time
        for (std::size_t i = 0; i < joint_states.name.size(); i++) {
            for (std::size_t j = 0; j < joint_names.size(); j++) {
                if (joint_states.name[i] == joint_names[j]) {
                    joints_pos.theta[j] = joint_states.position[i];
                }
            }
        }

        cbot::Delta::Joints joints_vel;
        if (!delta.ik_twist(twist_cmd, joints_pos, joints_vel)) {
            ROS_ERROR("Failed to do velocity IK");
            return;
        }
        for (int i = 0; i < 3; i++) {
            joint_vel_msg.data[i] = joints_vel.theta[i];
        }
        joint_vel_pub.publish(joint_vel_msg);
    }

private:
    cbot::Delta delta;
    std::vector<std::string> joint_names;

    ros::Subscriber ee_twist_cmd_sub;
    cbot::Twist twist_cmd;
    ros::Subscriber joint_states_sub;
    ros::Publisher joint_vel_pub;
    std_msgs::Float64MultiArray joint_vel_msg;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delta_ik");
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
