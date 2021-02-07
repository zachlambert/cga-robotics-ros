#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "cbot/delta.h"
namespace cbot { using namespace linalg_impl; }

#include "cbot/conversions.h"

class Node {
public:
    Node(ros::NodeHandle &n, cbot::Delta::Config config): delta(config)
    {
        joint_states_sub = n.subscribe(
            "joint_states", 1, &Node::joint_states_callback, this
        );

        ee_twist_pub = n.advertise<geometry_msgs::TwistStamped>("ee_twist", 1);

        joint_names.push_back("theta_1");
        joint_names.push_back("theta_2");
        joint_names.push_back("theta_3");
    }

    void joint_states_callback(const sensor_msgs::JointState &joint_states)
    {
        cbot::Delta::Joints pos, vel;
        // Don't rely on indexes in joint_states message remaining
        // consistent, so search through each time
        for (std::size_t i = 0; i < joint_states.name.size(); i++) {
            for (std::size_t j = 0; j < joint_names.size(); j++) {
                if (joint_states.name[i] == joint_names[j]) {
                    pos.theta[j] = joint_states.position[i];
                    vel.theta[j] = joint_states.velocity[i];
                }
            }
        }

        cbot::Twist twist;
        delta.fk_twist(pos, vel, twist);
        geometry_msgs::TwistStamped twist_msg;
        twist_msg.header.frame_id = "base";
        twist_msg.header.stamp = joint_states.header.stamp;
        twist_msg.twist = cbot::to_msg(twist);
        ee_twist_pub.publish(twist_msg);
    }

private:
    cbot::Delta delta;
    ros::Subscriber joint_states_sub;
    ros::Publisher ee_twist_pub;
    std::vector<std::string> joint_names;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delta_fk_vel");
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
