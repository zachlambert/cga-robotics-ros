#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <sstream>

#include "cbot/delta.h"
namespace cbot { using namespace linalg_impl; }

#include "cbot/conversions.h"

class Node {
public:
    Node(ros::NodeHandle &n, cbot::Delta::Config config): delta(config)
    {
        ee_cmd_sub = n.subscribe<geometry_msgs::TwistStamped>(
            "ee_twist_cmd", 1, &Node::ee_twist_cmd_callback, this
        );

        theta_vel_pub = n.advertise<std_msgs::Float64MultiArray>(
            "joint_velocities", 1
        );
    }

    void ee_twist_cmd_callback(const geometry_msgs::TwistStamped msg)
    {
        cbot::Twist twist = cbot::from_msg(msg.twist);
        cbot::Delta::Joints joints;
        cbot::Delta::Joints joints_vel;
        if (!delta.ik_twist(twist, joints, joints)) {
            ROS_ERROR("Failed to do velocity IK");
            return;
        }
        for (int i = 0; i < 3; i++) {
            std_msgs::Float64 msg;
            msg.data = joints.theta[i];
            theta_pub[i].publish(msg);
        }
    }

private:
    cbot::Delta delta;

    ros::Subscriber ee_cmd_sub;
    ros::Publisher theta_vel_pub[3];
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
