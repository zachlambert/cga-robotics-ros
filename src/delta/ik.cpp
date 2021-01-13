#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <sstream>

#include "cbot/delta.h"
namespace cbot { using namespace linalg_impl; }

#include "cbot/conversions.h"

class Node {
public:
    Node(ros::NodeHandle &n, cbot::Delta::Config config): delta(config)
    {
        ee_cmd_sub = n.subscribe<geometry_msgs::PoseStamped>(
            "ee_cmd", 1, &Node::ee_cmd_callback, this
        );

        std::stringstream ss;
        std::string topic;
        for (int i = 0; i < 3; i++) {
            ss.str("");
            ss << "theta_" << (i+1) << "_controller/command";
            topic = ss.str();
            ss.clear();
            theta_pub[i] = n.advertise<std_msgs::Float64>(topic, 1);
        }
    }

    void ee_cmd_callback(const geometry_msgs::PoseStamped msg)
    {
        cbot::Pose pose = cbot::from_msg(msg.pose);
        cbot::Delta::Joints joints;
        if (!delta.ik_pose(pose, joints)) {
            ROS_ERROR("Failed to do IK");
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
    ros::Publisher theta_pub[3];
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
