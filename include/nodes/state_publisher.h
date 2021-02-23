#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "cbot/cbot.h"

class StatePublisherNode {
public:
    StatePublisherNode(ros::NodeHandle &n, cbot::Robot *robot);

private:
    void joint_states_callback(const sensor_msgs::JointState &joint_states_in);

    std::unique_ptr<cbot::Robot> robot;

    ros::Subscriber joint_states_in_sub;
    ros::Publisher joint_states_out_pub;
    ros::Publisher ee_pose_pub;
    ros::Publisher ee_twist_pub;

    std::string theta_names[3];
    std::string alpha_names[3];
    std::string beta_names[3];
    std::string gamma_names[3];
};
