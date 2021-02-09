#include "joint_publisher.h"

#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>


JointPublisher::JointPublisher(ros::NodeHandle &n, const std::vector<std::string> &joints):
    mode(Mode::VELOCITY),
    N_JOINTS(joints.size()),
    joint_positions(N_JOINTS, 0),
    joint_velocities(N_JOINTS, 0)
{
    std::stringstream ss;
    std::string topic;
    joint_positions_pub.resize(N_JOINTS);
    for (int i = 0; i < N_JOINTS; i++) {
        ss.str("");
        ss << joints[i] << "_controller/command";
        topic = ss.str();
        ss.clear();
        joint_positions_pub[i] = n.advertise<std_msgs::Float64>(topic, 1);
    }
}

void JointPublisher::loop(const ros::TimerEvent &timer)
{
    double dt = (timer.current_real - timer.last_real).toSec();
    for (std::size_t i = 0; i < N_JOINTS; i++) {
        if (mode == Mode::VELOCITY) {
            joint_positions[i] += joint_positions[i]*dt;
        }
        std_msgs::Float64 msg;
        msg.data = joint_positions[i];
        joint_positions_pub[i].publish(msg);
    }
}

void JointPublisher::set_joint_velocities(const std::vector<double> &joint_velocities_in)
{
    std::copy(
        joint_velocities_in.begin(), joint_velocities_in.end(),
        joint_velocities.begin()
    );
}

void JointPublisher::set_joint_positions(const std::vector<double> &joint_positions_in)
{
    std::copy(
        joint_positions_in.begin(), joint_positions_in.end(),
        joint_positions.begin()
    );
}

void JointPublisher::set_mode(Mode mode)
{
    this->mode = mode;
}
