#include <ros/ros.h>
#include "nodes/state_publisher.h"
#include "cbot/serial.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delta_fk");
    ros::NodeHandle n;

    cbot::Serial::Dimensions dim;
    dim.dh_parameters.push_back(
        cbot::Serial::DHParameter(0, 0.1, 0));
    dim.dh_parameters.push_back(
        cbot::Serial::DHParameter(0, 0, -M_PI/2, -M_PI/2));
    dim.dh_parameters.push_back(
        cbot::Serial::DHParameter(0.4, 0, 0));
    dim.dh_parameters.push_back(
        cbot::Serial::DHParameter(0.4, 0, 0, 0, false));
    dim.dh_parameters.push_back(
        cbot::Serial::DHParameter(0.1, 0, 0));
    dim.dh_parameters.push_back(
        cbot::Serial::DHParameter(0.2, 0, 0, 0, false));

    cbot::Serial::JointNames joint_names;
    joint_names.push_back("theta_1");
    joint_names.push_back("theta_2");
    joint_names.push_back("theta_3");
    joint_names.push_back("theta_4");
    joint_names.push_back("theta_5");
    joint_names.push_back("theta_6");

    StatePublisherNode node(n, new cbot::Serial(dim, joint_names));
    ros::spin();
}
