#include <ros/ros.h>
#include "nodes/controller.h"
#include "cbot/serial.h"

bool state_constraint(const cbot::Joints &joints)
{
    for (const auto &pair: joints) {
        if (pair.second.position > M_PI/3) return false;
        if (pair.second.position < -M_PI/3) return false;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delta_fk");
    ros::NodeHandle n;

    cbot::Serial::Dimensions dim;
    dim.dh_parameters.push_back(
        cbot::Serial::DHParameter(0, 0.083, 0));
    dim.dh_parameters.push_back(
        cbot::Serial::DHParameter(0.04, 0, -M_PI/2, -1.4));
    dim.dh_parameters.push_back(
        cbot::Serial::DHParameter(0.13, 0, 0, 0.1));
    dim.dh_parameters.push_back(
        cbot::Serial::DHParameter(0.02, 0.165, -M_PI/2));
    dim.dh_parameters.push_back(
        cbot::Serial::DHParameter(0, 0, M_PI/2, 3.66));
    dim.dh_parameters.push_back(
        cbot::Serial::DHParameter(0.008, 0.075, M_PI/2, -M_PI/2));

    cbot::Serial::JointNames joint_names;
    joint_names.push_back("theta_1");
    joint_names.push_back("theta_2");
    joint_names.push_back("theta_3");
    joint_names.push_back("theta_4");
    joint_names.push_back("theta_5");
    joint_names.push_back("theta_6");

    cbot::Robot *robot = new cbot::Serial(dim, joint_names);
    ControllerNode node(n, robot, state_constraint);
    ros::spin();
}
