#include <ros/ros.h>
#include "nodes/controller.h"
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
        cbot::Serial::DHParameter(0.2, 0, 0, 0, false));
    dim.dh_parameters.push_back(
        cbot::Serial::DHParameter(0.1, 0, 0));
    dim.dh_parameters.push_back(
        cbot::Serial::DHParameter(0.3, 0, 0, 0, false));

    cbot::Serial::JointNames joint_names;
    joint_names.push_back("theta_1");
    joint_names.push_back("theta_2");
    joint_names.push_back("theta_3");
    joint_names.push_back("theta_4");
    joint_names.push_back("theta_5");
    joint_names.push_back("theta_6");

    cbot::Robot *robot = new cbot::Serial(dim, joint_names);
    // Set some initial joint positions, away from sigularity if all 0
    robot->set_joint_position("theta_1", 0);
    robot->set_joint_position("theta_2", 0.8);
    robot->set_joint_position("theta_3", 0.8);
    robot->set_joint_position("theta_4", 0.8);
    robot->set_joint_position("theta_5", 0.8);
    robot->set_joint_position("theta_6", 0.8);

    ControllerNode node(n, robot);
    ros::spin();
}
