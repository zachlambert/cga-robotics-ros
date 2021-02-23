#include <ros/ros.h>
#include "nodes/state_publisher.h"
#include "cbot/delta.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delta_fk");
    ros::NodeHandle n;

    cbot::Delta::Dimensions dim;
    ros::NodeHandle n_local("~");
    n_local.getParam("base_radius", dim.r_base);
    n_local.getParam("ee_radius", dim.r_ee);
    n_local.getParam("upper_length", dim.l_upper);
    n_local.getParam("lower_length", dim.l_lower);

    cbot::Delta::JointNames joint_names;
    joint_names.theta.push_back("theta_1");
    joint_names.theta.push_back("theta_2");
    joint_names.theta.push_back("theta_3");
    joint_names.alpha.push_back("alpha_1");
    joint_names.alpha.push_back("alpha_2");
    joint_names.alpha.push_back("alpha_3");
    joint_names.beta.push_back("beta_1");
    joint_names.beta.push_back("beta_2");
    joint_names.beta.push_back("beta_3");
    joint_names.gamma.push_back("gamma_1");
    joint_names.gamma.push_back("gamma_2");
    joint_names.gamma.push_back("gamma_3");

    StatePublisherNode node(n, new cbot::Delta(dim, joint_names));
    ros::spin();
}
