#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include "cbot/delta.h"
namespace cbot { using namespace cga_impl; }

#include "cbot/conversions.h"

class Node {
public:
    Node(ros::NodeHandle &n, cbot::Delta::Config config): delta(config)
    {
        joint_states_in_sub = n.subscribe(
            "joint_states_in", 1, &Node::joint_states_callback, this
        );
        joint_states_out_pub = n.advertise<sensor_msgs::JointState>(
            "joint_states_out", 1
        );

        theta_names[0] = "theta_1";
        theta_names[1] = "theta_2";
        theta_names[2] = "theta_3";
        alpha_names[0] = "alpha_1";
        alpha_names[1] = "alpha_2";
        alpha_names[2] = "alpha_3";
        beta_names[0] = "beta_1";
        beta_names[1] = "beta_2";
        beta_names[2] = "beta_3";
        gamma_names[0] = "gamma_1";
        gamma_names[1] = "gamma_2";
        gamma_names[2] = "gamma_3";

        ee_pose_pub = n.advertise<geometry_msgs::Pose>(
            "ee_pose", 1
        );
    }

    void joint_states_callback(const sensor_msgs::JointState &joint_states_in)
    {
        // Read theta joints in. Don't rely on the joints being provided
        // in any specific order, so have to search joint names.
        cbot::Delta::Joints joints_pos;
        for (int i = 0; i < 3; i++) {
            for (std::size_t j = 0; j < joint_states_in.name.size(); j++) {
                if (theta_names[i] == joint_states_in.name[j]) {
                    joints_pos.theta[i] = joint_states_in.position[j];
                    break;
                }
            }
        }

        // Get the end effector pose and dependent joint values
        cbot::Pose pose;
        cbot::Delta::JointsDep joints_pos_dep;
        delta.fk_pose(joints_pos, joints_pos_dep, pose);

        // Copy the joint_states_in message, then append the dependent
        // joints, which shouldn't be present in the original message

        sensor_msgs::JointState joint_states(joint_states_in);

        for (int i = 0; i < 3; i++) {
            joint_states.name.push_back(alpha_names[i]);
            joint_states.position.push_back(joints_pos_dep.alpha[i]);
        }
        for (int i = 0; i < 3; i++) {
            joint_states.name.push_back(beta_names[i]);
            joint_states.position.push_back(joints_pos_dep.beta[i]);
        }
        for (int i = 0; i < 3; i++) {
            joint_states.name.push_back(gamma_names[i]);
            joint_states.position.push_back(joints_pos_dep.gamma[i]);
        }
        joint_states.velocity.resize(joint_states.name.size());
        joint_states.effort.resize(joint_states.name.size());

        joint_states_out_pub.publish(joint_states);
        ee_pose_pub.publish(cbot::to_msg(pose));
    }

private:
    cbot::Delta delta;

    ros::Subscriber joint_states_in_sub;
    ros::Publisher joint_states_out_pub;
    ros::Publisher ee_pose_pub;

    std::string theta_names[3];
    std::string alpha_names[3];
    std::string beta_names[3];
    std::string gamma_names[3];
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delta_fk");
    ros::NodeHandle n;

    cbot::Delta::Config config;
    n.param("delta/config/base_radius", config.r_base, 0.15);
    n.param("delta/config/ee_radius", config.r_ee, 0.05);
    n.param("delta/config/upper_length", config.l_upper, 0.3);
    n.param("delta/config/lower_length", config.l_lower, 0.3);

    Node node(n, config);
    ros::spin();
}
