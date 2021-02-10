#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cga_robotics_ros/PoseCommand.h>
#include <cga_robotics_ros/VelocityCommand.h>

class Node {
public:
    struct Config {
        std::string gripper_joint;
    };

    Node(ros::NodeHandle &n, Config config): config(config)
    {
        ee_pose_sub = n.subscribe(
            "ee_pose", 1, &Node::ee_pose_callback, this
        );
        joint_states_sub = n.subscribe(
            "joint_states", 1, &Node::joint_states_callback, this
        );
        ee_twist_pub = n.advertise<geometry_msgs::TwistStamped>(
            "ee_twist", 1
        );
    }

    void ee_pose_callback(const geometry_msgs::PoseStamped pose_msg)
    {
        ee_pose = pose_msg;
    }

    void joint_states_callback(const sensor_msgs::JointState &joint_states)
    {
        for (std::size_t i = 0; i < joint_states.name.size(); i++) {
            if (joint_states.name[i] == config.gripper_joint) {
                gripper_angle = joint_states.position[i];
                return;
            }
        }
    }

private:
    Config config;

    ros::Subscriber joint_states_sub;
    ros::Subscriber ee_pose_sub;
    ros::Publisher ee_twist_pub;

    geometry_msgs::PoseStamped ee_pose;
    double gripper_angle;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delta_fk");
    ros::NodeHandle n;

    Node::Config config;
    config.gripper_joint = "gripper";
    // ros::NodeHandle n_local("~");
    // n_local.getParam("gripper_joint", config.gripper_joint);

    Node node(n, config);
    ros::spin();
}
