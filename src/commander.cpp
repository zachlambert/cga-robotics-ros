#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cga_robotics_ros/PoseCommand.h>
#include <cga_robotics_ros/VelocityCommand.h>
#include "joystick_listener.h"

class Node {
public:
    struct Config {
        std::string gripper_joint;
    };

    Node(ros::NodeHandle &n, Config config):
        config(config), joystick_listener(n)
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

        loop_timer = n.createTimer(
            ros::Duration(1.0/20),
            &Node::loop,
            this
        );
    }

    void loop(const ros::TimerEvent &timer)
    {
        if (joystick_listener.query_button_state(JoyButton::Y) == JoyButtonState::PRESSED) {

        }

        ee_twist.header.stamp = ros::Time::now();

        // Set linear velocity
        ee_twist.twist.linear.x =
            joystick_listener.query_axis(JoyAxis::LEFT_VERTICAL);
        ee_twist.twist.linear.y =
            joystick_listener.query_axis(JoyAxis::LEFT_HORIZONTAL);
        ee_twist.twist.linear.z =
            joystick_listener.query_axis(JoyAxis::RT)
            - joystick_listener.query_axis(JoyAxis::LT);

        // Don't bother with angular velocity for now, since delta doesn't use
        // it. Later on, set angular velocity using euler velocities.
        // Controller is responsible for the coordinate transformation to
        // angular velocity.

        ee_twist_pub.publish(ee_twist);
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
    JoystickListener joystick_listener;

    ros::Subscriber joint_states_sub;
    ros::Subscriber ee_pose_sub;
    ros::Publisher ee_twist_pub;
    ros::Timer loop_timer;

    geometry_msgs::PoseStamped ee_pose;
    geometry_msgs::TwistStamped ee_twist;
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
