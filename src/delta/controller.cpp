#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "cbot/delta.h"
// namespace cbot { using namespace cga_impl; }
namespace cbot { using namespace linalg_impl; }

#include "cga_robotics_ros/PoseCommand.h"
#include "cga_robotics_ros/VelocityCommand.h"
#include "cbot/conversions.h"
#include "joint_publisher.h"

enum class ControlMode {
    VELOCITY,
    TRAJECTORY
};

class Node {
public:
    Node(ros::NodeHandle &n, cbot::Delta::Config config): delta(config)
    {
        joint_names.push_back("theta_1");
        joint_names.push_back("theta_2");
        joint_names.push_back("theta_3");

        joint_publisher = std::unique_ptr<JointPublisher>(
            new JointPublisher(n, joint_names)
        );

        ee_twist_cmd_sub = n.subscribe(
            "ee_twist_sub", 1, &Node::ee_twist_cmd_callback, this
        );

        pose_command_server = n.advertiseService(
            "robot_command", &Node::pose_command_callback, this
        );

        velocity_command_server = n.advertiseService(
            "velocity_command", &Node::velocity_command_callback, this
        );

        loop_timer = n.createTimer(
            ros::Duration(1.0/20),
            &Node::loop,
            this
        );
    }

    bool pose_command_callback(
        cga_robotics_ros::PoseCommand::Request &req,
        cga_robotics_ros::PoseCommand::Request &res)
    {
        cbot::Pose goal = cbot::from_msg(req.goal.pose);
        double time = req.time.data;
        cbot::JointTrajectory trajectory;
        delta.calculate_trajectory(goal, time, trajectory);

        trajectory_msgs::JointTrajectory trajectory_msg = cbot::to_msg(trajectory);
        joint_publisher->load_trajectory(trajectory_msg);

        return true;
    }

    bool velocity_command_callback(
        cga_robotics_ros::VelocityCommand::Request &req,
        cga_robotics_ros::VelocityCommand::Request &res)
    {
        control_mode = ControlMode::VELOCITY;
        return true;
    }

    void ee_twist_cmd_callback(const geometry_msgs::TwistStamped &ee_twist_cmd)
    {
        this->ee_twist_cmd = cbot::from_msg(ee_twist_cmd.twist);
    }

    void loop(const ros::TimerEvent &timer)
    {
        if (control_mode == ControlMode::VELOCITY) {
            for (std::size_t i = 0; i < joint_publisher->joints.size(); i++) {
                delta.set_joint_position(
                    joint_publisher->joints[i],
                    joint_publisher->joint_positions[i]
                );
            }
            delta.set_twist(ee_twist_cmd);
            delta.update_joint_velocities();

            std::vector<double> joint_velocities(joint_publisher->joints.size());
            for (std::size_t i = 0; i < joint_publisher->joints.size(); i++) {
                joint_velocities[i] = delta.get_joints().at(
                    joint_publisher->joints[i]).velocity;
            }
            joint_publisher->set_joint_velocities(joint_velocities);
        }
        joint_publisher->loop(timer);
    }

private:
    cbot::Delta delta;
    std::vector<std::string> joint_names;
    std::unique_ptr<JointPublisher> joint_publisher;
    ControlMode control_mode;

    ros::ServiceServer pose_command_server;
    ros::ServiceServer velocity_command_server;

    ros::Subscriber ee_twist_cmd_sub;
    cbot::Twist ee_twist_cmd;

    ros::Timer loop_timer;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delta_fk");
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
