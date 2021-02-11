#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/server/simple_action_server.h>
#include <cga_robotics_ros/TrajectoryAction.h>

#include "cbot/delta.h"


#include "cbot/conversions.h"
#include "joint_publisher.h"

enum class ControlMode {
    VELOCITY,
    TRAJECTORY
};

class Node {
public:
    Node(
        ros::NodeHandle &n,
        const cbot::Delta::Dimensions &dim,
        const cbot::Delta::JointNames &joint_names):
        delta(dim, joint_names),
        joint_publisher(n, delta.get_independent_joint_names()),
        trajectory_server(
            n, "trajectory",
            boost::bind(&Node::trajectory_callback, this, _1),
            false)
    {
        ee_twist_cmd_sub = n.subscribe(
            "ee_twist_cmd", 1, &Node::ee_twist_cmd_callback, this
        );

        loop_timer = n.createTimer(
            ros::Duration(1.0/50),
            &Node::loop,
            this
        );

        // trajectory_server.start();
    }

    void trajectory_callback(const cga_robotics_ros::TrajectoryGoalConstPtr &goal)
    {
        cbot::Pose pose_goal = cbot::from_msg(goal->pose);
        double time = goal->time;

        cbot::JointTrajectory trajectory;
        delta.calculate_trajectory(pose_goal, time, trajectory);

        trajectory_msgs::JointTrajectory trajectory_msg = cbot::to_msg(trajectory);
        joint_publisher.load_trajectory(trajectory_msg);
    }

    void ee_twist_cmd_callback(const geometry_msgs::TwistStamped &ee_twist_cmd)
    {
        this->ee_twist_cmd = cbot::from_msg(ee_twist_cmd.twist);
        control_mode = ControlMode::VELOCITY;
    }

    void loop(const ros::TimerEvent &timer)
    {
        if (control_mode == ControlMode::VELOCITY) {
            for (std::size_t i = 0; i < joint_publisher.joints.size(); i++) {
                delta.set_joint_position(
                    joint_publisher.joints[i],
                    joint_publisher.joint_positions[i]
                );
            }
            std::cout << "Twist cmd = " << ee_twist_cmd << std::endl;
            delta.set_twist(ee_twist_cmd);
            delta.update_joint_velocities();

            std::vector<double> joint_velocities(joint_publisher.joints.size());
            for (std::size_t i = 0; i < joint_publisher.joints.size(); i++) {
                joint_velocities[i] = delta.get_joints().at(
                    joint_publisher.joints[i]).velocity;
                std::cout << i << " = " << joint_velocities[i] << std::endl;
            }
            joint_publisher.set_joint_velocities(joint_velocities);
        }

        joint_publisher.loop(timer);

        if (control_mode == ControlMode::TRAJECTORY) {
            if (trajectory_server.isPreemptRequested()) {
                joint_publisher.stop_trajectory();
                control_mode = ControlMode::VELOCITY;
                trajectory_server.setPreempted();
                return;
            }
            bool active = joint_publisher.get_trajectory_status().active;
            double progress = joint_publisher.get_trajectory_status().progress;
            if (!active) {
                trajectory_server.setSucceeded();
            } else {
                cga_robotics_ros::TrajectoryFeedback trajectory_feedback;
                trajectory_feedback.progress = progress;
                trajectory_server.publishFeedback(trajectory_feedback);
            }
        }
    }

private:
    cbot::Delta delta;
    JointPublisher joint_publisher;
    ControlMode control_mode;

    actionlib::SimpleActionServer<cga_robotics_ros::TrajectoryAction> trajectory_server;

    ros::Subscriber ee_twist_cmd_sub;
    cbot::Twist ee_twist_cmd;

    ros::Timer loop_timer;
};

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
    // TODO: Tidy this up
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

    Node node(n, dim, joint_names);
    ros::spin();
}
