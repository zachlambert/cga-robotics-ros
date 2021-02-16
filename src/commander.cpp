#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <cga_robotics_ros/PoseCommand.h>
#include <cga_robotics_ros/VelocityCommand.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cga_robotics_ros/TrajectoryAction.h>
#include "joystick_listener.h"

enum class TaskType {
    POSE,
    GRIPPER
};
struct Task {
    TaskType type;
    union {
        geometry_msgs::Pose goal_pose;
        double goal_gripper_angle;
    };
    Task(const geometry_msgs::Pose &pose):
        type(TaskType::POSE), goal_pose(pose) {}
    Task(const double gripper_angle):
        type(TaskType::GRIPPER), goal_gripper_angle(gripper_angle) {}
};

enum class ControlMode {
    MANUAL,
    TASKS
};

class Node {
public:
    struct Config {
        std::string gripper_joint;
    };

    Node(ros::NodeHandle &n, Config config):
        config(config),
        joystick_listener(n),
        mode(ControlMode::MANUAL),
        trajectory_client("trajectory", true)
    {
        ee_pose_sub = n.subscribe(
            "ee_pose", 1, &Node::ee_pose_callback, this
        );
        joint_states_sub = n.subscribe(
            "joint_states", 1, &Node::joint_states_callback, this
        );
        ee_twist_cmd_pub = n.advertise<geometry_msgs::TwistStamped>(
            "ee_twist_cmd", 1
        );

        loop_timer = n.createTimer(
            ros::Duration(1.0/20),
            &Node::loop,
            this
        );

        ROS_INFO("Waiting for trajectory server to start.");
        trajectory_client.waitForServer();
    }

    void start_next_task() {
        if (!tasks.empty()) {
            if (tasks.front().type == TaskType::POSE) {
                cga_robotics_ros::TrajectoryGoal goal;
                goal.pose = tasks.front().goal_pose;
                goal.max_linear_speed = 0.25;
                goal.max_angular_speed = 1;
                std::cout << "Pose goal: " << std::endl << goal.pose << std::endl;
                trajectory_client.sendGoal(goal);

            } else if (tasks.front().type == TaskType::GRIPPER) {
                // TODO
                std::cout << "Gripper task: " << tasks.front().goal_gripper_angle << std::endl;
            }
        } else {
            mode = ControlMode::MANUAL;
        }
    }

    void loop(const ros::TimerEvent &timer)
    {
        if (mode == ControlMode::MANUAL) {
            if (joystick_listener.query_button_state(JoyButton::A)
                    == JoyButtonState::PRESSED)
            {
                tasks.emplace(ee_pose);
            }
            if (joystick_listener.query_button_state(JoyButton::X)
                    == JoyButtonState::PRESSED)
            {
                tasks.emplace(gripper_angle);
            }
            if (joystick_listener.query_button_state(JoyButton::Y)
                    == JoyButtonState::PRESSED)
            {
                mode = ControlMode::TASKS;
                std::cout << "Starting tasks" << std::endl;
                start_next_task();
                return;
            }

            ee_twist_cmd.header.stamp = ros::Time::now();

            // Set linear velocity
            ee_twist_cmd.twist.linear.x =
                0.25*joystick_listener.query_axis(JoyAxis::LEFT_VERTICAL);
            ee_twist_cmd.twist.linear.y =
                0.25*joystick_listener.query_axis(JoyAxis::LEFT_HORIZONTAL);
            ee_twist_cmd.twist.linear.z =
                0.25*(joystick_listener.query_axis(JoyAxis::LT)
                - joystick_listener.query_axis(JoyAxis::RT));

            // Don't bother with angular velocity for now, since delta doesn't use
            // it. Later on, set angular velocity using euler velocities.
            // Controller is responsible for the coordinate transformation to
            // angular velocity.

            ee_twist_cmd_pub.publish(ee_twist_cmd);

        } else {
            if (tasks.front().type == TaskType::POSE &&
                trajectory_client.getState().isDone())
            {
                tasks.pop();
                start_next_task();
            } else if(tasks.front().type == TaskType::GRIPPER &&
                true /* TODO:gripper_client.getState().isDone()*/ )
            {
                tasks.pop();
                start_next_task();
            }
        }
    }

    void ee_pose_callback(const geometry_msgs::PoseStamped pose_msg)
    {
        ee_pose = pose_msg.pose;
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
    ControlMode mode;
    std::queue<Task> tasks;

    ros::Subscriber joint_states_sub;
    ros::Subscriber ee_pose_sub;
    ros::Publisher ee_twist_cmd_pub;
    ros::Timer loop_timer;

    actionlib::SimpleActionClient<cga_robotics_ros::TrajectoryAction> trajectory_client;

    geometry_msgs::Pose ee_pose;
    geometry_msgs::TwistStamped ee_twist_cmd;
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
