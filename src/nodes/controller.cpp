#include "nodes/controller.h"

#include "cbot/conversions.h"

ControllerNode::ControllerNode(ros::NodeHandle &n, cbot::Robot *robot):
    robot(robot),
    joint_publisher(n, robot->get_independent_joint_names()),
    trajectory_server(
        n, "trajectory",
        boost::bind(&ControllerNode::trajectory_callback, this, _1),
        false)
{
    ee_twist_cmd_sub = n.subscribe(
        "ee_twist_cmd", 1, &ControllerNode::ee_twist_cmd_callback, this
    );

    velocity_timer = n.createTimer(
        ros::Duration(1.0/20),
        &ControllerNode::loop_velocity,
        this
    );

    trajectory_server.start();
}

void ControllerNode::trajectory_callback(const cga_robotics_ros::TrajectoryGoalConstPtr &goal)
{
    velocity_timer.stop();

    for (std::size_t i = 0; i < joint_publisher.joints.size(); i++) {
        robot->set_joint_position(joint_publisher.joints[i],
            joint_publisher.joint_positions[i]);
    }
    cbot::Pose pose_goal = cbot::from_msg(goal->pose);

    cbot::TrajectoryConstraints constraints;
    constraints.max_linear_speed = goal->max_linear_speed;
    constraints.max_angular_speed = goal->max_angular_speed;
    robot->set_trajectory_constraints(constraints);

    if (!robot->calculate_trajectory(pose_goal)) {
        trajectory_server.setAborted();
        velocity_timer.start();
        return;
    }

    trajectory_msgs::JointTrajectory trajectory_msg = cbot::to_msg(robot->get_trajectory());
    joint_publisher.load_trajectory(trajectory_msg);

    ros::Rate rate(20);
        cga_robotics_ros::TrajectoryFeedback feedback;
    while (joint_publisher.get_trajectory_status().active) {
        if (trajectory_server.isPreemptRequested() || !ros::ok()) {
            trajectory_server.setPreempted();
            velocity_timer.start();
            return;
        }
        joint_publisher.update_from_trajectory();
        joint_publisher.publish();

        feedback.progress = joint_publisher.get_trajectory_status().progress;
        trajectory_server.publishFeedback(feedback);
        rate.sleep();
    }
    trajectory_server.setSucceeded();
    velocity_timer.start();
}

void ControllerNode::ee_twist_cmd_callback(const geometry_msgs::TwistStamped &ee_twist_cmd)
{
    this->ee_twist_cmd = cbot::from_msg(ee_twist_cmd.twist);
    if (trajectory_server.isActive()) {
        trajectory_server.setPreempted();
    }
}

void ControllerNode::loop_velocity(const ros::TimerEvent &timer)
{
    if (trajectory_server.isActive()) {
        std::cout << "Looping velocity" << std::endl;
    }
    for (std::size_t i = 0; i < joint_publisher.joints.size(); i++) {
        robot->set_joint_position(
            joint_publisher.joints[i],
            joint_publisher.joint_positions[i]
        );
    }
    robot->set_twist(ee_twist_cmd);
    robot->update_joint_velocities();

    std::vector<double> joint_velocities(joint_publisher.joints.size());
    for (std::size_t i = 0; i < joint_publisher.joints.size(); i++) {
        joint_velocities[i] = robot->get_joint_velocity(joint_publisher.joints[i]);
    }
    joint_publisher.set_joint_velocities(joint_velocities);
    joint_publisher.update_from_velocity(
        timer.current_real - timer.last_real);
    joint_publisher.publish();
}
