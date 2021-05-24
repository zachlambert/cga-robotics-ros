#include "nodes/controller.h"

#include "cbot/conversions.h"

ControllerNode::ControllerNode(ros::NodeHandle &n, cbot::Robot *robot):
    robot(robot),
    joint_publisher(n, "theta_controller", robot->get_independent_joint_names()),
    gripper_publisher(n, "gripper_controller", {"gripper"}),
    trajectory_server(
        n, "trajectory",
        boost::bind(&ControllerNode::trajectory_callback, this, _1),
        false),
    gripper_server(
        n, "gripper",
        boost::bind(&ControllerNode::gripper_callback, this, _1),
        false)
{
    ee_twist_cmd_sub = n.subscribe(
        "ee_twist_cmd", 1, &ControllerNode::ee_twist_cmd_callback, this
    );

    gripper_cmd_sub = n.subscribe(
        "gripper_cmd", 1, &ControllerNode::gripper_cmd_callback, this
    );
    gripper_cmd = 0;

    std::vector<double> initial_joint_positions(joint_publisher.joints.size());
    for (std::size_t i = 0; i < joint_publisher.joints.size(); i++) {
        initial_joint_positions[i] = robot->get_joint_position(joint_publisher.joints[i]);
    }
    joint_publisher.set_joint_positions(initial_joint_positions);

    gripper_publisher.set_joint_positions({0});

    // Start timer and servers

    velocity_timer = n.createTimer(
        ros::Duration(1.0/20),
        &ControllerNode::loop_velocity,
        this
    );

    trajectory_server.start();
    gripper_server.start();

    skip_velocity_timer = false;
}

void ControllerNode::trajectory_callback(const cga_robotics_ros::TrajectoryGoalConstPtr &goal)
{
    velocity_timer.stop();
    skip_velocity_timer = true;

    for (std::size_t i = 0; i < joint_publisher.joints.size(); i++) {
        robot->set_joint_position(joint_publisher.joints[i],
            joint_publisher.joint_positions[i]);
    }
    cbot::Pose pose_goal = cbot::from_msg(goal->pose);

    cbot::TrajectoryConstraints constraints;
    constraints.max_linear_speed = goal->max_linear_speed;
    constraints.max_angular_speed = goal->max_angular_speed;
    constraints.max_joint_speed = 6;
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

void ControllerNode::gripper_callback(const cga_robotics_ros::GripperGoalConstPtr &goal)
{
    velocity_timer.stop();
    skip_velocity_timer = true;

    double goal_angle = goal->angle;

    // Fit linear trajectory for gripper

    double initial_angle = gripper_publisher.joint_positions[0];
    double delta_angle = goal_angle - initial_angle;
    if (delta_angle > M_PI) delta_angle -= 2*M_PI;
    if (delta_angle < -M_PI) delta_angle += 2*M_PI;

    static constexpr double delta_t = 1e-2;
    double T = std::fabs(delta_angle/goal->speed);
    std::size_t N = (T/delta_t)+1;

    trajectory_msgs::JointTrajectory trajectory_msg;
    trajectory_msg.joint_names = {"gripper"};
    trajectory_msg.points.resize(N);
    for (std::size_t i = 0; i < N; i++) {
        double u = ((double)(i+1))/N;
        trajectory_msg.points[i].positions.resize(1);
        trajectory_msg.points[i].positions[0] = initial_angle + delta_angle*u;
        trajectory_msg.points[i].time_from_start = ros::Duration(u*T);
    }

    gripper_publisher.load_trajectory(trajectory_msg);

    ros::Rate rate(20);
    cga_robotics_ros::GripperFeedback feedback;
    while (gripper_publisher.get_trajectory_status().active) {
        if (gripper_server.isPreemptRequested() || !ros::ok()) {
            gripper_server.setPreempted();
            velocity_timer.start();
            return;
        }
        gripper_publisher.update_from_trajectory();
        gripper_publisher.publish();

        feedback.progress = gripper_publisher.get_trajectory_status().progress;
        gripper_server.publishFeedback(feedback);
        rate.sleep();
    }
    gripper_server.setSucceeded();
    velocity_timer.start();
}

void ControllerNode::ee_twist_cmd_callback(const geometry_msgs::TwistStamped &ee_twist_cmd)
{
    this->ee_twist_cmd = cbot::from_msg(ee_twist_cmd.twist);
    if (trajectory_server.isActive()) {
        trajectory_server.setPreempted();
    }
}

void ControllerNode::gripper_cmd_callback(const std_msgs::Float64 gripper_cmd)
{
    this->gripper_cmd = gripper_cmd.data;
    if (gripper_server.isActive()) {
        gripper_server.setPreempted();
    }
}

void ControllerNode::loop_velocity(const ros::TimerEvent &timer)
{
    if (skip_velocity_timer) {
        // Do this after finishing trajectory, since timer.last is far
        // in the past.
        skip_velocity_timer = false;
        return;
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

    // Update robot with new joints and validate state
    for (std::size_t i = 0; i < joint_publisher.joints.size(); i++) {
        robot->set_joint_position(
            joint_publisher.joints[i],
            joint_publisher.joint_positions[i]
        );
    }
    if (!robot->is_valid()) {
        joint_publisher.revert_from_velocity();
        ROS_INFO("Invalid next state");
        return;
    }
    joint_publisher.publish();

    gripper_publisher.set_joint_velocities({gripper_cmd});
    gripper_publisher.update_from_velocity(timer.current_real - timer.last_real);
    gripper_publisher.publish();
}
