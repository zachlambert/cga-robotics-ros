#include "joint_publisher.h"

#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>


JointPublisher::JointPublisher(ros::NodeHandle &n, const std::vector<std::string> &joints):
    joints(joints),
    joint_positions(joints.size(), 0),
    joint_velocities(joints.size(), 0),
    mode(Mode::INACTIVE)
{
    std::stringstream ss;
    std::string topic;
    joint_positions_pub.resize(joints.size());
    for (int i = 0; i < joints.size(); i++) {
        ss.str("");
        ss << joints[i] << "_controller/command";
        topic = ss.str();
        ss.clear();
        joint_positions_pub[i] = n.advertise<std_msgs::Float64>(topic, 1);
    }

    trajectory.joint_names = joints;

    trajectory_status.active = false;
    trajectory_status.progress = 0;
}

void JointPublisher::set_joint_velocities(const std::vector<double> &joint_velocities_in)
{
    mode = Mode::VELOCITY;
    std::copy(
        joint_velocities_in.begin(), joint_velocities_in.end(),
        joint_velocities.begin()
    );
}

void JointPublisher::load_trajectory(const trajectory_msgs::JointTrajectory &trajectory_in)
{
    // Don't rely on the joints being provided in the same order
    // in the trajectory as in the joints vector, so need to copy the joint
    // positions after checking the index mapping

    mode = Mode::TRAJECTORY;
    trajectory_status.active = true;
    trajectory_status.progress = 0;

    trajectory.header.stamp = ros::Time::now();
    trajectory.points.resize(trajectory_in.points.size());

    // If indexes[i] = j, this means that
    // joints[i] == trajectory_in.joint_names[j]
    std::vector<std::size_t> indexes(joints.size());
    for (std::size_t i = 0; i < joints.size(); i++) {
        for (std::size_t j = 0; j < trajectory.joint_names.size(); j++) {
            if (joints[i] == trajectory.joint_names[j]) {
                indexes[i] = j;
                continue;
            }
        }
    }
    for (std::size_t n = 0; n < trajectory.points.size(); n++) {
        for (std::size_t i = 0; i < joints.size(); i++) {
            trajectory.points[n].positions[i] = trajectory_in.points[n].positions[indexes[i]];
        }
    }
}

void JointPublisher::stop_trajectory() {
    trajectory_status.active = false;
    mode = Mode::INACTIVE;
}

void JointPublisher::loop(const ros::TimerEvent &timer)
{
    if (mode == Mode::VELOCITY) {
        double dt = (timer.current_real - timer.last_real).toSec();
        for (std::size_t i = 0; i < joints.size(); i++) {
            std::cout << "Vel " << i << " = " << joint_velocities[i] << std::endl;
            joint_positions[i] += joint_velocities[i]*dt;
        }

    } else if (mode == Mode::TRAJECTORY) {
        double t = (timer.current_real - trajectory.header.stamp).toSec();
        double u = t / trajectory.points.rbegin()->time_from_start.toSec();
        std::size_t n = u * trajectory.points.size();
        if (n < 0) n = 0;
        if (n >= trajectory.points.size()) {
            n = trajectory.points.size() - 1;
        }
        joint_positions = trajectory.points[n].positions;

        trajectory_status.progress = u;
        if (t > trajectory.points.rbegin()->time_from_start.toSec()) {
            mode = Mode::INACTIVE;
            trajectory_status.active = false;
        }
    } else { // INACTIVE
        // Do nothing
    }

    for (std::size_t i = 0; i < joints.size(); i++) {
        std_msgs::Float64 msg;
        msg.data = joint_positions[i];
        joint_positions_pub[i].publish(msg);
    }
}
