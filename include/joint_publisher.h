#ifndef JOINT_PUBLISHER_H
#define JOINT_PUBLISHER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/JointTrajectory.h>

/* JointPublisher: Responsible for publishing joint position commands
 * Three modes:
 * - INACTIVE: Keep positions constant.
 * - VELOCITY: Integrate positions from velocities.
 * - TRAJECTORY: Interpolate along a joint trajectory.
 * In each case, positions are published at a regular interval, with the
 * loop function.
 * 
 * The rules for how the joint publisher works are as follows:
 * - Initialises with INACTIVE mode.
 * - Whenever joint velocities are set, it moves to VELOCITY mode,
 *   possibly interrupting a trajectory.
 * - Whenever a trajectory is loaded, it moves to TRAJECTORY mode.
 *
 * Finally, a function is needed for providing the status of the trajectory.
 * Is it active? and what is the progress (0 -> 1)?
 */

class JointPublisher {
public:
    enum class Mode {
        INACTIVE,
        VELOCITY,
        TRAJECTORY
    };
    struct TrajectoryStatus {
        double progress;
        bool active;
    };

    JointPublisher(ros::NodeHandle &n, const std::vector<std::string> &joints);

    void set_joint_velocities(const std::vector<double> &joint_velocities_in);
    void load_trajectory(const trajectory_msgs::JointTrajectory &trajectory);
    void stop_trajectory();
    const TrajectoryStatus &get_trajectory_status()const { 
        return trajectory_status;
    }

    void loop(const ros::TimerEvent &timer);

    std::vector<std::string> joints;
    std::vector<double> joint_positions, joint_velocities;
private:
    Mode mode;
    std::vector<ros::Publisher> joint_positions_pub;

    trajectory_msgs::JointTrajectory trajectory;
    TrajectoryStatus trajectory_status;
};

#endif
