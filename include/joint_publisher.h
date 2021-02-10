#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/JointTrajectory.h>

class JointPublisher {
public:
    enum class Mode {
        VELOCITY,
        TRAJECTORY
    };

    JointPublisher(ros::NodeHandle &n, const std::vector<std::string> &joints);
    void set_joint_velocities(const std::vector<double> &joint_velocities_in);
    void load_trajectory(const trajectory_msgs::JointTrajectory &trajectory);
    void loop(const ros::TimerEvent &timer);

    std::vector<std::string> joints;
    std::vector<double> joint_positions, joint_velocities;
private:
    Mode mode;
    std::vector<ros::Publisher> joint_positions_pub;

    trajectory_msgs::JointTrajectory trajectory;
};
