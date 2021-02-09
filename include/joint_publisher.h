#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class JointPublisher {
public:
    enum class Mode {
        VELOCITY,
        POSITION
    };

    JointPublisher(ros::NodeHandle &n, const std::vector<std::string> &joints);
    void loop(const ros::TimerEvent &timer);
    void set_joint_velocities(const std::vector<double> &joint_velocities_in);
    void set_joint_positions(const std::vector<double> &joint_positions_in);
    void set_mode(Mode mode);

private:
    Mode mode;
    const std::size_t N_JOINTS;
    std::vector<double> joint_positions, joint_velocities;
    std::vector<ros::Publisher> joint_positions_pub;
};
