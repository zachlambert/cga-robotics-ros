#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class Node {
public:
    Node(ros::NodeHandle &n): joints_pos(N_JOINTS, 0), joints_vel(N_JOINTS, 0)
    {
        theta_vel_sub = n.subscribe(
            "joint_velocities", 1, &Node::joint_velocities_callback, this
        );

        loop_timer = n.createTimer(ros::Duration(1.0/20), &Node::loop, this);

        std::stringstream ss;
        std::string topic;
        for (int i = 0; i < N_JOINTS; i++) {
            ss.str("");
            ss << "theta_" << (i+1) << "_controller/command";
            topic = ss.str();
            ss.clear();
            theta_pub[i] = n.advertise<std_msgs::Float64>(topic, 1);
        }
    }

    void loop(const ros::TimerEvent &timer)
    {
        double dt = (timer.current_real - timer.last_real).toSec();
        for (std::size_t i = 0; i < N_JOINTS; i++) {
            joints_pos[i] += joints_vel[i]*dt;
            std_msgs::Float64 msg;
            msg.data = joints_pos[i];
            theta_pub[i].publish(msg);
        }
    }

    void joint_velocities_callback(const std_msgs::Float64MultiArray &joint_velocities)
    {
        std::size_t n = joint_velocities.data.size() < N_JOINTS ?
            joint_velocities.data.size(): N_JOINTS;
        for (int i = 0; i < n; i++) {
            joints_vel[i] = joint_velocities.data[i];
        }
    }

private:
    static constexpr std::size_t N_JOINTS = 3;
    std::vector<double> joints_pos, joints_vel;
    ros::Subscriber theta_vel_sub;
    ros::Timer loop_timer;
    ros::Publisher theta_pub[N_JOINTS];
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_velocity_integrator");
    ros::NodeHandle n;

    // ros::NodeHandle n_local("~");
    // n_local.getParam("base_radius", config.r_base);
    // n_local.getParam("ee_radius", config.r_ee);
    // n_local.getParam("upper_length", config.l_upper);
    // n_local.getParam("lower_length", config.l_lower);

    Node node(n);
    ros::spin();
}
