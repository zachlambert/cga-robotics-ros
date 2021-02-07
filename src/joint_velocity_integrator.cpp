#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class Node {
public:
    Node(ros::NodeHandle &n, const std::vector<std::string> &joints):
        N_JOINTS(joints.size()), joints_pos(N_JOINTS, 0), joints_vel(N_JOINTS, 0)
    {
        joint_vel_sub = n.subscribe(
            "joint_velocities", 1, &Node::joint_velocities_callback, this
        );

        loop_timer = n.createTimer(ros::Duration(1.0/20), &Node::loop, this);

        std::stringstream ss;
        std::string topic;
        joint_pos_pub.resize(N_JOINTS);
        for (int i = 0; i < N_JOINTS; i++) {
            ss.str("");
            ss << joints[i] << "_controller/command";
            topic = ss.str();
            ss.clear();
            joint_pos_pub[i] = n.advertise<std_msgs::Float64>(topic, 1);
        }
    }

    void loop(const ros::TimerEvent &timer)
    {
        double dt = (timer.current_real - timer.last_real).toSec();
        for (std::size_t i = 0; i < N_JOINTS; i++) {
            joints_pos[i] += joints_vel[i]*dt;
            std_msgs::Float64 msg;
            msg.data = joints_pos[i];
            joint_pos_pub[i].publish(msg);
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
    const std::size_t N_JOINTS;
    std::vector<double> joints_pos, joints_vel;
    ros::Subscriber joint_vel_sub;
    ros::Timer loop_timer;
    std::vector<ros::Publisher> joint_pos_pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_velocity_integrator");
    ros::NodeHandle n;

    ros::NodeHandle n_local("~");
    std::string joints_string;
    n_local.getParam("joints", joints_string);

    std::vector<std::string> joints;
    std::stringstream ss(joints_string);
    std::string joint;
    while (ss >> joint) {
        joints.push_back(joint);
    }

    Node node(n, joints);
    ros::spin();
}
