#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "cbot/delta.h"
namespace cbot { using namespace linalg_impl; }

#include "cbot/conversions.h"

class Node {
public:
    Node(ros::NodeHandle &n, cbot::Delta::Config config): delta(config)
    {
        theta_vel_sub = n.subscribe(
            "joint_velocities", 1, &Node::joint_velocities_callback, this
        );

        loop_timer = n.createTimer(ros::Duration(1.0/20), &Node::loop, this);

        std::stringstream ss;
        std::string topic;
        for (int i = 0; i < 3; i++) {
            ss.str("");
            ss << "theta_" << (i+1) << "_controller/command";
            topic = ss.str();
            ss.clear();
            theta_pub[i] = n.advertise<std_msgs::Float64>(topic, 1);
        }

        ee_twist_pub = n.advertise<geometry_msgs::TwistStamped>("ee_twist", 1);

        std::fill_n(joints_pos.theta, 3, 0);
    }

    void loop(const ros::TimerEvent &timer)
    {
        double dt = (timer.current_real - timer.last_real).toSec();
        for (int i = 0; i < 3; i++) {
            joints_pos.theta[i] += joints_vel.theta[i]*dt;
            std_msgs::Float64 msg;
            msg.data = joints_pos.theta[i];
            theta_pub[i].publish(msg);
        }

        cbot::Twist twist;
        delta.fk_twist(joints_pos, joints_vel, twist);
        std::cout << twist.linear.x << std::endl;
        std::cout << twist.linear.y << std::endl;
        std::cout << twist.linear.z << std::endl;
        geometry_msgs::TwistStamped twist_msg;
        twist_msg.twist = cbot::to_msg(twist);
        twist_msg.header.frame_id = "base";
        twist_msg.header.stamp = ros::Time::now();
        ee_twist_pub.publish(twist_msg);
    }

    void joint_velocities_callback(const std_msgs::Float64MultiArray &joint_velocities)
    {
        std::size_t n = 3;
        if (joint_velocities.data.size() < 3) { n = joint_velocities.data.size(); }
        for (int i = 0; i < n; i++) {
            joints_vel.theta[i] = joint_velocities.data[i];
        }
    }

private:
    cbot::Delta delta;
    ros::Subscriber theta_vel_sub;
    cbot::Delta::Joints joints_pos, joints_vel;
    ros::Timer loop_timer;

    ros::Publisher theta_pub[3];
    ros::Publisher ee_twist_pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delta_fk_vel");
    ros::NodeHandle n;

    cbot::Delta::Config config;
    ros::NodeHandle n_local("~");
    n_local.getParam("base_radius", config.r_base);
    n_local.getParam("ee_radius", config.r_ee);
    n_local.getParam("upper_length", config.l_upper);
    n_local.getParam("lower_length", config.l_lower);

    Node node(n, config);
    ros::spin();
}
