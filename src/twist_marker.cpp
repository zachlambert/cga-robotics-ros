#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

void configure_arrow(
    const geometry_msgs::PoseStamped pose,
    const geometry_msgs::Vector3 vector,
    visualization_msgs::Marker &arrow)
{
    arrow.header.frame_id = pose.header.frame_id;
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "";
    arrow.id = 0;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.pose.position = pose.pose.position;

    // Quaternion rotates from [1 0 0] to vector;
    Eigen::Vector3d a(1, 0, 0);
    Eigen::Vector3d b(vector.x, vector.y, vector.z);
    auto quat = Eigen::Quaterniond::FromTwoVectors(a, b.normalized());
    arrow.pose.orientation.x = quat.x();
    arrow.pose.orientation.y = quat.y();
    arrow.pose.orientation.z = quat.z();
    arrow.pose.orientation.w = quat.w();

    arrow.scale.x = b.norm();
    arrow.scale.y = 0.01;
    arrow.scale.z = 0.01;
}

class Node {
public:
    Node(ros::NodeHandle &n)
    {
        ee_pose_sub = n.subscribe(
            "ee_pose", 1, &Node::ee_pose_callback, this
        );
        ee_twist_sub = n.subscribe(
            "ee_twist", 1, &Node::ee_twist_callback, this
        );

        loop_timer = n.createTimer(ros::Duration(1.0/20), &Node::loop, this);

        linear_pub = n.advertise<visualization_msgs::Marker>(
            "ee_twist_linear", 1
        );
        angular_pub = n.advertise<visualization_msgs::Marker>(
            "ee_twist_angular", 1
        );
    }

    void ee_pose_callback(const geometry_msgs::PoseStamped &ee_pose)
    {
        this->ee_pose = ee_pose;
    }

    void ee_twist_callback(const geometry_msgs::TwistStamped &ee_twist)
    {
        this->ee_twist = ee_twist;
    }

    void loop(const ros::TimerEvent &timer)
    {
        visualization_msgs::Marker linear;
        configure_arrow(ee_pose, ee_twist.twist.linear, linear);
        linear.color.a = 1.0;
        linear.color.r = 0;
        linear.color.g = 1.0;
        linear.color.b = 1.0;
        linear_pub.publish(linear);

        visualization_msgs::Marker angular;
        configure_arrow(ee_pose, ee_twist.twist.angular, angular);
        angular.color.a = 1.0;
        angular.color.r = 1.0;
        angular.color.g = 1.0;
        angular.color.b = 0;
        angular_pub.publish(angular);
    }


private:
    ros::Subscriber ee_pose_sub, ee_twist_sub;
    geometry_msgs::PoseStamped ee_pose;
    geometry_msgs::TwistStamped ee_twist;
    ros::Publisher linear_pub, angular_pub;
    ros::Timer loop_timer;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_marker");
    ros::NodeHandle n;
    Node node(n);
    ros::spin();
}
