// src/odom_to_tf.cpp

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class OdomToTf : public rclcpp::Node
{
public:
    OdomToTf() : Node("odom_to_tf")
    {
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/ground_truth", 10, std::bind(&OdomToTf::odom_callback, this, std::placeholders::_1));
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        RCLCPP_INFO(this->get_logger(), "Odometry to TF node has been started.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;
        t.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(t);
        RCLCPP_INFO(this->get_logger(), "Publishing transform from odom to base_link");
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToTf>());
    rclcpp::shutdown();
    return 0;
}
