#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class HumanFollower : public rclcpp::Node
{
public:
    HumanFollower()
        : Node("human_follower"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/copub", 10, std::bind(&HumanFollower::position_callback, this, std::placeholders::_1));
    }

private:
    void position_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        double relative_x = msg->x;
        double relative_y = msg->y;

        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform not found: %s", ex.what());
            return;
        }

        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double absolute_x = transform.transform.translation.x + relative_x * cos(yaw) - relative_y * sin(yaw);
        double absolute_y = transform.transform.translation.y + relative_x * sin(yaw) + relative_y * cos(yaw);

        auto goal_pose = geometry_msgs::msg::PoseStamped();
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = this->get_clock()->now();
        goal_pose.pose.position.x = absolute_x;
        goal_pose.pose.position.y = absolute_y;
        goal_pose.pose.position.z = 0.0;
        goal_pose.pose.orientation = transform.transform.rotation;

        publisher_->publish(goal_pose);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanFollower>());
    rclcpp::shutdown();
    return 0;
}
