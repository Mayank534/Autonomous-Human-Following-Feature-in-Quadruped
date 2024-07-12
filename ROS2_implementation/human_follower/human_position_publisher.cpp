#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class HumanPositionPublisher : public rclcpp::Node
{
public:
    HumanPositionPublisher()
        : Node("human_position_publisher"), x_(0.0), y_(0.0), angle_(0.0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/human/position", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&HumanPositionPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // Update the human position
        double velocity = 1.0 / 20.0; // 1 m/min = 1/60 m/s
        x_ += velocity * cos(angle_);
        y_ += velocity * sin(angle_);

        // Check if it's time to turn right (every 15 seconds)
        // if (++tick_count_ == 15)
        // {
        //     angle_ -= M_PI / 2.0; // Turn right
        //     tick_count_ = 0;
        // }

        // Publish the position
        auto message = geometry_msgs::msg::Point();
        message.x = x_;
        message.y = y_;
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Publishing: '(%f, %f)'", message.x, message.y);
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double x_, y_, angle_;
    int tick_count_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanPositionPublisher>());
    rclcpp::shutdown();
    return 0;
}
