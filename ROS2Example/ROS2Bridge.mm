#import "ROS2Bridge.h"

// From https://github.com/ros2/examples/blob/rolling/rclcpp/topics/minimal_publisher/lambda.cpp

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

// https://github.com/ros2/examples/blob/rolling/rclcpp/topics/minimal_subscriber/lambda.cpp

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      [this](std_msgs::msg::String::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

@implementation ROS2Bridge
{
}

- (void)startPublishing
{
    rclcpp::init(0, NULL);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
}

- (void)startListening
{
    rclcpp::init(0, NULL);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
}

@end
