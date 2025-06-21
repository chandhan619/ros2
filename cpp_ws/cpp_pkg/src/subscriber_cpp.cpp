#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <chrono>
#include <string>
#include <memory>

using namespace std::chrono_literals;

auto topic_1 = "float_topic";
auto topic_2 = "string_topic";
class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode() : Node("subscriber_node")
  {
    sub_float_ = this->create_subscription<std_msgs::msg::Float32>(
      topic_1, 10, std::bind(&SubscriberNode::float_callback, this, std::placeholders::_1));
    sub_string_ = this->create_subscription<std_msgs::msg::String>(
      topic_2, 10, std::bind(&SubscriberNode::string_callback, this, std::placeholders::_1));
  }
  void float_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received float: '%f'", msg->data);
  }
  void string_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received string: '%s'", msg->data.c_str());
  }
private:
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_float_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_string_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto subscriber_node = std::make_shared<SubscriberNode>();
  rclcpp::spin(subscriber_node);
  rclcpp::shutdown();
  return 0;
}