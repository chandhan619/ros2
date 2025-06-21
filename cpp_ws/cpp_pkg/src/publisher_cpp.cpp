#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <chrono>
#include <string>
#include <memory>

using namespace std::chrono_literals;

auto topic_1 = "float_topic";
auto topic_2 = "string_topic";

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode() : Node("publisher_node")
  {
    pub_float_ = this->create_publisher<std_msgs::msg::Float32>(topic_1, 10);
    pub_string_ = this->create_publisher<std_msgs::msg::String>(topic_2, 10);
    period = 1000ms;
    timer_ = this->create_wall_timer(period, std::bind(&PublisherNode::timer_callback, this));
    counter = 0;
    value = 0.0f;
  }

  void timer_callback()
  {
    auto float_msg = std_msgs::msg::Float32();
    auto string_msg = std_msgs::msg::String();

    float_msg.data = value;
    string_msg.data = "Counter: " + std::to_string(counter);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%f' and '%s'", float_msg.data, string_msg.data.c_str());
    
        pub_float_->publish(float_msg);
        pub_string_->publish(string_msg);
    
        counter++;
        value += 1.0f;
    }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_float_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_string_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds period;
  int counter;
  float value;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto publisher_node = std::make_shared<PublisherNode>();
  rclcpp::spin(publisher_node);
  rclcpp::shutdown();
  return 0;
}
