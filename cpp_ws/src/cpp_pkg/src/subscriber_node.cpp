#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"


#include <memory>


// This is a simple ROS 2 publisher node that publishes messages to a topic named "topic".
// It publishes a string message every 500 milliseconds, incrementing a count each time.
using namespace std::chrono_literals;

auto topic_1 = "float_topic";
auto topic_2 = "string_topic";

using std::placeholders::_1; 

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode() : Node("subscriber_node")
    {
        subscriber_float_ = this->create_subscription<std_msgs::msg::Float32>(
            topic_1, 10, std::bind(&SubscriberNode::float_callback, this, _1));
        subscriber_string_ = this->create_subscription<std_msgs::msg::String>(
            topic_2, 10, std::bind(&SubscriberNode::string_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "SubscriberNode has been started.");
    }
    void float_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received float message: '%f'", msg->data);
    }
    void string_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received string message: '%s'", msg->data.c_str());
    }
private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_float_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_string_;
};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
// This code defines a ROS 2 subscriber node that listens to two topics: "float_topic" and "string_topic".