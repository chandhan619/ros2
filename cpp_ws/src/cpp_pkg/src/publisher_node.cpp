#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

#include <chrono>
#include <memory>
#include <string>

// This is a simple ROS 2 publisher node that publishes messages to a topic named "topic".
// It publishes a string message every 500 milliseconds, incrementing a count each time.
using namespace std::chrono_literals;

auto topic_1 = "float_topic";
auto topic_2 = "string_topic";

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode(): Node("publisher_node"), count_(0)
    {
        publisher_float_ = this->create_publisher<std_msgs::msg::Float32>(topic_1, 10);
        publisher_string_ = this->create_publisher<std_msgs::msg::String>(topic_2, 10);

        timer_ = this->create_wall_timer(
            500ms, std::bind(&PublisherNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "PublisherNode has been started.");

        counter= 0;
        value=0.0f;
    }

    void timer_callback()
    {
        auto message_float = std_msgs::msg::Float32();
        auto message_string = std_msgs::msg::String();

        message_float.data = value;
        message_string.data = "Hello, world! Count: " + std::to_string(counter);

        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message_float.data);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_string.data.c_str());

        publisher_float_->publish(message_float);
        publisher_string_->publish(message_string);

        counter++;
        value += 0.1f;
    }
private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_float_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_string_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter;
    float value;
    int count_;
};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}
