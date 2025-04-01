#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>

using std::placeholders::_1;
using namespace std::chrono_literals;

class ChatPublisher : public rclcpp::Node {
public:
    ChatPublisher() : Node("chat_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("chat_topic", 10);
        timer_ = this->create_wall_timer(2000ms, std::bind(&ChatPublisher::publish_message, this));
    }

private:
    void publish_message() {
        std_msgs::msg::String msg;
        std::cout << "Enter message: ";
        std::getline(std::cin, msg.data);
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChatPublisher>());
    rclcpp::shutdown();
    return 0;
}

