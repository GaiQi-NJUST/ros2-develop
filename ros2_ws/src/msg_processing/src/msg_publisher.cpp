#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MsgPublisher : public rclcpp::Node {
public:
    MsgPublisher() : Node("msg_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MsgPublisher::publish_msg, this));
    }

private:
    void publish_msg() {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello from publisher!";
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published: '%s'", msg.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MsgPublisher>());
    rclcpp::shutdown();
    return 0;
}