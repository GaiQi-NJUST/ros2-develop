#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/vector3.hpp"
using namespace std_msgs::msg;
class SignalSubscriber : public rclcpp::Node
{
public:
    SignalSubscriber() : Node("signal_subscriber")
    {
        // 1. 创建订阅者，订阅"raw_signals"话题，接收后触发回调
        subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "raw_signals",
            10,
            std::bind(&SignalSubscriber::signal_callback, this, std::placeholders::_1)
        );
        
        // 2. 创建发布者，发布处理后信号（话题名"processed_signal"）
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("processed_signal", 10);
        RCLCPP_INFO(this->get_logger(), "Signal subscriber started");
    }

private:
    void signal_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        auto processed_msg = std_msgs::msg::Float64();
        
        // 3. 判断正弦（msg->x）与方波（msg->y）是否同号
        if (msg->x * msg->y > 0)  
        {
            processed_msg.data = msg->x;  
        }
        else
        {
            processed_msg.data = 0.0;  
        }
        
        // 4. 发布处理后信号
        publisher_->publish(processed_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Raw Sin: %.2f, Raw Square: %.2f, Processed: %.2f",
            msg->x, msg->y, processed_msg.data);
    }

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalSubscriber>());
    rclcpp::shutdown();
    return 0;
}