#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/vector3.hpp"  // 用x存正弦、y存方波、z存时间戳
#include <cmath>
using namespace std_msgs::msg;
class SignalPublisher : public rclcpp::Node
{
public:
    SignalPublisher() : Node("signal_publisher"), count_(0)
    {
        // 1. 创建发布者，话题名"raw_signals"，队列大小10
        publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("raw_signals", 10);
        
        // 2. 设置500Hz发布频率（周期0.002秒）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&SignalPublisher::timer_callback, this)
        );
        RCLCPP_INFO(this->get_logger(), "Signal publisher started (500Hz)");
    }

private:
    void timer_callback()
    {
        auto msg = geometry_msgs::msg::Vector3();
        double time = this->now().seconds();  // 获取当前时间（秒）
        
        // 3. 生成10Hz正弦信号
        msg.x = sin(2 * M_PI * 10 * time);
        
        // 4. 生成1Hz方波信号
        msg.y = (fmod(time, 1.0) < 0.5) ? 1.0 : -1.0;
        
        // 5. 发布信号
        publisher_->publish(msg);
        count_++;
    }

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalPublisher>());
    rclcpp::shutdown();
    return 0;
}