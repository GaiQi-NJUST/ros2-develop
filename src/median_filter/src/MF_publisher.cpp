// median filter based on cpp
// ROS2 Publisher
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vector>
#include <algorithm>
#include <random>
#include <chrono>
#include <cmath>

class MedianFilterPublisher : public rclcpp::Node
{
public:
  MedianFilterPublisher()
  : Node("median_filter_publisher"),
    counter_(0)  // 添加计数器用于生成固定频率信号
  {
    // 创建发布者，发布到"raw_data"话题
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("raw_data", 10);
    
    // 设置定时器，每100毫秒发布一次数据
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MedianFilterPublisher::timer_callback, this));
    
    // 初始化随机数生成器
    std::random_device rd;
    gen_ = std::mt19937(rd());
    // 将噪声分布范围设置为较小的值，确保噪声幅值不会太高
    noise_dist_ = std::uniform_real_distribution<double>(-5.0, 5.0);
    
    RCLCPP_INFO(this->get_logger(), "Median Filter Publisher Node Started");
    RCLCPP_INFO(this->get_logger(), "Generating fixed frequency signal with moderate noise");
  }

private:
  void timer_callback()
  {
    // 创建消息
    auto message = std_msgs::msg::Float64();
    
    // 生成固定频率的信号 - 这里使用正弦波作为示例
    // 参数说明：
    // 50.0: 信号的基础值（偏移量）
    // 20.0: 信号的振幅
    // 0.1: 角频率（控制信号变化的快慢）
    double fixed_freq_signal = 50.0 + 20.0 * std::sin(0.1 * counter_);
    
    // 添加适度的随机噪声
    double noise = noise_dist_(gen_);
    
    // 组合信号和噪声
    message.data = fixed_freq_signal + noise;
    
    // 递增计数器，用于下一次计算
    counter_++;
    
    // 发布消息
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%f' (Signal: %f, Noise: %f)", 
                message.data, fixed_freq_signal, noise);
  }
  
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mt19937 gen_;
  std::uniform_real_distribution<double> noise_dist_;  // 噪声分布
  int counter_;  // 计数器，用于生成固定频率信号
};

int main(int argc, char * argv[])
{
  // 初始化ROS2
  rclcpp::init(argc, argv);
  
  // 创建并运行节点
  rclcpp::spin(std::make_shared<MedianFilterPublisher>());
  
  // 关闭ROS2
  rclcpp::shutdown();
  return 0;
}