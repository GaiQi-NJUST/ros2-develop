//lowpass filter publisher
//自拟数据发生器，形式为固定频率的数据加上随机噪声,使用滤波器进行数据处理

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <random>
#include <iostream>
#include <cmath>

class LowPassFilterPublisher : public rclcpp::Node
{
public:
  LowPassFilterPublisher() 
  : Node("lowpass_filter_publisher"),
    counter_(0)  // 添加计数器用于生成固定频率信号
  {
    // 创建发布者，发布到 "raw_data" 话题
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("raw_data", 10);
    
    // 设置发布频率为 10 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LowPassFilterPublisher::timer_callback, this)
    );
    
    // 初始化随机数生成器
    std::random_device rd;
    gen_ = std::mt19937(rd());
    // 改为均匀分布噪声，与MF类似
    dist_ = std::uniform_real_distribution<double>(-5.0, 5.0);
    
    RCLCPP_INFO(this->get_logger(), "LowPassFilterPublisher has been started");
    RCLCPP_INFO(this->get_logger(), "Generating fixed frequency signal with uniform noise");
  }

private:
  void timer_callback()
  {
    // 生成固定频率的信号，与MF类似的形式
    // 参数说明：
    // 50.0: 信号的基础值（偏移量）
    // 20.0: 信号的振幅
    // 0.1: 角频率（控制信号变化的快慢）
    double fixed_freq_signal = 50.0 + 20.0 * std::sin(0.1 * counter_);
    
    // 添加随机噪声
    double noise = dist_(gen_);
    double raw_data = fixed_freq_signal + noise;
    
    // 递增计数器，用于下一次计算
    counter_++;
    
    // 创建消息并发布
    auto message = std_msgs::msg::Float64();
    message.data = raw_data;
    publisher_->publish(message);
    
    // 添加详细的日志输出，与MF类似
    RCLCPP_INFO(this->get_logger(), "Publishing: '%f' (Signal: %f, Noise: %f)", 
                message.data, fixed_freq_signal, noise);
  }
  
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mt19937 gen_;
  std::uniform_real_distribution<double> dist_;  // 改为均匀分布
  int counter_;  // 添加计数器，用于生成固定频率信号
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LowPassFilterPublisher>());
  rclcpp::shutdown();
  return 0;
}