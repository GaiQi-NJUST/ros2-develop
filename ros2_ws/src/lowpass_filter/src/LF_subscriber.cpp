//lowpass filter subscriber
//接收原始数据并应用低通滤波器进行处理

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <iostream>

class LowPassFilterSubscriber : public rclcpp::Node
{
public:
  LowPassFilterSubscriber() : Node("lowpass_filter_subscriber")
  {
    // 创建订阅者，订阅 "raw_data" 话题
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "raw_data",
      10,
      std::bind(&LowPassFilterSubscriber::topic_callback, this, std::placeholders::_1)
    );
    
    // 创建发布者，发布滤波后的数据到 "filtered_data" 话题
    filtered_publisher_ = this->create_publisher<std_msgs::msg::Float64>("filtered_data", 10);
    
    // 初始化低通滤波器参数
    alpha_ = 0.1; // 滤波系数，范围在0-1之间
    filtered_value_ = 0.0;
    is_first_data_ = true;
    
    RCLCPP_INFO(this->get_logger(), "LowPassFilterSubscriber has been started");
    RCLCPP_INFO(this->get_logger(), "Low-pass filter with alpha = %f", alpha_);
  }

private:
  void topic_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    double raw_data = msg->data;
    
    // 应用低通滤波器
    if (is_first_data_)
    {
      // 对于第一个数据点，直接设置滤波后的值
      filtered_value_ = raw_data;
      is_first_data_ = false;
    }
    else
    {
      // 一阶低通滤波器公式: filtered = alpha * new_value + (1 - alpha) * previous_filtered
      filtered_value_ = alpha_ * raw_data + (1.0 - alpha_) * filtered_value_;
    }
    
    // 发布滤波后的数据
    auto filtered_msg = std_msgs::msg::Float64();
    filtered_msg.data = filtered_value_;
    filtered_publisher_->publish(filtered_msg);
    
    // 打印原始数据和滤波后的数据
    RCLCPP_INFO(this->get_logger(), "Received: %.4f, Filtered: %.4f", raw_data, filtered_value_);
  }
  
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filtered_publisher_;
  double alpha_; // 低通滤波器系数
  double filtered_value_; // 上一次滤波后的值
  bool is_first_data_; // 标记是否是第一个数据点
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LowPassFilterSubscriber>());
  rclcpp::shutdown();
  return 0;
}