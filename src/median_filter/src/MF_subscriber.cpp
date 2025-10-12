// median filter based on cpp
// ROS2 Subscriber
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vector>
#include <algorithm>
#include <random>
#include <chrono>
#include <cmath>

class MedianFilterSubscriber : public rclcpp::Node
{
public:
  MedianFilterSubscriber()
  : Node("median_filter_subscriber"),
    window_size_(5), // 中值滤波器窗口大小，可以根据需要调整
    data_buffer_()
  {
    // 创建订阅者，订阅"raw_data"话题
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "raw_data", 10, std::bind(&MedianFilterSubscriber::topic_callback, this, std::placeholders::_1));
    
    // 创建发布者，发布滤波后的数据到"filtered_data"话题
    filtered_publisher_ = this->create_publisher<std_msgs::msg::Float64>("filtered_data", 10);
    
    RCLCPP_INFO(this->get_logger(), "Median Filter Subscriber Node Started");
    RCLCPP_INFO(this->get_logger(), "Window size: %d", window_size_);
  }

private:
  void topic_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%f'", msg->data);
    
    // 添加新数据到缓冲区
    data_buffer_.push_back(msg->data);
    
    // 如果缓冲区大小超过窗口大小，删除最早的数据
    if (data_buffer_.size() > window_size_) {
      data_buffer_.erase(data_buffer_.begin());
    }
    
    // 当缓冲区满时，应用中值滤波
    if (data_buffer_.size() == window_size_) {
      // 创建临时副本用于排序
      std::vector<double> sorted_buffer = data_buffer_;
      
      // 对数据进行排序
      std::sort(sorted_buffer.begin(), sorted_buffer.end());
      
      // 计算中值
      double median;
      if (window_size_ % 2 == 0) {
        // 偶数窗口大小，取中间两个数的平均值
        median = (sorted_buffer[window_size_ / 2 - 1] + sorted_buffer[window_size_ / 2]) / 2.0;
      } else {
        // 奇数窗口大小，取中间的数
        median = sorted_buffer[window_size_ / 2];
      }
      
      // 创建并发布滤波后的数据
      auto filtered_msg = std_msgs::msg::Float64();
      filtered_msg.data = median;
      filtered_publisher_->publish(filtered_msg);
      
      RCLCPP_INFO(this->get_logger(), "Filtered value: '%f'", filtered_msg.data);
    }
  }
  
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filtered_publisher_;
  size_t window_size_;
  std::vector<double> data_buffer_;
};

int main(int argc, char * argv[])
{
  // 初始化ROS2
  rclcpp::init(argc, argv);
  
  // 创建并运行节点
  rclcpp::spin(std::make_shared<MedianFilterSubscriber>());
  
  // 关闭ROS2
  rclcpp::shutdown();
  return 0;
}