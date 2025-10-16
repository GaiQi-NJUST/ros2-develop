#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "motor_simulator/lowpass_filter.hpp"
#include "motor_simulator/pid_controller.hpp"

using namespace std::chrono_literals;

class MotorController : public rclcpp::Node {
public:
    MotorController() : Node("motor_controller"), start_time_(this->now()) {
       
        this->declare_parameter("target_velocity", 250.0);
        this->declare_parameter("kp", 1.0);
        this->declare_parameter("ki", 0.3);
        this->declare_parameter("kd", 0.4);
        this->declare_parameter("filter_cutoff", 0.6);
        this->declare_parameter("filter_order", 2);
        this->declare_parameter("output_limit", 15.0);
        this->declare_parameter("dead_zone", 0.1);
        
        double target_velocity = this->get_parameter("target_velocity").as_double();
        double kp = this->get_parameter("kp").as_double();
        double ki = this->get_parameter("ki").as_double();
        double kd = this->get_parameter("kd").as_double();
        double filter_cutoff = this->get_parameter("filter_cutoff").as_double();
        int filter_order = this->get_parameter("filter_order").as_int();
        double output_limit = this->get_parameter("output_limit").as_double();
        double dead_zone = this->get_parameter("dead_zone").as_double();
        
        filter_ = std::make_unique<motor_simulator::LowPassFilter>(filter_cutoff, 0.01, filter_order);
        pid_.reset(new motor_simulator::PIDController(kp, ki, kd, 0.01));
        pid_->setOutputLimit(output_limit);
        
        dead_zone_ = dead_zone;
        target_velocity_ = target_velocity;
        
        velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "simulated_motor_velocity", 10,
            std::bind(&MotorController::velocityCallback, this, std::placeholders::_1));
            
        control_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "motor_simulator_control_input", 10);
        filtered_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "filtered_velocity", 10);
        raw_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("raw_velocity", 10);
        error_pub_ = this->create_publisher<std_msgs::msg::Float64>("velocity_error", 10);
        control_output_pub_ = this->create_publisher<std_msgs::msg::Float64>("control_output", 10);
        
        data_output_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "motor_simulator_data_output", 10);
        
        RCLCPP_INFO(this->get_logger(), "Enhanced Motor Controller with timed data output:");
        RCLCPP_INFO(this->get_logger(), "Target velocity: %.1f", target_velocity_);
        RCLCPP_INFO(this->get_logger(), "PID gains: Kp=%.3f, Ki=%.3f, Kd=%.3f", kp, ki, kd);
        RCLCPP_INFO(this->get_logger(), "Filter: %.1f Hz, %d order", filter_cutoff, filter_order);
        RCLCPP_INFO(this->get_logger(), "Output limit: %.1f, Dead zone: %.1f", output_limit, dead_zone);
    }

private:
    void velocityCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        double raw_velocity = msg->data;
        double filtered_velocity = filter_->update(raw_velocity);
        double error = target_velocity_ - filtered_velocity;
        
        double control_output = 0.0;
        if (std::abs(error) > dead_zone_) {
            control_output = pid_->update(target_velocity_, filtered_velocity);
        } else {
            control_output = 0.0;
        }
        
        auto control_msg = std_msgs::msg::Float64();
        control_msg.data = control_output;
        control_pub_->publish(control_msg);
        
        auto data_msg = std_msgs::msg::Float64();
        auto elapsed = this->now() - start_time_;
        if (elapsed.seconds() < 1.0) {
            data_msg.data = raw_velocity;
        } else {
            data_msg.data = filtered_velocity;
        }
        data_output_pub_->publish(data_msg);
        
        publishDebugInfo(raw_velocity, filtered_velocity, control_output, error);
    }
    
    void publishDebugInfo(double raw_velocity, double filtered_velocity, double control_output, double error) {
        auto filtered_msg = std_msgs::msg::Float64();
        filtered_msg.data = filtered_velocity;
        filtered_velocity_pub_->publish(filtered_msg);
        
        auto error_msg = std_msgs::msg::Float64();
        error_msg.data = error;
        error_pub_->publish(error_msg);
        
        auto control_out_msg = std_msgs::msg::Float64();
        control_out_msg.data = control_output;
        control_output_pub_->publish(control_out_msg);
        
        static int count = 0;
        if (count++ % 30 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                       "原始: %6.1f | 滤波: %6.1f | 目标: %6.1f | 误差: %6.1f | 控制: %7.3f",
                       raw_velocity, filtered_velocity, target_velocity_, error, control_output);
        }
    }

    rclcpp::Time start_time_;
    std::unique_ptr<motor_simulator::LowPassFilter> filter_;
    std::unique_ptr<motor_simulator::PIDController> pid_;
    double target_velocity_;
    double dead_zone_;
    
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filtered_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr raw_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_output_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr data_output_pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}