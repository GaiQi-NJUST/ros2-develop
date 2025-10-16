#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

namespace motor_simulator {

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double sampling_time)
        : kp_(kp), ki_(ki), kd_(kd), sampling_time_(sampling_time),
          integral_(0.0), previous_error_(0.0), output_limit_(5.0) {}

    void setGains(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void setOutputLimit(double limit) {
        output_limit_ = limit;
    }

    double update(double setpoint, double measurement) {
        double error = setpoint - measurement;
        
        double proportional = kp_ * error;
        
        integral_ += ki_ * error * sampling_time_;
        
        if (integral_ > output_limit_) integral_ = output_limit_;
        if (integral_ < -output_limit_) integral_ = -output_limit_;
        
        double derivative = kd_ * (error - previous_error_) / sampling_time_;
        
        double output = proportional + integral_ + derivative;
        
        if (output > output_limit_) output = output_limit_;
        if (output < -output_limit_) output = -output_limit_;
        
        previous_error_ = error;
        return output;
    }

    void reset() {
        integral_ = 0.0;
        previous_error_ = 0.0;
    }

private:
    double kp_, ki_, kd_;
    double sampling_time_;
    double integral_;
    double previous_error_;
    double output_limit_;
};

} // namespace motor_simulator

#endif // PID_CONTROLLER_HPP