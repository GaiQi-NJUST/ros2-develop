#include <cmath>
#include <vector>

namespace motor_simulator {

class LowPassFilter {
public:
    LowPassFilter(double cutoff_frequency, double sampling_time, int order = 1)
        : sampling_time_(sampling_time), order_(order), initialized_(false) {
        setCutoffFrequency(cutoff_frequency);
        for (int i = 0; i < order_; i++) {
            outputs_.push_back(0.0);
        }
    }

    void setCutoffFrequency(double cutoff_frequency) {
        double omega = 2.0 * M_PI * cutoff_frequency;
        double alpha = omega * sampling_time_;
        coefficient_ = alpha / (1.0 + alpha);
    }

    void setOrder(int order) {
        order_ = order;
        outputs_.clear();
        for (int i = 0; i < order_; i++) {
            outputs_.push_back(0.0);
        }
        initialized_ = false;
    }

    double update(double input) {
        if (!initialized_) {
            for (int i = 0; i < order_; i++) {
                outputs_[i] = input;
            }
            initialized_ = true;
        } else {
            outputs_[0] = coefficient_ * input + (1.0 - coefficient_) * outputs_[0];
            for (int i = 1; i < order_; i++) {
                outputs_[i] = coefficient_ * outputs_[i-1] + (1.0 - coefficient_) * outputs_[i];
            }
        }
        return outputs_.back();
    }

    double getOutput() const { 
        if (outputs_.empty()) return 0.0;
        return outputs_.back(); 
    }
    
    void reset() { 
        initialized_ = false; 
        for (auto& output : outputs_) {
            output = 0.0;
        }
    }

private:
    double sampling_time_;
    double coefficient_;
    int order_;
    std::vector<double> outputs_;
    bool initialized_;
};

} // namespace motor_simulator