#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class SignalProcessor : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sine_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr square_subscriber_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr processed_publisher_;
    
    double latest_sine_ = 0.0;
    double latest_square_ = 0.0;
    void sine_callback(const std_msgs::msg::Float64::SharedPtr sine_msg)
    {
        latest_sine_ = sine_msg->data;
        process_signals();
    }

    void square_callback(const std_msgs::msg::Float64::SharedPtr square_msg)
    {
        latest_square_ = square_msg->data;
        process_signals();
    }

    void process_signals()
    {
        bool is_same_sign = 
            ((latest_sine_ * latest_square_ )> 0);

        auto processed_msg = std_msgs::msg::Float64();
        processed_msg.data = is_same_sign ? latest_sine_ : 0.0;

        processed_publisher_->publish(processed_msg);

    }
public:
    SignalProcessor() 
    : Node("signal_processor")
    {
        sine_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "Sine",  
            10,         
            std::bind(&SignalProcessor::sine_callback, this, std::placeholders::_1)
        );

        square_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "Square",  
            10,            
            std::bind(&SignalProcessor::square_callback, this, std::placeholders::_1)
        );

        processed_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "processed_sine",  
            10
        );

        RCLCPP_INFO(this->get_logger(), "Signal processor node started");
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalProcessor>()); 
    rclcpp::shutdown();
    return 0;
}

