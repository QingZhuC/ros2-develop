#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
class SignalGenerate : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sine_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr square_publisher_;
  size_t count_;
  void timer_callback()
  {
    double time = count_ * 0.002;
    auto sine_msg = std_msgs::msg::Float64();
    sine_msg.data = sin(2 * M_PI * 10 * time);  // sin(2πft)
      
    auto square_msg = std_msgs::msg::Float64();
    square_msg.data = (fmod(time, 1.0) < 0.5) ? 1.0 : -1.0;
    
    sine_publisher_->publish(sine_msg);
    square_publisher_->publish(square_msg);

    count_++;
  }
public:
  SignalGenerate()
  : Node("signal_generate"),count_(0)
  {
    sine_publisher_ = this->create_publisher<std_msgs::msg::Float64>("Sine", 10);
    square_publisher_ = this->create_publisher<std_msgs::msg::Float64>("Square", 10);
    timer_ = this->create_wall_timer(2ms, std::bind(&SignalGenerate::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Signal generator started");
  }
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SignalGenerate>());
  rclcpp::shutdown();
  return 0;
}