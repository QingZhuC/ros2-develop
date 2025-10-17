#include <chrono>
#include <memory>
#include <cmath>
#include <random>  // 用于生成随机噪声

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class SignalGenerate : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr timer_;
  // 原始信号发布者
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sine_publisher_;
  // 带噪声信号发布者
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr noisy_sine_publisher_;
  
  size_t count_;
  // 随机数生成器（用于噪声）
  std::default_random_engine generator_;
  std::normal_distribution<double> noise_distribution_;  // 高斯分布噪声

  void timer_callback()
  {
    double time = count_ * 0.002;
    
    // 生成原始信号
    auto sine_msg = std_msgs::msg::Float64();
    sine_msg.data = sin(2 * M_PI * 10 * time);  // 10Hz正弦波
    
    // 生成噪声（均值0，标准差0.1，噪声幅值较低）
    double noise = noise_distribution_(generator_);
    
    // 生成带噪声的信号
    auto noisy_sine_msg = std_msgs::msg::Float64();
    noisy_sine_msg.data = sine_msg.data + noise;  // 正弦波+噪声
    
    
    // 发布所有信号
    sine_publisher_->publish(sine_msg);
    noisy_sine_publisher_->publish(noisy_sine_msg);

    count_++;
  }

public:
  SignalGenerate()
  : Node("signal_generate"), 
    count_(0),
    // 初始化噪声分布：均值0.0，标准差0.1
    noise_distribution_(0.0, 0.1)
  {
    // 创建发布者，话题名区分原始信号和带噪声信号
    sine_publisher_ = this->create_publisher<std_msgs::msg::Float64>("Sine", 10);
    noisy_sine_publisher_ = this->create_publisher<std_msgs::msg::Float64>("NoisySine", 10);
    
    // 500Hz发布频率
    timer_ = this->create_wall_timer(
      2ms, 
      std::bind(&SignalGenerate::timer_callback, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Signal generator with noise started");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SignalGenerate>());
  rclcpp::shutdown();
  return 0;
}
