#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"               // 适配Float64消息类型
#include "std_msgs/msg/float64_multi_array.hpp"
#include <vector>
#include <algorithm>
#include <numeric>

class FilterNode : public rclcpp::Node
{
public:
    FilterNode() : Node("filter_node")
    {
        // 订阅带噪声的正弦信号（适配Float64类型和NoisySine话题）
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "simulated_motor_velocity", 10,  // 订阅NoisySine话题，队列大小适应500Hz输入
            std::bind(&FilterNode::signal_callback, this, std::placeholders::_1));
        
        // 发布滤波结果
        median_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "median_filtered", 10);
        lowpass_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "lowpass_filtered", 10);
        
        // 滤波器参数
        median_window_size_ = 5;    // 中值滤波窗口大小（奇数）
        lowpass_window_size_ = 5;   // 低通滤波窗口大小
        max_buffer_size_ = 1000;    // 缓冲区大小（适应500Hz输入）
        
        // 记录节点启动时间（用于计算相对时间戳）
        
        RCLCPP_INFO(this->get_logger(), "filter started");
    }

private:
    void signal_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        float noisy_value = msg->data;  // 直接获取带噪声的信号值

        noisy_buffer_.push_back(noisy_value);
        
        
        // 应用滤波器（确保缓冲区有足够数据）
        if (noisy_buffer_.size() >= median_window_size_) {
            float median_result = median_filter(noisy_buffer_);
            publish_filtered_result(median_pub_, median_result);
        }
        
        if (noisy_buffer_.size() >= lowpass_window_size_) {
            float lowpass_result = lowpass_filter(noisy_buffer_);
            publish_filtered_result(lowpass_pub_, lowpass_result);
        }
    }
    
    // 中值滤波器
    float median_filter(const std::vector<float>& buffer)
    {
        std::vector<float> window(
            buffer.end() - median_window_size_, 
            buffer.end()
        );
        std::sort(window.begin(), window.end());
        return window[median_window_size_ / 2];
    }
    
    // 低通滤波器
    float lowpass_filter(const std::vector<float>& buffer)
    {
        std::vector<float> window(
            buffer.end() - lowpass_window_size_, 
            buffer.end()
        );
        float sum = std::accumulate(window.begin(), window.end(), 0.0f);
        return sum / lowpass_window_size_;
    }
    
    void publish_filtered_result(
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& pub,
        float value)
    {
        auto msg = std_msgs::msg::Float64();
        msg.data = value; 
        pub->publish(msg);
    }

    // 订阅者和发布者
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr median_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lowpass_pub_;
    
    std::vector<float> noisy_buffer_;       // 存储带噪声的信号
    
    // 滤波器参数
    size_t median_window_size_;
    size_t lowpass_window_size_;
    size_t max_buffer_size_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilterNode>());
    rclcpp::shutdown();
    return 0;
}
    