#include "../include/motor_simulator/pid_control.hpp"

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"

#define PI 3.14159265358979323846

PID velocity_pid;
PID angle_pid;

class PIDControl : public rclcpp::Node {
public:
    PIDControl()
        : Node("pidcontrol") {

        Velocity_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "lowpass_filtered", 10,
            std::bind(&PIDControl::Velocity_subscriber_callback, this, std::placeholders::_1));
        
        Angle_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "simulated_motor_angle", 10,
            std::bind(&PIDControl::Angle_subscriber_callback, this, std::placeholders::_1));

        output_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>("motor_simulator_control_input", 10);

        RCLCPP_INFO(get_logger(), "PID_Control launch");
    }

private:
    void data_publish() {

        std_msgs::msg::Float64 output_msg;
        output_msg.data = velocity_pid.get_out();
        output_publisher_->publish(output_msg);
    }

    void Velocity_subscriber_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        current_velocity = msg->data;

        //Velocity_PID_Control(500.0);//控制角度时注释掉这行//设置目标值
    }

    void Angle_subscriber_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        current_angle = msg->data;

        Angle_PID_Control(PI*6/5);//控制速度时注释掉这行//设置目标值
    }

    void Velocity_PID_Control(double target_change)
    {
        velocity_pid.change_target(target_change);
        velocity_pid.PID_calc(current_velocity);
        data_publish();
    }

    void Angle_PID_Control(double target_change)
    {
        if(target_change > PI) {target_change = target_change - 2*PI;}//走优弧
        angle_pid.change_target(target_change);
        angle_pid.PID_calc(current_angle);
        Velocity_PID_Control(angle_pid.get_out());
        data_publish();
    }

    double current_velocity;
    double current_angle;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr output_publisher_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr Velocity_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr Angle_subscriber_;
};

int main(int argc, char* argv[]) {
    velocity_pid.motor_PID_Init(0, 0, 2, 0.0, 0);//调节pid参数
    angle_pid.motor_PID_Init(0, 0, 4, 0, 0);//调节pid参数

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDControl>());
    rclcpp::shutdown();
    return 0;
}