#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class SignalPublisher : public rclcpp::Node
{
public:
    SignalPublisher() : Node("signal_publisher"), start_time_(this->now())
    {
        //创建正弦、方波的信号发生器
        sin_pub_ = this->create_publisher<std_msgs::msg::Float32>("sin_signal", 10);
        square_pub_ = this->create_publisher<std_msgs::msg::Float32>("square_signal", 10);

        //创建定时器
        sin_timer_ = this->create_wall_timer(2ms, std::bind(&SignalPublisher::publish_sin_signal, this)); // 正弦波采样频率
        square_timer_ = this->create_wall_timer(1ms, std::bind(&SignalPublisher::publish_square_signal, this)); // 方波采样频率
    }

private:
    void publish_sin_signal()
    {
        auto msg = std_msgs::msg::Float32();
        double t = (this->now() - start_time_).seconds();
        msg.data = std::sin(2 * M_PI * 10.0 * t);  //根据时间生成正弦信号
        sin_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing sin: %.2f", msg.data);
    }

    void publish_square_signal()
    {
        auto msg = std_msgs::msg::Float32();
        double t = (this->now() - start_time_).seconds();
        const double square = std::sin(2*M_PI*1.0*t);
        msg.data = (square > 0)?1.0 : -1.0;         //已频率为1hz的正弦信号为基准，大于0为1，小于0为-1，生成方波
        square_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing square: %.1f", msg.data);
    }


    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sin_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr square_pub_;
    rclcpp::TimerBase::SharedPtr sin_timer_;
    rclcpp::TimerBase::SharedPtr square_timer_;
    rclcpp::Time start_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalPublisher>());
    rclcpp::shutdown();
    return 0;
}
