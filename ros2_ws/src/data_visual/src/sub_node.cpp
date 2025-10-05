#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <chrono>

using namespace std::chrono_literals;

class SignalProcessor : public rclcpp::Node
{
public:
    SignalProcessor() : Node("signal_processor")
    {
        //创建两个订阅器用于接收正弦，方波信号
        sin_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "sin_signal", 10, std::bind(&SignalProcessor::sin_callback, this, std::placeholders::_1));

        square_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "square_signal", 10, std::bind(&SignalProcessor::square_callback, this, std::placeholders::_1));

        //创建发布器发布处理后的信号
        proc_pub_ = this->create_publisher<std_msgs::msg::Float32>("processed", 10);

        timer_ = this->create_wall_timer(
            1ms, std::bind(&SignalProcessor::process_signals, this));
    }

private:
    //记录最新收到的数据
    double last_sin_ = 0.0;
    double last_square_ = 0.0;

    void sin_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        last_sin_ = msg->data;
        process_signals();
    }

    void square_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        last_square_ = msg->data;
    }

    void process_signals()
    {
        auto msg = std_msgs::msg::Float32();
        if (last_sin_ * last_square_ > 0)
        {
            msg.data = last_sin_;
            RCLCPP_INFO(this->get_logger(), "Processed: %.2f", msg.data);
        }
        else
        {
            msg.data = 0.0f;
        }
        proc_pub_->publish(msg);
    }

    std::optional<float> position_;
    std::optional<float> square_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sin_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr square_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr proc_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalProcessor>());
    rclcpp::shutdown();
    return 0;
}
