#include <memory>
#include <string>
#include <sstream>
#include <algorithm>
#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CmdVelSerialBridge : public rclcpp::Node
{
public:
    CmdVelSerialBridge() : Node("cmd_vel_serial_bridge"), io_(), serial_(io_)
    {
        this->declare_parameter("port", "/dev/ttyACM0");
        this->declare_parameter("baudrate", 115200);

        // Simple mapping gains from cmd_vel to PWM
        this->declare_parameter("linear_gain", 800.0);
        this->declare_parameter("angular_gain", 250.0);

        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();

        linear_gain_ = this->get_parameter("linear_gain").as_double();
        angular_gain_ = this->get_parameter("angular_gain").as_double();

        try {
            serial_.open(port);
            serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
            RCLCPP_INFO(this->get_logger(), "Opened serial port: %s", port.c_str());
        } catch (const std::exception &e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port: %s", e.what());
            throw;
        }

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelSerialBridge::cmdVelCallback, this, std::placeholders::_1));
    }

private:
    int clampPwm(int value)
    {
        return std::max(-255, std::min(255, value));
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double v = msg->linear.x;
        double w = msg->angular.z;

        // Convert cmd_vel into left/right PWM
        int left_pwm = static_cast<int>(linear_gain_ * v - angular_gain_ * w);
        int right_pwm = static_cast<int>(linear_gain_ * v + angular_gain_ * w);

        left_pwm = clampPwm(left_pwm);
        right_pwm = clampPwm(right_pwm);

        std::ostringstream ss;
        ss << left_pwm << "," << right_pwm << "\n";
        std::string out = ss.str();

        try {
            boost::asio::write(serial_, boost::asio::buffer(out));
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Sent PWM: left=%d, right=%d from v=%.3f w=%.3f",
                left_pwm, right_pwm, v, w);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Serial write failed: %s", e.what());
        }
    }

    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    double linear_gain_;
    double angular_gain_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelSerialBridge>());
    rclcpp::shutdown();
    return 0;
}