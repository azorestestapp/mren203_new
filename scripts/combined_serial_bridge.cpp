#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <thread>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <cmath>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class CombinedSerialBridge : public rclcpp::Node
{
public:
    CombinedSerialBridge() : Node("combined_serial_bridge")
    {
        this->declare_parameter("port", "/dev/ttyACM0");
        this->declare_parameter("baudrate", 115200);
        this->declare_parameter("linear_gain", 800.0);
        this->declare_parameter("angular_gain", 300.0);
        this->declare_parameter("min_pwm", 255);
        this->declare_parameter("turn_deadband_linear", 0.01);
        this->declare_parameter("turn_deadband_angular", 0.01);

        port_ = this->get_parameter("port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();
        linear_gain_ = this->get_parameter("linear_gain").as_double();
        angular_gain_ = this->get_parameter("angular_gain").as_double();
        min_pwm_ = this->get_parameter("min_pwm").as_int();
        turn_deadband_linear_ = this->get_parameter("turn_deadband_linear").as_double();
        turn_deadband_angular_ = this->get_parameter("turn_deadband_angular").as_double();

        wheel_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/wheel_ticks_delta", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&CombinedSerialBridge::cmdVelCallback, this, std::placeholders::_1));

        openSerial();

        running_ = true;
        read_thread_ = std::thread(&CombinedSerialBridge::readLoop, this);

        RCLCPP_INFO(this->get_logger(), "Combined serial bridge started.");
    }

    ~CombinedSerialBridge() override
    {
        running_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    std::string port_;
    int baudrate_;
    double linear_gain_;
    double angular_gain_;
    int min_pwm_;
    double turn_deadband_linear_;
    double turn_deadband_angular_;

    int serial_fd_{-1};
    std::thread read_thread_;
    std::atomic<bool> running_{false};
    std::string read_buffer_;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr wheel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    speed_t baudToTermios(int baudrate)
    {
        switch (baudrate) {
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            default: return B115200;
        }
    }

    void openSerial()
    {
        serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ < 0) {
            throw std::runtime_error("Failed to open serial port " + port_ + ": " + std::strerror(errno));
        }

        struct termios tty{};
        if (tcgetattr(serial_fd_, &tty) != 0) {
            throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
        }

        cfsetispeed(&tty, baudToTermios(baudrate_));
        cfsetospeed(&tty, baudToTermios(baudrate_));

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
        }

        tcflush(serial_fd_, TCIOFLUSH);

        RCLCPP_INFO(this->get_logger(), "Opened serial port: %s", port_.c_str());
    }

    int clampPwm(int value) const
    {
        return std::max(-255, std::min(255, value));
    }

    void applyDriveFloorPreserveRatio(int &left_pwm, int &right_pwm) const
    {
        const int max_mag = std::max(std::abs(left_pwm), std::abs(right_pwm));
        if (max_mag == 0) {
            return;
        }

        if (max_mag < min_pwm_) {
            const double scale = static_cast<double>(min_pwm_) / static_cast<double>(max_mag);
            left_pwm = static_cast<int>(std::round(static_cast<double>(left_pwm) * scale));
            right_pwm = static_cast<int>(std::round(static_cast<double>(right_pwm) * scale));
        }

        left_pwm = clampPwm(left_pwm);
        right_pwm = clampPwm(right_pwm);
    }

    void applyTurnInPlaceFloor(int &left_pwm, int &right_pwm, double w) const
    {
        if (w > 0.0) {
            left_pwm = -min_pwm_;
            right_pwm = min_pwm_;
        } else {
            left_pwm = min_pwm_;
            right_pwm = -min_pwm_;
        }

        left_pwm = clampPwm(left_pwm);
        right_pwm = clampPwm(right_pwm);
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const double v = msg->linear.x;
        const double w = msg->angular.z;

        RCLCPP_INFO(this->get_logger(), "Received /cmd_vel: v=%.3f w=%.3f", v, w);

        int left_pwm = static_cast<int>(std::round(linear_gain_ * v - angular_gain_ * w));
        int right_pwm = static_cast<int>(std::round(linear_gain_ * v + angular_gain_ * w));

        const bool is_turn_in_place =
            (std::abs(v) < turn_deadband_linear_) &&
            (std::abs(w) >= turn_deadband_angular_);

        if (is_turn_in_place) {
            applyTurnInPlaceFloor(left_pwm, right_pwm, w);
        } else {
            applyDriveFloorPreserveRatio(left_pwm, right_pwm);
        }

        left_pwm = clampPwm(left_pwm);
        right_pwm = clampPwm(right_pwm);

        std::ostringstream ss;
        ss << left_pwm << "," << right_pwm << "\n";
        const std::string out = ss.str();

        const ssize_t written = write(serial_fd_, out.c_str(), out.size());
        if (written < 0) {
            RCLCPP_ERROR(this->get_logger(), "Serial write failed: %s", std::strerror(errno));
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Sent PWM: left=%d, right=%d from v=%.3f w=%.3f",
            left_pwm, right_pwm, v, w);
    }

    std::vector<std::string> split(const std::string &s, char delim) const
    {
        std::vector<std::string> elems;
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, delim)) {
            elems.push_back(item);
        }
        return elems;
    }

    void parseLine(const std::string &line)
    {
        auto tokens = split(line, ',');

        if (tokens.size() != 5) {
            return;
        }

        try {
            const int left_ticks = std::stoi(tokens[0]);
            const int right_ticks = std::stoi(tokens[1]);
            const double gx = std::stod(tokens[2]);
            const double gy = std::stod(tokens[3]);
            const double gz = std::stod(tokens[4]);

            std_msgs::msg::Int32MultiArray ticks_msg;
            ticks_msg.data = {left_ticks, right_ticks};
            wheel_pub_->publish(ticks_msg);

            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = this->get_clock()->now();
            imu_msg.header.frame_id = "base_link";
            imu_msg.angular_velocity.x = gx;
            imu_msg.angular_velocity.y = gy;
            imu_msg.angular_velocity.z = gz;
            imu_pub_->publish(imu_msg);
        } catch (...) {
        }
    }

    void readLoop()
    {
        char buf[256];

        while (running_) {
            const ssize_t n = read(serial_fd_, buf, sizeof(buf));

            if (n > 0) {
                for (ssize_t i = 0; i < n; ++i) {
                    const char c = buf[i];
                    if (c == '\n') {
                        if (!read_buffer_.empty()) {
                            parseLine(read_buffer_);
                            read_buffer_.clear();
                        }
                    } else if (c != '\r') {
                        read_buffer_ += c;
                    }
                }
            } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Serial read failed: %s", std::strerror(errno));
            }

            std::this_thread::sleep_for(5ms);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CombinedSerialBridge>());
    rclcpp::shutdown();
    return 0;
}