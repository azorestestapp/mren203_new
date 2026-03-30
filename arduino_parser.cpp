#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <boost/asio.hpp>

using namespace std::chrono_literals;

class ArduinoParser : public rclcpp::Node {
public:
    ArduinoParser() : Node("arduino_parser"), io_(), serial_(io_) {
        this->declare_parameter("port", "/dev/ttyACM0");
        this->declare_parameter("baudrate", 115200);

        std::string port = this->get_parameter("port").as_string();
        uint32_t baud = this->get_parameter("baudrate").as_int();

        try {
            serial_.open(port);
            serial_.set_option(boost::asio::serial_port_base::baud_rate(baud));
            RCLCPP_INFO(this->get_logger(), "Opened serial port: %s", port.c_str());
        } catch (std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Could not open serial port: %s", e.what());
        }

        wheel_ticks_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/wheel_ticks_delta", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);

        timer_ = this->create_wall_timer(20ms, std::bind(&ArduinoParser::read_serial, this));
    }

private:
    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr wheel_ticks_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    void read_serial() {
        if (!serial_.is_open()) return;

        try {
            boost::asio::streambuf buf;
            boost::asio::read_until(serial_, buf, '\n');
            std::istream is(&buf);
            std::string line;
            std::getline(is, line);
            parse_and_publish(line);
        } catch (std::exception& e) {
            // Ignore transient serial parse/read errors
        }
    }

    void parse_and_publish(const std::string& line) {
        std::vector<std::string> tokens;
        std::stringstream ss(line);
        std::string item;

        while (std::getline(ss, item, ',')) {
            tokens.push_back(item);
        }

        if (tokens.size() != 5) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Malformed line (expected 5 values, got %zu): %s",
                tokens.size(), line.c_str());
            return;
        }

        try {
            int delta_left_ticks = std::stoi(tokens[0]);
            int delta_right_ticks = std::stoi(tokens[1]);
            double gx = std::stod(tokens[2]);
            double gy = std::stod(tokens[3]);
            double gz = std::stod(tokens[4]);

            std_msgs::msg::Int32MultiArray wheel_msg;
            wheel_msg.data = {delta_left_ticks, delta_right_ticks};
            wheel_ticks_pub_->publish(wheel_msg);

            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = this->now();
            imu_msg.header.frame_id = "imu_link";

            imu_msg.angular_velocity.x = gx;
            imu_msg.angular_velocity.y = gy;
            imu_msg.angular_velocity.z = gz;

            // Mark orientation and linear acceleration as unavailable
            imu_msg.orientation_covariance[0] = -1.0;
            imu_msg.linear_acceleration_covariance[0] = -1.0;

            imu_pub_->publish(imu_msg);

        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Failed to parse line: %s", line.c_str());
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoParser>());
    rclcpp::shutdown();
    return 0;
}