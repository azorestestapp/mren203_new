#include <chrono>
#include <memory>
#include <string>
#include <cstdio>

#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class ArduinoParser : public rclcpp::Node {
public:
    ArduinoParser() : Node("arduino_parser"), io_(), serial_(io_) {
        std::string port = this->declare_parameter("port", "/dev/ttyACM0");
        int baud = this->declare_parameter("baudrate", 115200);

        serial_.open(port);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baud));

        encoder_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("wheel_encoders", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&ArduinoParser::read_serial, this));

        RCLCPP_INFO(this->get_logger(), "Opened serial port: %s", port.c_str());
    }

private:
    void read_serial() {
        boost::asio::streambuf buf;
        boost::asio::read_until(serial_, buf, '\n');

        std::istream is(&buf);
        std::string line;
        std::getline(is, line);

        int ticks_L, ticks_R;
        double omega_x, omega_y, omega_z;

        if (std::sscanf(line.c_str(), "%d,%d,%lf,%lf,%lf",
                        &ticks_L, &ticks_R, &omega_x, &omega_y, &omega_z) != 5) {
            RCLCPP_WARN(this->get_logger(), "Bad line: %s", line.c_str());
            return;
        }

        // Publish encoder deltas
        std_msgs::msg::Int32MultiArray enc_msg;
        enc_msg.data = {ticks_L, ticks_R};
        encoder_pub_->publish(enc_msg);

        // Publish IMU (already in rad/s)
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";

        imu_msg.orientation_covariance[0] = -1.0;  // no orientation

        imu_msg.angular_velocity.x = omega_x;
        imu_msg.angular_velocity.y = omega_y;
        imu_msg.angular_velocity.z = omega_z;

        imu_pub_->publish(imu_msg);
    }

    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoParser>());
    rclcpp::shutdown();
    return 0;
}