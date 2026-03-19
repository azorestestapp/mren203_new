// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>
// #include <vector>
// #include <sstream>
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/int32_multi_array.hpp"
// #include "sensor_msgs/msg/imu.hpp"
// #include "sensor_msgs/msg/joint_state.hpp"
// #include <boost/asio.hpp>

// using namespace std::chrono_literals;

// class ArduinoParser : public rclcpp::Node {
// public:
//     ArduinoParser() : Node("arduino_parser"), io_(), serial_(io_) {
//         this->declare_parameter("port", "/dev/ttyACM0");
//         this->declare_parameter("baudrate", 115200);

//         std::string port = this->get_parameter("port").as_string();
//         uint32_t baud = this->get_parameter("baudrate").as_int();

//         try {
//             serial_.open(port);
//             serial_.set_option(boost::asio::serial_port_base::baud_rate(baud));
//             RCLCPP_INFO(this->get_logger(), "Opened serial port: %s", port.c_str());
//         } catch (std::exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "Could not open serial port: %s", e.what());
//         }

//         // Corrected Publishers
//         encoder_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("wheel_encoders", 10);
//         imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
//         joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("actuator/data", 10);

//         timer_ = this->create_wall_timer(100ms, std::bind(&ArduinoParser::read_serial, this));
//     }

// private:
//     boost::asio::io_service io_;
//     boost::asio::serial_port serial_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     // Definitions must match usage
//     rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_pub_;
//     rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
//     rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    
//     void read_serial() {
//         if (!serial_.is_open()) return;
//         try {
//             boost::asio::streambuf buf;
//             boost::asio::read_until(serial_, buf, '\n');
//             std::istream is(&buf);
//             std::string line;
//             std::getline(is, line);
//             parse_and_publish(line);
//         } catch (std::exception& e) {
//             // Silence common timeout errors if needed
//         }
//     }

// void parse_and_publish(const std::string& line) {
//     std::vector<std::string> tokens;
//     std::stringstream ss(line);
//     std::string item;
    
//     while (std::getline(ss, item, ',')) {
//         tokens.push_back(item);
//     }

//     if (tokens.size() == 6) {
//         // --- PRINT THE DATA TO TERMINAL ---
//         RCLCPP_INFO(this->get_logger(), 
//             "Data Received -> EncL: %s, EncR: %s | Quat: [%s, %s, %s, %s]",
//             tokens[0].c_str(), tokens[1].c_str(), 
//             tokens[2].c_str(), tokens[3].c_str(), tokens[4].c_str(), tokens[5].c_str());

//         // 1. Int32MultiArray
//         auto enc_msg = std_msgs::msg::Int32MultiArray();
//         enc_msg.data = {std::stoi(tokens[0]), std::stoi(tokens[1])};
//         encoder_pub_->publish(enc_msg);

//         // 2. JointState
//         auto joint_msg = sensor_msgs::msg::JointState();
//         joint_msg.header.stamp = this->now();
//         joint_msg.name = {"left_wheel", "right_wheel"};
//         joint_msg.position = {std::stod(tokens[0]), std::stod(tokens[1])};
//         joint_pub_->publish(joint_msg);

//         // 3. IMU
//         auto imu_msg = sensor_msgs::msg::Imu();
//         imu_msg.header.stamp = this->now();
//         imu_msg.header.frame_id = "imu_link";
//         imu_msg.orientation.x = std::stod(tokens[2]);
//         imu_msg.orientation.y = std::stod(tokens[3]);
//         imu_msg.orientation.z = std::stod(tokens[4]);
//         imu_msg.orientation.w = std::stod(tokens[5]);
//         imu_pub_->publish(imu_msg);
//     } else {
//         RCLCPP_WARN(this->get_logger(), "Malformed line (expected 6 values, got %zu): %s", tokens.size(), line.c_str());
//     }
// }



// };

// int main(int argc, char * argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ArduinoParser>());
//     rclcpp::shutdown();
//     return 0;
// }
