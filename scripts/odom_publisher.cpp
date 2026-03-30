#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher()
    : Node("odom_publisher"),
      x_(0.0),
      y_(0.0),
      theta_(0.0),
      imu_wz_(0.0),
      imu_received_(false)
    {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        encoder_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "wheel_encoders",
            10,
            std::bind(&OdomPublisher::encoder_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data_raw",
            10,
            std::bind(&OdomPublisher::imu_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Odom publisher with IMU fusion started.");
    }

private:

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double x_;
    double y_;
    
    double theta_;

    double imu_wz_;
    bool imu_received_;
    static constexpr double TPR = 3000.0;   // ticks/rev
    static constexpr double RHO = 0.0625;   // wheel radius [m]
    static constexpr double ELL = 0.2775;   // wheel separation [m]
    static constexpr double DT  = 0.1;      // 100 ms sample period [s]

    // Fusion weight:
    // 0.0 = encoder heading only
    // 1.0 = IMU heading only
    static constexpr double IMU_BLEND = 0.85;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_wz_ = msg->angular_velocity.z;
        imu_received_ = true;
    }

    void encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "wheel_encoders message too short");
            return;
        }

        int ticks_L = msg->data[0];
        int ticks_R = msg->data[1];

        auto now = this->get_clock()->now();

        // Tick deltas -> wheel angular displacement [rad]
        double dphi_L = 2.0 * M_PI * static_cast<double>(ticks_L) / TPR;
        double dphi_R = 2.0 * M_PI * static_cast<double>(ticks_R) / TPR;

        // Wheel angular speeds [rad/s]
        double omega_L = dphi_L / DT;
        double omega_R = dphi_R / DT;

        // Wheel linear displacement [m]
        double ds_L = RHO * dphi_L;
        double ds_R = RHO * dphi_R;

        // Wheel linear speeds [m/s]
        double v_L = ds_L / DT;
        double v_R = ds_R / DT;

        // Forward body displacement from encoders
        double ds = 0.5 * (ds_L + ds_R);

        // Yaw rate from encoders
        double wz_enc = (ds_R - ds_L) / (ELL * DT);

        // Fused yaw rate
        double wz_fused = wz_enc;
        if (imu_received_) {
            wz_fused = (1.0 - IMU_BLEND) * wz_enc + IMU_BLEND * imu_wz_;
        }

        // Heading change from fused yaw rate
        double dtheta = wz_fused * DT;

        // Forward speed
        double v = ds / DT;

        // Integrate pose
        x_ += ds * std::cos(theta_ + dtheta / 2.0);
        y_ += ds * std::sin(theta_ + dtheta / 2.0);
        theta_ += dtheta;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = v;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;

        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = wz_fused;

        odom_pub_->publish(odom_msg);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";

        tf_msg.transform.translation.x = x_;
        tf_msg.transform.translation.y = y_;
        tf_msg.transform.translation.z = 0.0;

        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_msg);

        RCLCPP_INFO(
            this->get_logger(),
            "ticks L/R: %d, %d | omega L/R: %.3f, %.3f rad/s | v L/R: %.3f, %.3f m/s | wz_enc: %.3f | wz_imu: %.3f | wz: %.3f | x: %.3f y: %.3f th: %.3f",
            ticks_L, ticks_R, omega_L, omega_R, v_L, v_R,
            wz_enc, imu_wz_, wz_fused, x_, y_, theta_);
    }


};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}