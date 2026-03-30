#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher()
    : Node("odom_publisher"),
      x_(0.0),
      y_(0.0),
      theta_(0.0),
      delta_left_ticks_(0),
      delta_right_ticks_(0),
      imu_wz_(0.0),
      got_ticks_(false),
      first_time_(true)
    {
        this->declare_parameter("wheel_radius", 0.0625);
        this->declare_parameter("track_width", 0.2775);
        this->declare_parameter("ticks_per_revolution", 3000.0);

        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        track_width_ = this->get_parameter("track_width").as_double();
        ticks_per_revolution_ = this->get_parameter("ticks_per_revolution").as_double();

        meters_per_tick_ = (2.0 * M_PI * wheel_radius_) / ticks_per_revolution_;

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        wheel_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/wheel_ticks_delta",
            10,
            std::bind(&OdomPublisher::wheelCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw",
            10,
            std::bind(&OdomPublisher::imuCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            50ms, std::bind(&OdomPublisher::publishOdom, this));

        RCLCPP_INFO(this->get_logger(), "Odom publisher started.");
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr wheel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_;
    double y_;
    double theta_;

    int delta_left_ticks_;
    int delta_right_ticks_;
    double imu_wz_;  // kept for later if you want fusion, not used now

    bool got_ticks_;
    bool first_time_;
    rclcpp::Time last_time_;

    double wheel_radius_;
    double track_width_;
    double ticks_per_revolution_;
    double meters_per_tick_;

    void wheelCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "wheel_ticks_delta needs 2 values");
            return;
        }

        delta_left_ticks_ = msg->data[0];
        delta_right_ticks_ = msg->data[1];
        got_ticks_ = true;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_wz_ = msg->angular_velocity.z;
    }

    void publishOdom()
    {
        if (!got_ticks_) {
            return;
        }

        auto now = this->get_clock()->now();

        if (first_time_) {
            last_time_ = now;
            first_time_ = false;
            got_ticks_ = false;
            return;
        }

        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt <= 0.0) {
            got_ticks_ = false;
            return;
        }

        // Wheel-based odometry only
        double d_left = delta_left_ticks_ * meters_per_tick_;
        double d_right = delta_right_ticks_ * meters_per_tick_;
        double d_center = 0.5 * (d_left + d_right);
        double d_theta = (d_right - d_left) / track_width_;

        theta_ += d_theta;
        theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

        x_ += d_center * std::cos(theta_);
        y_ += d_center * std::sin(theta_);

        double v = d_center / dt;
        double wz = d_theta / dt;

        double qz = std::sin(theta_ / 2.0);
        double qw = std::cos(theta_ / 2.0);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = qz;
        odom_msg.pose.pose.orientation.w = qw;

        odom_msg.twist.twist.linear.x = v;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;

        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = wz;

        odom_pub_->publish(odom_msg);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";

        tf_msg.transform.translation.x = x_;
        tf_msg.transform.translation.y = y_;
        tf_msg.transform.translation.z = 0.0;

        tf_msg.transform.rotation.x = 0.0;
        tf_msg.transform.rotation.y = 0.0;
        tf_msg.transform.rotation.z = qz;
        tf_msg.transform.rotation.w = qw;

        tf_broadcaster_->sendTransform(tf_msg);

        got_ticks_ = false;
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}