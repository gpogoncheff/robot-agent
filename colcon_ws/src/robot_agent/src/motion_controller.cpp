#include "robot_agent/motion_controller.hpp"

#include <cmath>
#include <chrono>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_agent {

namespace {
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
}

MotionController::MotionController() : 
    rclcpp::Node("motion_controller"),
    current_x_(0.0),
    current_y_(0.0),
    current_yaw_(0.0),
    odom_received_(false) {

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&MotionController::odom_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "MotionController started");
}

void MotionController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    const auto& q = msg->pose.pose.orientation;
    const double siny_cosp = 2.0 * ((q.w * q.z) + (q.x * q.y));
    const double cosy_cosp = 1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z)));
    current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    odom_received_ = true;
}

void MotionController::publish_cmd(double linear_x, double angular_z) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "base_link";

    msg.twist.linear.x = linear_x;
    msg.twist.angular.z = angular_z;

    cmd_pub_->publish(msg);
}

void MotionController::stop() {
    publish_cmd(0.0, 0.0);
}

void MotionController::move_forward(double distance_m, double speed_mps) {
    if (!odom_received_) {
        RCLCPP_WARN(this->get_logger(), "No odometry yet; cannot move forward");
        return;
    }

    const double start_x = current_x_, start_y = current_y_;
    
    rclcpp::Rate rate(20.0);

    while (rclcpp::ok()) {
        const double dx = current_x_ - start_x;
        const double dy = current_y_ - start_y;
        const double traveled = std::sqrt((dx*dx) + (dy*dy));
        if (traveled >= distance_m) {
            break;
        }
        publish_cmd(speed_mps, 0.0);
        rclcpp::spin_some(shared_from_this());
        rate.sleep();
    }

    stop();
}

void MotionController::rotate(double angle_rad, double angular_speed_rps = 0.5) {
    if (!odom_received_) {
        RCLCPP_WARN(this->get_logger(), "No odometry yet; cannot rotate");
        return;
    }

    const double start_yaw = current_yaw_;
    const double direction = angle_rad >= 0.0 ? 1.0 : -1.0;

    rclcpp::Rate rate(20.0);

    while (rclcpp::ok()) {
        const double delta = normalize_angle(current_yaw_ - start_yaw);
        if (std::abs(delta) >= std::abs(angle_rad)) {
            break;
        }
        publish_cmd(0.0, direction*angular_speed_rps);
        rclcpp::spin_some(shared_from_this());
        rate.sleep();
    }

    stop();
}
 
}