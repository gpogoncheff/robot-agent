#pragma once

#include <memory>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"


namespace robot_agent {

class MotionController : public rclcpp::Node {
    public:
        MotionController();
        void stop();
        void move_forward(double distance_m, double speed_mps = 0.2);
        void rotate(double angle_rad, double angular_speed_rps = 0.5);
    private:
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void publish_cmd(double linear_x, double angular_z);
        double current_x_, current_y_;
        double current_yaw_;
        bool odom_received_;

        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}