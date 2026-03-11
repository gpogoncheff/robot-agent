#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_agent/motion_controller.hpp"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_agent::MotionController>();
    
    // wait briefly for odom data to come in
    if (!node->wait_for_odom(5.0)) {
        RCLCPP_ERROR(node->get_logger(), "Timed out waiting for odometry");
        return 1;
    }

    node->move_forward(1.0);
    node->rotate(3.14);
    node->move_forward(1.0);
    node->rotate(-3.14);
    node->stop();
    
    rclcpp::shutdown();
    
    return 0;
}