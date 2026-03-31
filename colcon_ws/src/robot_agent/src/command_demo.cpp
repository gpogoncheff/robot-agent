#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_agent/command.hpp"
#include "robot_agent/command_dispatcher.hpp"
#include "robot_agent/motion_controller.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<robot_agent::MotionController>();

    if (!node->wait_for_odom(5.0)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to receive odometry data within timeout");
        return 1;
    }

    robot_agent::CommandDispatcher dispatcher(*node);

    robot_agent::RobotPlan plan = {
        {robot_agent::CommandType::MoveForward, 1.0},
        {robot_agent::CommandType::Rotate, M_PI / 2},
        {robot_agent::CommandType::MoveForward, 1.0},
        {robot_agent::CommandType::Stop, 0.0}
    };

    dispatcher.execute_plan(plan);

    rclcpp::shutdown();
    return 0;

}