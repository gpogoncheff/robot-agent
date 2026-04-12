#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "robot_agent/command_parser.hpp"
#include "robot_agent/plan_validator.hpp"
#include "robot_agent/command_dispatcher.hpp"
#include "robot_agent/motion_controller.hpp"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        std::cerr << "Usage: ros2 run robot_agent file_plan_demo <plan.json>\n";
        rclcpp::shutdown();
        return 1;
    }

    const std::string plan_path = argv[1];

    auto node = std::make_shared<robot_agent::MotionController>();

    if (!node->wait_for_odom(5.0)) {
        RCLCPP_ERROR(node->get_logger(), "Timed out waiting for odometry");
        rclcpp::shutdown();
        return 1;
    }

    robot_agent::CommandDispatcher dispatcher(*node);

    try {
        const robot_agent::RobotPlan plan =
            robot_agent::CommandParser::parse_plan_file(plan_path);

        robot_agent::PlanValidator::validate_plan(plan);

        RCLCPP_INFO(node->get_logger(), "Plan validated");

        const bool ok = dispatcher.execute_plan(plan);

        if (!ok) {
            RCLCPP_ERROR(node->get_logger(), "Plan execution failed");
            rclcpp::shutdown();
            return 1;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load or execute plan: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}