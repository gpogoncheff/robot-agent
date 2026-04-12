#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "robot_agent/command_dispatcher.hpp"
#include "robot_agent/command_parser.hpp"
#include "robot_agent/plan_validator.hpp"
#include "robot_agent/motion_controller.hpp"
#include "robot_agent/llm_bridge.hpp"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        std::cerr << "Usage: ros2 run robot_agent llm_agent_demo \"your command here\"";
        rclcpp::shutdown();
        return 1;
    }

    const std::string user_command = argv[1];

    auto node = std::make_shared<robot_agent::MotionController>();

    if (!node->wait_for_odom(5.0)) {
        RCLCPP_ERROR(node->get_logger(), "Timed out waiting for odometry");
        rclcpp::shutdown();
        return 1;
    }

    robot_agent::CommandDispatcher dispatcher(*node);

    try {
        RCLCPP_INFO(node->get_logger(), "User command: %s", user_command.c_str());

        const std::string json_text = robot_agent::LLMBridge::query_tool_plan(user_command);

        RCLCPP_INFO(node->get_logger(), "LLM returned JSON:\n%s", json_text.c_str());

        const robot_agent::RobotPlan plan = robot_agent::CommandParser::parse_plan_json(json_text);

        RCLCPP_INFO(node->get_logger(), "Parsed plan with %zu step(s)", plan.size());

        robot_agent::PlanValidator::validate_plan(plan);

        RCLCPP_INFO(node->get_logger(), "Plan validation passed");

        const bool ok = dispatcher.execute_plan(plan);
        if (!ok) {
            RCLCPP_ERROR(node->get_logger(), "Plan execution failed");
            rclcpp::shutdown();
            return 1;
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Agent loop failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;

}