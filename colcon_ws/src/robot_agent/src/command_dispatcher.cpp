#include "robot_agent/command_dispatcher.hpp"


namespace robot_agent {
    CommandDispatcher::CommandDispatcher(MotionController& controller) 
        : controller_(controller) {}

    bool CommandDispatcher::execute_command(const RobotCommand& cmd) {
        RCLCPP_INFO(
            controller_.get_logger(), 
            "Executing command: %s (value=%.3f)", 
            to_string(cmd.type).c_str(), cmd.value
        );

        switch (cmd.type) {
            case CommandType::MoveForward:
                controller_.move_forward(cmd.value);
                return true;
            case CommandType::Rotate:
                controller_.rotate(cmd.value);
                return true;
            case CommandType::Stop:
                controller_.stop();
                return true;
            default:
                RCLCPP_ERROR(controller_.get_logger(), "Unkown command type received by dispatcher");
                return false;
        }
    }

    bool CommandDispatcher::execute_plan(const RobotPlan& plan) {
        for (const auto& cmd : plan) {
            if (!execute_command(cmd)) return false;
        }
        return true;
    }

}