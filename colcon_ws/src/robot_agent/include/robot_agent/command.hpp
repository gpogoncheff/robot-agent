#pragma once

#include <string>
#include <vector>

namespace robot_agent {

enum class CommandType {
    MoveForward,
    Rotate,
    Stop
};

struct RobotCommand {
    CommandType type;
    double value;  // distance passed to MoveForward, angle for Rotate, ignored for stop
};

using RobotPlan = std::vector<RobotCommand>;

std::string to_string(CommandType type);

}