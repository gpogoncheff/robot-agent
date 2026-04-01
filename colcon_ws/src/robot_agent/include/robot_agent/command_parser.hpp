#pragma once

#include <string>

#include "robot_agent/command.hpp"

namespace robot_agent {

class CommandParser {
public:
    static RobotPlan parse_plan_json(const std::string& json_text);
    static RobotPlan parse_plan_file(const std::string& file_path);
};

}