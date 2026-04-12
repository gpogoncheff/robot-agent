#pragma once

#include <string>

#include "robot_agent/command.hpp"

namespace robot_agent {

class PlanValidator {
    public:
        static void validate_plan(const RobotPlan& plan);
};

}