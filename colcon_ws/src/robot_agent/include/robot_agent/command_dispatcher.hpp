#pragma once

#include "robot_agent/command.hpp"
#include "robot_agent/motion_controller.hpp"

namespace robot_agent {

class CommandDispatcher {
    public:
        explicit CommandDispatcher(MotionController& controller);
        bool execute_command(const RobotCommand& cmd);
        bool execute_plan(const RobotPlan& plan);
    private:
        MotionController& cotroller_;
};

}