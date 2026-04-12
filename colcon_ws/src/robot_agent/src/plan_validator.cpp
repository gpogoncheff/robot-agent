#include "robot_agent/plan_validator.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace robot_agent {

namespace {
    void validate_command(const RobotCommand& cmd) {
        switch (cmd.type) {
            case CommandType::MoveForward:
                if (!std::isfinite(cmd.value)) {
                    throw std::runtime_error("MoveForward distance must be finite");
                }
                if (cmd.value <= 0) {
                    throw std::runtime_error("MoveForward distance must be > 0");
                }
                return;
            case CommandType::Rotate:
                if (!std::isfinite(cmd.value)) {
                    throw std::runtime_error("Rotate angle must be finite");
                }
                if (cmd.value <= 1e-6) {
                    throw std::runtime_error("Rotate angle must be non-zero");
                }
                if (std::abs(cmd.value) > 2.0*M_PI) {
                    throw std::runtime_error("Rotate angle should be within +/- 2pi radians");
                }
                return;
            case CommandType::Stop:
                return;
            case CommandType::GetPose:
                return;
            default:
                throw std::runtime_error("Unknown command type in validator");
        }
    }
}

void PlanValidator::validate_plan(const RobotPlan& plan) {
    if (plan.empty()) {
        throw std::runtime_error("Plan must not be empty");
    }
    
    for (std::size_t i = 0; i < plan.size(); ++i) {
        validate_command(plan[i]);
    }
}

}