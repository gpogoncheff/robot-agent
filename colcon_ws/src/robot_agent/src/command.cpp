#include "robot_agent/command.hpp"

namespace robot_agent {

std::string to_string(CommandType type) {
    switch (type) {
        case CommandType::MoveForward:
            return "MoveForward";
        case CommandType::Rotate:
            return "Rotate";
        case CommandType::Stop:
            return "Stop";
        default:
            return "Unknown";
    }
}

}