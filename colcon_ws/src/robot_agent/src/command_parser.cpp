#include "robot_agent/command_parser.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>


namespace robot_agent {

namespace {
    CommandType parse_command_type(const std::string& action) {
        if (action == "move_forward") {
            return CommandType::MoveForward;
        }
        if (action == "rotate") {
            return CommandType::Rotate;
        }
        if (action == "stop") {
            return CommandType::Stop;
        }

        throw std::runtime_error("Unknown action: " + action);
    }
}

RobotPlan CommandParser::parse_plan_json(const std::string& json_text) {
    RobotPlan plan;

    const auto j = nlohmann::json::parse(json_text);

    if (!j.is_array()) throw std::runtime_error("Plan JSON must be an array");

    for (const auto& item : j) {
        if (!item.is_object()) {
            throw std::runtime_error("Each plan item must be a json object");
        }

        if (!item.contains("action")) {
            throw std::runtime_error("Plan item missing required field: action");
        }

        const std::string action = item.at("action").get<std::string>();
        const CommandType type = parse_command_type(action);

        double value = 0.0;
        if (item.contains("value")) {
            value = item.at("value").get<double>();
        }

        plan.push_back(RobotCommand{type, value});
    }

    return plan;
}

RobotPlan CommandParser::parse_plan_file(const std::string& file_path) {
    std::ifstream in(file_path);
    if (!in) {
        throw std::runtime_error("Faile to open plan file: " + file_path);
    }

    std::stringstream buffer;
    buffer << in.rdbuf();

    return parse_plan_json(buffer.str());
}

}