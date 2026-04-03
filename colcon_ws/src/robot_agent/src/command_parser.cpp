#include "robot_agent/command_parser.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>


namespace robot_agent {

namespace {
    RobotCommand parse_tool_call(const nlohmann::json& item) {
        if (!item.is_object()) {
            throw std::runtime_error("Tool call must be a json object");
        }

        if (!item.contains("tool")) {
            throw std::runtime_error("Tool call missing required field: tool");
        }

        if (!item.at("tool").is_string()) {
            throw std::runtime_error("Field 'tool' must be a string");
        }

        const std::string tool = item.at("tool").get<std::string>();

        if (!item.contains("arguments")) {
            throw std::runtime_error("Tool call missing required field: arguments");
        }

        if (!item.at("arguments").is_object()) {
            throw std::runtime_error("Arguments field must be a json object");
        }

        const auto& args = item.at("arguments");

        if (tool == "move_forward") {
            if (!args.contains("distance_m")) {
                throw std::runtime_error("move_forward requires argument: distance_m");
            }
            const double distance = args.at("distance_m").get<double>();
            return RobotCommand{CommandType::MoveForward, distance};
        }

        if (tool == "rotate") {
            if (!args.contains("angle_rad")) {
                throw std::runtime_error("rotate requires argument: angle_rad");
            }
            const double angle = args.at("angle_rad").get<double>();
            return RobotCommand{CommandType::Rotate, angle};
        }

        if (tool == "stop") {
            return RobotCommand{CommandType::Stop, 0.0};
        }

        throw std::runtime_error("Unkown tool: " + tool);
    }
}


RobotPlan CommandParser::parse_plan_json(const std::string& json_text) {
    RobotPlan plan;

    const auto j = nlohmann::json::parse(json_text);

    if (j.is_object()) {
        plan.push_back(parse_tool_call(j));
        return plan;
    }

    if (j.is_array()) {
        for (const auto& item : j) {
            plan.push_back(parse_tool_call(item));
        }
        return plan;
    }

    throw std::runtime_error("Plan json ust be either an object or an array");
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