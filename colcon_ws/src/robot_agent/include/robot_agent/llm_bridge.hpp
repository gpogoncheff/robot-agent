#pragma once

#include <string>

namespace robot_agent {

class LLMBridge {
    public:
        static std::string query_tool_plan(const std::string& user_command);
};

}