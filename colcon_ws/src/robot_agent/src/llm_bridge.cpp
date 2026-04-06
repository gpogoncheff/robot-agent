#include "robot_agent/llm_bridge.hpp"

#include <array>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

namespace robot_agent {

namespace {
    std::string shell_escape_single_quotes(const std::string& s) {
        std::string out;
        for (char c : s) {
            if (c == '\'') {
                out += "'\\''";
            } else {
                out += c;
            }
        }
        return out;
    }

    std::string exec_capture(const std::string& cmd) {
        std::array<char, 256> buffer;
        std::string result;

        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) {
            throw std::runtime_error("Failed to open pipe for command: " + cmd);
        }

        while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
            result += buffer.data();
        }

        const int rc = pclose(pipe);
        if (rc != 0) {
            throw std::runtime_error("Command failed with exit code " + std::to_string(rc));
        }

        return result;
    }
}


std::string LLMBridge::query_tool_plan(const std::string& user_command) {
    const std::string escaped = shell_escape_single_quotes(user_command);
    const std::string cmd = "python3 tools/query_llm.py '" + escaped + "'";
    return exec_capture(cmd);
}

}