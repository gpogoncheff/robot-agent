cmake_minimum_required(VERSION 3.8)
project(robot_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(robot_agent_lib
  src/motion_controller.cpp
  src/command.cpp
  src/command_dispatcher.cpp
  src/command_parser.cpp
  src/plan_validator.cpp
  src/llm_bridge.cpp
)

target_include_directories(robot_agent_lib PUBLIC include)
target_link_libraries(robot_agent_lib nlohmann_json::nlohmann_json)
ament_target_dependencies(robot_agent_lib
  rclcpp
  nav_msgs
  geometry_msgs
)

add_executable(command_demo src/command_demo.cpp)
target_link_libraries(command_demo robot_agent_lib)
ament_target_dependencies(command_demo
  rclcpp
  nav_msgs
  geometry_msgs
)

add_executable(file_plan_demo src/file_plan_demo.cpp)
target_link_libraries(file_plan_demo robot_agent_lib)
ament_target_dependencies(file_plan_demo
  rclcpp
  nav_msgs
  geometry_msgs
)

add_executable(llm_agent_demo src/llm_agent_demo.cpp)
target_link_libraries(llm_agent_demo robot_agent_lib)
ament_target_dependencies(llm_agent_demo
  rclcpp
  nav_msgs
  geometry_msgs
)

install(TARGETS
  robot_agent_lib
  command_demo
  file_plan_demo
  llm_agent_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
