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

add_library(robot_agent_lib
  src/motion_controller.cpp
)

target_include_directories(robot_agent_lib PUBLIC include)

ament_target_dependencies(robot_agent_lib
  rclcpp
  nav_msgs
  geometry_msgs
)

add_executable(motion_demo src/motion_demo.cpp)
target_link_libraries(motion_demo robot_agent_lib)
ament_target_dependencies(motion_demo
  rclcpp
  nav_msgs
  geometry_msgs
)

install(TARGETS
  robot_agent_lib
  motion_demo
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
