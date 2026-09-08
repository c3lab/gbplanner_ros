# Standalone CMake, not an ament package: this binary is a throwaway diagnostic
# and must not end up in the workspace install space that the parity runs use.
cmake_minimum_required(VERSION 3.16)
project(suite_a_quat_client CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(rclcpp REQUIRED)
find_package(planner_msgs REQUIRED)
add_executable(quat_client quat_client.cpp)
target_link_libraries(quat_client rclcpp::rclcpp ${planner_msgs_TARGETS})
