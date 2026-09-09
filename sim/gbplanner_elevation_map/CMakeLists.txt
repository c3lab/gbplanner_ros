cmake_minimum_required(VERSION 3.8)
project(gbplanner_elevation_map)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(elevation_map_node
  src/elevation_map_node.cpp
  src/elevation_map_main.cpp
)
target_include_directories(elevation_map_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(elevation_map_node
  grid_map_core
  grid_map_msgs
  grid_map_ros
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_eigen
  tf2_ros
)

install(TARGETS elevation_map_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
