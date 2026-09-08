cmake_minimum_required(VERSION 3.8)
project(planner_control_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(planner_common REQUIRED)
find_package(planner_msgs REQUIRED)
find_package(planner_semantic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(PCI_DEPS
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  visualization_msgs
  interactive_markers
  tf2
  tf2_ros
  tf2_geometry_msgs
  planner_common
  planner_msgs
  planner_semantic_msgs
)

add_library(${PROJECT_NAME} SHARED
  src/planner_control_interface.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(${PROJECT_NAME} ${PCI_DEPS})

# The ROS 1 build silenced these warnings; keeping the same set means the port
# only surfaces problems it actually introduced, not the pre-existing noise.
target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall
  -Wno-sign-compare
  -Werror=return-type)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${PCI_DEPS} Eigen3)

ament_package()
