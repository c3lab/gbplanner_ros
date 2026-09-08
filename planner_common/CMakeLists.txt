cmake_minimum_required(VERSION 3.8)
project(planner_common)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(kdtree REQUIRED)
find_package(planner_msgs REQUIRED)
find_package(planner_semantic_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

add_library(${PROJECT_NAME} SHARED
  src/graph_manager.cpp
  src/graph.cpp
  src/params.cpp
  src/trajectory.cpp
  src/random_sampler.cpp
  src/geofence_manager.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  kdtree
  planner_msgs
  planner_semantic_msgs
)
target_link_libraries(${PROJECT_NAME} ${PCL_COMMON_LIBRARIES})

# The ROS 1 build silenced these warnings; keeping the same set means the port
# only surfaces problems it actually introduced, not the pre-existing noise.
target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall
  -Wno-sign-compare
  -Wno-unused-variable
  -Wno-unused-but-set-variable
  -Wno-format
  -Werror=return-type)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  kdtree
  planner_msgs
  planner_semantic_msgs
  Eigen3
  Boost
  PCL)

ament_package()
