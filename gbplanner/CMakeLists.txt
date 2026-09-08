cmake_minimum_required(VERSION 3.8)
project(gbplanner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(kdtree REQUIRED)
find_package(planner_common REQUIRED)
find_package(planner_msgs REQUIRED)
find_package(planner_semantic_msgs REQUIRED)
find_package(map_manager REQUIRED)
find_package(adaptive_obb REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters surface)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)

set(GBPLANNER_DEPS
  rclcpp
  ament_index_cpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  pcl_conversions
  kdtree
  planner_common
  planner_msgs
  planner_semantic_msgs
  map_manager
  adaptive_obb
  grid_map_core
  grid_map_msgs
  grid_map_ros
  behaviortree_cpp
)

add_library(${PROJECT_NAME} SHARED
  src/gbplanner.cpp
  src/rrg.cpp
  src/gbplanner_rviz.cpp
  src/gbplanner_ros.cpp
  src/gbplanner_bt_nodes.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})

ament_target_dependencies(${PROJECT_NAME} ${GBPLANNER_DEPS})
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  glog::glog)

# The ROS 1 build silenced these warnings; keeping the same set means the port
# only surfaces problems it actually introduced, not the pre-existing noise.
target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall
  -Wno-sign-compare
  -Wno-catch-value
  -Wno-switch
  -Wno-maybe-uninitialized
  -Wno-unused-variable
  -Wno-unused-but-set-variable
  -Wno-format
  -Werror=return-type)

add_executable(gbplanner_node src/gbplanner_ros_node.cpp)
target_link_libraries(gbplanner_node ${PROJECT_NAME} glog::glog gflags)
ament_target_dependencies(gbplanner_node ${GBPLANNER_DEPS})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS gbplanner_node DESTINATION lib/${PROJECT_NAME})
# config/ keeps the ROS 1 tree shape: main_tree.xml reaches the shared subtrees
# with ../../../bt_xml/... and BehaviorTree.CPP resolves those relative to the
# including file, so the installed layout has to match the source layout.
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# launch/ is flat, unlike the ROS 1 custom_robot/ and uav/gz/ subdirectories:
# `ros2 launch gbplanner <file>` addresses a launch file by bare filename, and
# docker-compose calls `ros2 launch gbplanner sim_gbplanner.launch.py`.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${GBPLANNER_DEPS} Eigen3 Boost PCL)

ament_package()
