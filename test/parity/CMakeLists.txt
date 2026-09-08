cmake_minimum_required(VERSION 3.10)
project(gbp_parity C CXX)

# One project file, two configurations. `GBP_PARITY_SIDE` selects which shim and
# which dependency set is used; everything else - flags, source list, include
# layout - is shared, so a difference in the results cannot come from the two
# sides having been compiled differently.
#
# GBP_PARITY_SRC points at the *patched copy* of kdtree/, planner_common/ and
# map_manager/ that build_common.sh makes inside the container. Neither
# repository is ever built from in place.

if(NOT GBP_PARITY_SIDE)
  message(FATAL_ERROR "set -DGBP_PARITY_SIDE=ros1|ros2")
endif()
if(NOT GBP_PARITY_SRC)
  message(FATAL_ERROR "set -DGBP_PARITY_SRC=<patched source copy>")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE None)  # nothing behind our back

# Identical on both sides. -O2 rather than -O3 because GCC 13's -O3 reassociates
# floating-point reductions that GCC 9's does not, and -ffp-contract=off stops
# GCC from folding a*b+c into an FMA on one side only. -march is pinned to the
# baseline so neither compiler's default ISA leaks in.
set(GBP_PARITY_FLAGS
  -O2
  -ffp-contract=off
  -fno-fast-math
  -fno-unsafe-math-optimizations
  -march=x86-64
  -mtune=generic
  -DNDEBUG
  -w                       # the ported sources are noisy; warnings are not the subject
)

# The three compile-time switches map_manager's CMakeLists sets. RAY_CAST_METHOD
# picks between two entirely different getScanStatus bodies, so it has to match.
add_compile_definitions(RAY_CAST_METHOD=0 COL_CHECK_METHOD=0 EDGE_CHECK_METHOD=0)

set(GBP_PARITY_SOURCES
  ${GBP_PARITY_SRC}/kdtree/src/kdtree.c
  ${GBP_PARITY_SRC}/planner_common/src/params.cpp
  ${GBP_PARITY_SRC}/planner_common/src/trajectory.cpp
  ${GBP_PARITY_SRC}/planner_common/src/graph.cpp
  ${GBP_PARITY_SRC}/planner_common/src/graph_manager.cpp
  ${GBP_PARITY_SRC}/planner_common/src/random_sampler.cpp
  ${GBP_PARITY_SRC}/planner_common/src/geofence_manager.cpp
  ${GBP_PARITY_SRC}/map_manager/src/map_manager.cpp
  ${GBP_PARITY_SRC}/map_manager/src/voxblox/voxblox_common_impl.cpp
  ${GBP_PARITY_SRC}/map_manager/src/voxblox/voxblox_alt_impl.cpp
  # Only the two translation units the planning core lives in. gbplanner.cpp,
  # gbplanner_ros.cpp and gbplanner_bt_nodes.cpp are the node wrapper and the
  # behaviour tree, which have nothing to compare.
  ${GBP_PARITY_SRC}/gbplanner/src/rrg.cpp
  ${GBP_PARITY_SRC}/gbplanner/src/gbplanner_rviz.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/unit/parity_body.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/unit/parity_main.cpp
)

add_executable(gbp_parity ${GBP_PARITY_SOURCES})
target_compile_options(gbp_parity PRIVATE ${GBP_PARITY_FLAGS})
target_include_directories(gbp_parity PRIVATE
  ${GBP_PARITY_SRC}/kdtree/include
  ${GBP_PARITY_SRC}/planner_common/include
  ${GBP_PARITY_SRC}/map_manager/include
  ${GBP_PARITY_SRC}/gbplanner/include
  ${CMAKE_CURRENT_SOURCE_DIR}/unit
)

# The seed shim is forced into random_sampler.cpp only; nothing else in the
# suite sees it.
set_source_files_properties(${GBP_PARITY_SRC}/planner_common/src/random_sampler.cpp
  PROPERTIES COMPILE_OPTIONS "-include;${CMAKE_CURRENT_SOURCE_DIR}/unit/gbp_parity_seed.h")

find_package(Eigen3 REQUIRED)
target_include_directories(gbp_parity SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS})

if(GBP_PARITY_SIDE STREQUAL "ros1")
  target_compile_definitions(gbp_parity PRIVATE GBP_PARITY_ROS1)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    std_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    pcl_ros
    pcl_conversions
    voxblox
    voxblox_ros
    planner_msgs
    planner_semantic_msgs
    adaptive_obb
    grid_map_core
    grid_map_msgs
    grid_map_ros
    roslib
    cv_bridge
  )
  find_package(PCL REQUIRED COMPONENTS common filters surface)
  find_package(OpenCV REQUIRED)
  target_include_directories(gbp_parity SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
  target_include_directories(gbp_parity SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(gbp_parity ${catkin_LIBRARIES} ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES} glog gflags protobuf pthread)
elseif(GBP_PARITY_SIDE STREQUAL "ros2")
  target_compile_definitions(gbp_parity PRIVATE GBP_PARITY_ROS2)
  # ament_target_dependencies rather than hand-rolled target names: voxblox and
  # voxblox_ros are ament_auto packages and do not export namespaced targets.
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(planner_msgs REQUIRED)
  find_package(planner_semantic_msgs REQUIRED)
  find_package(voxblox REQUIRED)
  find_package(voxblox_ros REQUIRED)
  find_package(adaptive_obb REQUIRED)
  find_package(grid_map_core REQUIRED)
  find_package(grid_map_msgs REQUIRED)
  find_package(grid_map_ros REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(PCL REQUIRED COMPONENTS common filters surface)
  find_package(OpenCV REQUIRED)
  find_package(glog REQUIRED)
  target_include_directories(gbp_parity SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
  ament_target_dependencies(gbp_parity
    rclcpp
    tf2
    tf2_geometry_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    planner_msgs
    planner_semantic_msgs
    voxblox
    voxblox_ros
    adaptive_obb
    grid_map_core
    grid_map_msgs
    grid_map_ros
    ament_index_cpp
    pcl_conversions
    tf2_ros)
  target_include_directories(gbp_parity SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(gbp_parity ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}
    glog::glog pthread)
else()
  message(FATAL_ERROR "unknown GBP_PARITY_SIDE '${GBP_PARITY_SIDE}'")
endif()
