cmake_minimum_required(VERSION 3.8)
project(map_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Ray casting:
#   0 - Original
#   1 - Iterative ray casting
# Exactly one of the two getScanStatus bodies in voxblox_alt_impl.cpp is
# compiled, so this has to stay a compile definition.
add_compile_definitions(RAY_CAST_METHOD=0)

# Point collision checking:
#   0 - Box check
#   1 - Direct T/ESDF
#   2 - Interpolated T/ESDF
# Edge collision checking:
#   0 - Multiple box checks
#   1 - Cuboid around the edge
#   2 - Direct T/ESDF check
#   3 - Interpolated T/ESDF check
# The ROS 1 build never defined either macro, so every `#if (X == 0)` branch was
# taken by the "undefined identifier is 0" preprocessor rule. Spelling them out
# keeps that behaviour while staying safe under -Wundef.
add_compile_definitions(COL_CHECK_METHOD=0 EDGE_CHECK_METHOD=0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(planner_common REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(glog REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/map_manager.cpp
  src/voxblox/voxblox_common_impl.cpp
  src/voxblox/voxblox_alt_impl.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  planner_common
  voxblox
  voxblox_ros
)
target_link_libraries(${PROJECT_NAME} ${PCL_COMMON_LIBRARIES} glog::glog)

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
  planner_common
  voxblox
  voxblox_ros
  Eigen3
  PCL
  glog)

ament_package()
