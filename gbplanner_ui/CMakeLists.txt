cmake_minimum_required(VERSION 3.8)
project(gbplanner_ui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall)

# rviz2 on Jazzy links Qt5 (rviz_common depends on qtbase5-dev), so the panel
# must too - mixing Qt versions inside one process does not link.
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(planner_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

add_library(${PROJECT_NAME} SHARED src/gbplanner_ui.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib rclcpp rviz_common planner_msgs std_msgs std_srvs)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Tells rviz2's plugin loader where the panel lives. Without this the class is
# built but never appears in the Panels menu, and any .rviz naming it is dropped.
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(pluginlib rclcpp rviz_common planner_msgs std_srvs)
ament_package()
