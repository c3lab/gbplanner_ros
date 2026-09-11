cmake_minimum_required(VERSION 3.8)
project(gbplanner_gz_sim)

find_package(ament_cmake REQUIRED)

# No compiled code: this package is worlds, SDF templates, launch and rviz.
install(
  DIRECTORY worlds models launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
