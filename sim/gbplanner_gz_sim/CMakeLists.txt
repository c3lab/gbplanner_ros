cmake_minimum_required(VERSION 3.8)
project(gbplanner_gz_sim)

find_package(ament_cmake REQUIRED)

# No compiled code: this package is worlds, SDF templates, launch, rviz and
# the config for the robot-support nodes it starts.
install(
  DIRECTORY worlds models launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
