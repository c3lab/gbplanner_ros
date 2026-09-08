cmake_minimum_required(VERSION 3.8)
project(planner_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BehaviourPlannerLogger.msg"
  "msg/BoundMode.msg"
  "msg/CoveragePlannerLogger.msg"
  "msg/Edge.msg"
  "msg/ExecutionPathMode.msg"
  "msg/FrontierPoint.msg"
  "msg/Graph.msg"
  "msg/MultipleOpeningDetections.msg"
  "msg/OpeningDetection.msg"
  "msg/PlannerStatus.msg"
  "msg/PlanningBound.msg"
  "msg/PlanningMode.msg"
  "msg/RectangleShape.msg"
  "msg/RobotStatus.msg"
  "msg/TargetCost.msg"
  "msg/TriggerMode.msg"
  "msg/Vertex.msg"
  "srv/PciGeofence.srv"
  "srv/PciGlobal.srv"
  "srv/PciHomingTrigger.srv"
  "srv/PciInitialization.srv"
  "srv/PciSearch.srv"
  "srv/PciSetHomingPos.srv"
  "srv/PciStop.srv"
  "srv/PciToWaypoint.srv"
  "srv/PciTrigger.srv"
  "srv/PlannerDynamicGlobalBound.srv"
  "srv/PlannerGeofence.srv"
  "srv/PlannerGetFrontiers.srv"
  "srv/PlannerValidateFrontiers.srv"
  "srv/PlannerGetTargetCosts.srv"
  "srv/PlannerGlobal.srv"
  "srv/PlannerGoToWaypoint.srv"
  "srv/PlannerHoming.srv"
  "srv/PlannerOpeningApproval.srv"
  "srv/PlannerRequestPath.srv"
  "srv/PlannerSearch.srv"
  "srv/PlannerSetExpMode.srv"
  "srv/PlannerSetGlobalBound.srv"
  "srv/PlannerSetHomingPos.srv"
  "srv/PlannerSetPlanningMode.srv"
  "srv/PlannerSetSearchMode.srv"
  "srv/PlannerSetVel.srv"
  "srv/PlannerSrv.srv"
  "srv/PlannerStringTrigger.srv"
  "action/PathFollowerAction.action"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(
  FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
