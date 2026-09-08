
#include "planner_control_interface/planner_control_interface.h"

#include <chrono>
#include <thread>

#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace explorer {

using namespace std::chrono_literals;

PlannerControlInterface::PlannerControlInterface(
    rclcpp::Node* node, std::shared_ptr<PCIManager> pci_manager)
    : node_(node) {
  main_cb_group_ = pci_manager->mainCallbackGroup();
  client_cb_group_ = pci_manager->clientCallbackGroup();

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = main_cb_group_;

  reference_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "ref_pose", rclcpp::QoS(5));
  planner_status_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "gbplanner_status", rclcpp::QoS(5));
  stop_request_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "planner_control_interface/stop_request", rclcpp::QoS(5));

  odometry_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::QoS(1),
      std::bind(&PlannerControlInterface::odometryCallback, this,
                std::placeholders::_1),
      sub_options);
  pose_sub_ =
      node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "pose", rclcpp::QoS(1),
          std::bind(&PlannerControlInterface::poseCallback, this,
                    std::placeholders::_1),
          sub_options);
  pose_stamped_sub_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "pose_stamped", rclcpp::QoS(1),
          std::bind(&PlannerControlInterface::poseStampedCallback, this,
                    std::placeholders::_1),
          sub_options);

  pci_server_ = node_->create_service<planner_msgs::srv::PciTrigger>(
      "planner_control_interface_trigger",
      std::bind(&PlannerControlInterface::triggerCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), main_cb_group_);
  pci_std_automatic_planning_server_ =
      node_->create_service<std_srvs::srv::Trigger>(
          "planner_control_interface/std_srvs/automatic_planning",
          std::bind(&PlannerControlInterface::stdSrvsAutomaticPlanningCallback,
                    this, std::placeholders::_1, std::placeholders::_2),
          rclcpp::ServicesQoS(), main_cb_group_);
  pci_std_single_planning_server_ =
      node_->create_service<std_srvs::srv::Trigger>(
          "planner_control_interface/std_srvs/single_planning",
          std::bind(&PlannerControlInterface::stdSrvsSinglePlanningCallback,
                    this, std::placeholders::_1, std::placeholders::_2),
          rclcpp::ServicesQoS(), main_cb_group_);
  pci_homing_server_ =
      node_->create_service<planner_msgs::srv::PciHomingTrigger>(
          "pci_homing_trigger",
          std::bind(&PlannerControlInterface::homingCallback, this,
                    std::placeholders::_1, std::placeholders::_2),
          rclcpp::ServicesQoS(), main_cb_group_);
  pci_std_homing_server_ = node_->create_service<std_srvs::srv::Trigger>(
      "planner_control_interface/std_srvs/homing_trigger",
      std::bind(&PlannerControlInterface::stdSrvHomingCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), main_cb_group_);
  pci_std_go_to_waypoint_server_ = node_->create_service<std_srvs::srv::Trigger>(
      "planner_control_interface/std_srvs/go_to_waypoint",
      std::bind(&PlannerControlInterface::stdSrvGoToWaypointCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), main_cb_group_);
  pci_initialization_server_ =
      node_->create_service<planner_msgs::srv::PciInitialization>(
          "pci_initialization_trigger",
          std::bind(&PlannerControlInterface::initializationCallback, this,
                    std::placeholders::_1, std::placeholders::_2),
          rclcpp::ServicesQoS(), main_cb_group_);

  // ROS 1 blocked in the constructor until both planner services answered.
  // That cannot happen here -- the node has to spin for the clients to discover
  // anything at all -- so the clients are created unconnected and run() waits
  // for them instead (see the |services_connected_| gate).
  planner_client_ = node_->create_client<planner_msgs::srv::PlannerSrv>(
      "planner_server", rclcpp::ServicesQoS(), client_cb_group_);
  planner_homing_client_ =
      node_->create_client<planner_msgs::srv::PlannerHoming>(
          "planner_homing_server", rclcpp::ServicesQoS(), client_cb_group_);

  pci_set_homing_pos_server_ =
      node_->create_service<planner_msgs::srv::PciSetHomingPos>(
          "pci_set_homing_pos",
          std::bind(&PlannerControlInterface::setHomingPosCallback, this,
                    std::placeholders::_1, std::placeholders::_2),
          rclcpp::ServicesQoS(), main_cb_group_);
  pci_std_set_homing_pos_server_ =
      node_->create_service<std_srvs::srv::Trigger>(
          "planner_control_interface/std_srvs/set_homing_position_here",
          std::bind(
              &PlannerControlInterface::stdSrvSetHomingPositionHereCallback,
              this, std::placeholders::_1, std::placeholders::_2),
          rclcpp::ServicesQoS(), main_cb_group_);
  planner_set_homing_pos_client_ =
      node_->create_client<planner_msgs::srv::PlannerSetHomingPos>(
          "gbplanner/set_homing_pos", rclcpp::ServicesQoS(), client_cb_group_);

  planner_set_trigger_mode_client_ =
      node_->create_client<planner_msgs::srv::PlannerSetPlanningMode>(
          "/gbplanner/set_planning_trigger_mode", rclcpp::ServicesQoS(),
          client_cb_group_);

  pci_search_server_ = node_->create_service<planner_msgs::srv::PciSearch>(
      "pci_search",
      std::bind(&PlannerControlInterface::searchCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), main_cb_group_);
  planner_search_client_ =
      node_->create_client<planner_msgs::srv::PlannerSearch>(
          "gbplanner/search", rclcpp::ServicesQoS(), client_cb_group_);

  pci_global_server_ = node_->create_service<planner_msgs::srv::PciGlobal>(
      "pci_global",
      std::bind(&PlannerControlInterface::globalPlannerCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), main_cb_group_);
  planner_global_client_ =
      node_->create_client<planner_msgs::srv::PlannerGlobal>(
          "gbplanner/global", rclcpp::ServicesQoS(), client_cb_group_);
  pci_stop_server_ = node_->create_service<planner_msgs::srv::PciStop>(
      "pci_stop",
      std::bind(&PlannerControlInterface::stopPlannerCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), main_cb_group_);
  pci_std_stop_server_ = node_->create_service<std_srvs::srv::Trigger>(
      "planner_control_interface/std_srvs/stop",
      std::bind(&PlannerControlInterface::stdSrvStopPlannerCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), main_cb_group_);

  planner_geofence_client_ =
      node_->create_client<planner_msgs::srv::PlannerGeofence>(
          "gbplanner/geofence", rclcpp::ServicesQoS(), client_cb_group_);
  pci_geofence_server_ = node_->create_service<planner_msgs::srv::PciGeofence>(
      "pci_geofence",
      std::bind(&PlannerControlInterface::addGeofenceCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), main_cb_group_);
  pci_to_waypoint_server_ =
      node_->create_service<planner_msgs::srv::PciToWaypoint>(
          "pci_to_waypoint",
          std::bind(&PlannerControlInterface::goToWaypointCallback, this,
                    std::placeholders::_1, std::placeholders::_2),
          rclcpp::ServicesQoS(), main_cb_group_);

  planner_passing_gate_client_ =
      node_->create_client<planner_msgs::srv::PlannerRequestPath>(
          "gbplanner/passing_gate", rclcpp::ServicesQoS(), client_cb_group_);
  pci_passing_gate_server_ = node_->create_service<std_srvs::srv::Trigger>(
      "planner_control_interface/std_srvs/pass_gate",
      std::bind(&PlannerControlInterface::passingGateCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), main_cb_group_);
  pci_inspection_srv_server_ = node_->create_service<std_srvs::srv::Trigger>(
      "planner_control_interface/std_srvs/inspection_srv_trigger",
      std::bind(&PlannerControlInterface::inspectionSrvCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), main_cb_group_);
  planner_inspection_srv_client_ =
      node_->create_client<planner_msgs::srv::PlannerSrv>(
          "/gbplanner/get_inspection_path", rclcpp::ServicesQoS(),
          client_cb_group_);
  rotate_180_deg_server_ = node_->create_service<std_srvs::srv::Trigger>(
      "pci_rotate_180_trigger",
      std::bind(&PlannerControlInterface::rotate180DegCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), main_cb_group_);

  pci_std_global_last_specified_frontier_server_ =
      node_->create_service<std_srvs::srv::Trigger>(
          "planner_control_interface/std_srvs/replan_last_specified_frontier",
          std::bind(&PlannerControlInterface::
                        stdSrvReplanLastSpecifiedFrontierCallback,
                    this, std::placeholders::_1, std::placeholders::_2),
          rclcpp::ServicesQoS(), main_cb_group_);

  planner_set_exp_mode_client_ =
      node_->create_client<planner_msgs::srv::PlannerSetExpMode>(
          "gbplanner/set_exp_mode", rclcpp::ServicesQoS(), client_cb_group_);

  // The ROS 2 server has no server-id and no internal spin thread; it lives on
  // the node's executor instead.
  imarker_server_ =
      std::make_shared<interactive_markers::InteractiveMarkerServer>(
          "waypoints", node_);

  semantic_server =
      std::make_shared<interactive_markers::InteractiveMarkerServer>(
          "semantics", node_);
  semantic_pub =
      node_->create_publisher<planner_semantic_msgs::msg::SemanticPoint>(
          "semantic_location", rclcpp::QoS(10));

  nav_goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", rclcpp::QoS(1),
      std::bind(&PlannerControlInterface::navGoalCallback, this,
                std::placeholders::_1),
      sub_options);
  pose_goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/global_planner/waypoint_request", rclcpp::QoS(1),
      std::bind(&PlannerControlInterface::poseGoalCallback, this,
                std::placeholders::_1),
      sub_options);
  nav_goal_client_ =
      node_->create_client<planner_msgs::srv::PlannerGoToWaypoint>(
          "gbplanner/go_to_waypoint", rclcpp::ServicesQoS(), client_cb_group_);
  // ROS 1 used queue size 0 here, i.e. an unbounded outgoing queue.
  go_to_waypoint_visualization_pub_ =
      node_->create_publisher<visualization_msgs::msg::Marker>(
          "gbplanner/go_to_waypoint_pose_visualization",
          rclcpp::QoS(rclcpp::KeepAll()));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                         global_verbosity >= Verbosity::WARN,
                         "[PCI]: Setting pci_manager_");
  pci_manager_ = pci_manager;

  RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                         global_verbosity >= Verbosity::WARN,
                         "[PCI]: Loading params");
  if (!loadParams()) {
    RCLCPP_ERROR_EXPRESSION(node_->get_logger(),
                            global_verbosity >= Verbosity::ERROR,
                            "Can not load params. Shut down ros node.");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                         global_verbosity >= Verbosity::WARN,
                         "[PCI]: Initializing pci");
  if (!init()) {
    RCLCPP_ERROR_EXPRESSION(node_->get_logger(),
                            global_verbosity >= Verbosity::ERROR,
                            "Can not initialize the node. Shut down ros node.");
    rclcpp::shutdown();
    return;
  }

  // The ROS 1 constructor tail-called run(), an infinite loop with a manual
  // ros::spinOnce().  Here one iteration of that loop is a timer callback at
  // the same 20 Hz, so the constructor returns and main() can spin the node.
  RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                         global_verbosity >= Verbosity::WARN,
                         "[PCI]: Starting run() loop");
  run_timer_ = node_->create_timer(
      50ms, std::bind(&PlannerControlInterface::run, this), main_cb_group_);
}

void PlannerControlInterface::poseGoalCallback(
    const geometry_msgs::msg::PoseStamped& pose_msgs) {
  setGoal(pose_msgs);
}

void PlannerControlInterface::navGoalCallback(
    const geometry_msgs::msg::PoseStamped& nav_msgs) {
  geometry_msgs::msg::PoseStamped posest;
  posest.header = nav_msgs.header;
  posest.pose = nav_msgs.pose;
  setGoal(posest);
}

void PlannerControlInterface::setGoal(
    const geometry_msgs::msg::PoseStamped& pose) {
  geometry_msgs::msg::PoseStamped pose_in_world_frame;

  geometry_msgs::msg::TransformStamped darpa_to_world_transform;
  try {
    darpa_to_world_transform = tf_buffer_->lookupTransform(
        world_frame_id_, pose.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR_EXPRESSION(node_->get_logger(),
                            global_verbosity >= Verbosity::ERROR,
                            "[gbplanner_pci::setGoal] %s", ex.what());
    return;
  }
  tf2::doTransform(pose, pose_in_world_frame, darpa_to_world_transform);
  pose_in_world_frame.header.stamp = darpa_to_world_transform.header.stamp;
  pose_in_world_frame.header.frame_id = world_frame_id_;

  received_first_waypoint_to_go_ = true;
  go_to_waypoint_with_checking_ = true;
  set_waypoint_stamped_ = pose_in_world_frame;
  RCLCPP_INFO_EXPRESSION(
      node_->get_logger(), global_verbosity >= Verbosity::INFO,
      "[gbplanner_pci::setGoal] set waypoint to (%.2f, %.2f, %.2f) in frame "
      "'%s'",
      pose_in_world_frame.pose.position.x, pose_in_world_frame.pose.position.y,
      pose_in_world_frame.pose.position.z,
      pose_in_world_frame.header.frame_id.c_str());
  rclcpp::Rate rr(10.0, node_->get_clock());  // 10Hz
  for (int i = 0; i < 5; ++i) {
    publishGoToWaypointVisualization(set_waypoint_stamped_);
    rr.sleep();
  }
}

void PlannerControlInterface::passingGateCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
  (void)req;
  // ROS 1 signalled "already active" by returning false from the callback,
  // which the caller saw as a failed service call; a ROS 2 callback is void, so
  // the same information has to travel in the response.
  if (!passing_gate_success_) {
    passing_gate_request_ = true;
    res->success = true;
  } else {
    RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                           global_verbosity >= Verbosity::WARN,
                           "Passing gate already activated.");
    res->success = false;
  }
}

void PlannerControlInterface::inspectionSrvCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
  (void)req;
  //
  if (!inspection_srv_request_) {
    inspection_srv_request_ = true;
    res->success = true;
  } else {
    RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                           global_verbosity >= Verbosity::WARN,
                           "Inspection service already activated.");
    res->success = false;
  }
}

void PlannerControlInterface::resetPlanner() {
  // Set back to manual mode, and stop all current requests.
  trigger_mode_ = PlannerTriggerModeType::kManual;
  run_en_ = false;
  search_request_ = false;
  homing_request_ = false;
  init_request_ = false;
  global_request_ = false;
  go_to_waypoint_request_ = false;
  go_to_waypoint_with_checking_ = false;
  inspection_srv_request_ = false;

  // Remove the last waypoint to prevent the planner starts from that last wp.
  current_path_.clear();
  pci_manager_->setStatus(PCIManager::PCIStatus::kReady);
}

void PlannerControlInterface::rotate180DegCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
  (void)req;
  auto planning_mode_req =
      std::make_shared<planner_msgs::srv::PlannerSetPlanningMode::Request>();
  planning_mode_req->planning_mode =
      planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  callService(planner_set_trigger_mode_client_, planning_mode_req);

  go_to_waypoint_request_ = true;
  go_to_waypoint_with_checking_ = false;
  set_waypoint_.position.x = current_pose_.position.x;
  set_waypoint_.position.y = current_pose_.position.y;
  set_waypoint_.position.z = current_pose_.position.z;
  set_waypoint_.orientation.x = 0.0;
  set_waypoint_.orientation.y = 0.0;
  set_waypoint_.orientation.z = 1.0;
  set_waypoint_.orientation.w = 0.0;
  res->success = true;
}

void PlannerControlInterface::goToWaypointCallback(
    const planner_msgs::srv::PciToWaypoint::Request::SharedPtr req,
    planner_msgs::srv::PciToWaypoint::Response::SharedPtr res) {
  (void)res;
  auto planning_mode_req =
      std::make_shared<planner_msgs::srv::PlannerSetPlanningMode::Request>();
  planning_mode_req->planning_mode =
      planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  callService(planner_set_trigger_mode_client_, planning_mode_req);

  go_to_waypoint_request_ = true;
  go_to_waypoint_with_checking_ = false;
  set_waypoint_.position.x = req->waypoint.position.x;
  set_waypoint_.position.y = req->waypoint.position.y;
  set_waypoint_.position.z = req->waypoint.position.z;
  set_waypoint_.orientation.x = req->waypoint.orientation.x;
  set_waypoint_.orientation.y = req->waypoint.orientation.y;
  set_waypoint_.orientation.z = req->waypoint.orientation.z;
  set_waypoint_.orientation.w = req->waypoint.orientation.w;
}

void PlannerControlInterface::searchCallback(
    const planner_msgs::srv::PciSearch::Request::SharedPtr req,
    planner_msgs::srv::PciSearch::Response::SharedPtr res) {
  (void)res;
  search_request_ = true;
  exe_path_en_ = !req->not_exe_path;
  use_current_state_ = req->use_current_state;
  bound_mode_ = req->bound_mode;
}

void PlannerControlInterface::globalPlannerCallback(
    const planner_msgs::srv::PciGlobal::Request::SharedPtr req,
    planner_msgs::srv::PciGlobal::Response::SharedPtr res) {
  global_request_ = true;
  exe_path_en_ = !req->not_exe_path;
  bound_mode_ = req->bound_mode;
  frontier_id_ = req->id;
  pci_global_request_params_ = *req;
  res->success = true;
}

void PlannerControlInterface::stopPlannerCallback(
    const planner_msgs::srv::PciStop::Request::SharedPtr req,
    planner_msgs::srv::PciStop::Response::SharedPtr res) {
  (void)req;
  RCLCPP_INFO_EXPRESSION(node_->get_logger(),
                         global_verbosity >= Verbosity::INFO,
                         "[PlannerControlInterface::stopPlannerCallback]");
  std_msgs::msg::Bool stop_msg;
  stop_request_pub_->publish(stop_msg);
  pci_manager_->stopPCI();
  stop_planner_request_ = true;
  resetPlanner();

  // planner_msgs::srv::PlannerSetPlanningMode planning_mode_srv;
  // planning_mode_srv.request.planning_mode =
  //     planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  // planner_set_trigger_mode_client_.call(planning_mode_srv);

  res->success = true;
  RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                         global_verbosity >= Verbosity::PLANNER_STATUS,
                         "[PCI] STOP PLANNER.");
}

void PlannerControlInterface::stdSrvStopPlannerCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
  (void)req;
  auto stop_request = std::make_shared<planner_msgs::srv::PciStop::Request>();
  auto stop_response = std::make_shared<planner_msgs::srv::PciStop::Response>();

  stopPlannerCallback(stop_request, stop_response);
  res->success = stop_response->success;
}

void PlannerControlInterface::addGeofenceCallback(
    const planner_msgs::srv::PciGeofence::Request::SharedPtr req,
    planner_msgs::srv::PciGeofence::Response::SharedPtr res) {
  auto plan_req =
      std::make_shared<planner_msgs::srv::PlannerGeofence::Request>();
  plan_req->rectangles = req->rectangles;
  auto plan_res = callService(planner_geofence_client_, plan_req);
  if (plan_res) res->success = plan_res->success;
}

void PlannerControlInterface::setHomingPosCallback(
    const planner_msgs::srv::PciSetHomingPos::Request::SharedPtr req,
    planner_msgs::srv::PciSetHomingPos::Response::SharedPtr res) {
  (void)req;
  // Bypass this request to the planner.
  auto plan_req =
      std::make_shared<planner_msgs::srv::PlannerSetHomingPos::Request>();
  auto plan_res = callService(planner_set_homing_pos_client_, plan_req);
  if (plan_res) {
    res->success = plan_res->success;
  }
}

void PlannerControlInterface::stdSrvSetHomingPositionHereCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
  (void)req;
  auto set_homing_pos_request =
      std::make_shared<planner_msgs::srv::PciSetHomingPos::Request>();
  auto set_homing_pos_response =
      std::make_shared<planner_msgs::srv::PciSetHomingPos::Response>();

  setHomingPosCallback(set_homing_pos_request, set_homing_pos_response);
  res->success = set_homing_pos_response->success;
}

void PlannerControlInterface::initializationCallback(
    const planner_msgs::srv::PciInitialization::Request::SharedPtr req,
    planner_msgs::srv::PciInitialization::Response::SharedPtr res) {
  (void)req;
  init_request_ = true;
  res->success = true;
}

void PlannerControlInterface::homingCallback(
    const planner_msgs::srv::PciHomingTrigger::Request::SharedPtr req,
    planner_msgs::srv::PciHomingTrigger::Response::SharedPtr res) {
  exe_path_en_ = !req->not_exe_path;
  homing_request_ = true;
  res->success = true;
}

void PlannerControlInterface::stdSrvHomingCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
  (void)req;
  auto homing_trigger_request =
      std::make_shared<planner_msgs::srv::PciHomingTrigger::Request>();
  auto homing_trigger_response =
      std::make_shared<planner_msgs::srv::PciHomingTrigger::Response>();

  homing_trigger_request->not_exe_path = false;

  homingCallback(homing_trigger_request, homing_trigger_response);
  res->success = homing_trigger_response->success;
}

void PlannerControlInterface::triggerCallback(
    const planner_msgs::srv::PciTrigger::Request::SharedPtr req,
    planner_msgs::srv::PciTrigger::Response::SharedPtr res) {
  if (pci_manager_->getStatus() == PCIManager::PCIStatus::kError) {
    RCLCPP_WARN_EXPRESSION(
        node_->get_logger(), global_verbosity >= Verbosity::WARN,
        "PCIManager is curretely in error state and cannot accept planning "
        "requests.");
    res->success = false;
  } else {
    if ((!req->set_auto) && (trigger_mode_ == PlannerTriggerModeType::kAuto)) {
      RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                             global_verbosity >= Verbosity::WARN,
                             "Switch to manual mode.");
      trigger_mode_ = PlannerTriggerModeType::kManual;
    } else if ((req->set_auto) &&
               (trigger_mode_ == PlannerTriggerModeType::kManual)) {
      RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                             global_verbosity >= Verbosity::WARN,
                             "Switch to auto mode.");
      trigger_mode_ = PlannerTriggerModeType::kAuto;
    }
    pci_manager_->setVelocity(req->vel_max);
    bound_mode_ = req->bound_mode;
    run_en_ = true;
    exe_path_en_ = !req->not_exe_path;
    res->success = true;
  }
}

void PlannerControlInterface::stdSrvsAutomaticPlanningCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
  (void)req;
  auto pci_trigger_request =
      std::make_shared<planner_msgs::srv::PciTrigger::Request>();
  auto pci_trigger_response =
      std::make_shared<planner_msgs::srv::PciTrigger::Response>();
  pci_trigger_request->not_exe_path = false;
  pci_trigger_request->set_auto = true;
  pci_trigger_request->bound_mode = 0;
  pci_trigger_request->vel_max = 0.0;

  triggerCallback(pci_trigger_request, pci_trigger_response);
  res->success = pci_trigger_response->success;
}

void PlannerControlInterface::stdSrvGoToWaypointCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
  (void)req;
  auto planning_mode_req =
      std::make_shared<planner_msgs::srv::PlannerSetPlanningMode::Request>();
  planning_mode_req->planning_mode =
      planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  callService(planner_set_trigger_mode_client_, planning_mode_req);

  if (received_first_waypoint_to_go_) {
    go_to_waypoint_request_ = true;
    go_to_waypoint_with_checking_ = true;
    res->success = true;
  } else {
    RCLCPP_ERROR_EXPRESSION(
        node_->get_logger(), global_verbosity >= Verbosity::ERROR,
        "No waypoint was set, 'go_to_waypoint' feature will not be triggered.");
    res->success = false;
  }
}

void PlannerControlInterface::stdSrvsSinglePlanningCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
  (void)req;
  auto pci_trigger_request =
      std::make_shared<planner_msgs::srv::PciTrigger::Request>();
  auto pci_trigger_response =
      std::make_shared<planner_msgs::srv::PciTrigger::Response>();
  pci_trigger_request->not_exe_path = false;
  pci_trigger_request->set_auto = false;
  pci_trigger_request->bound_mode = 0;
  pci_trigger_request->vel_max = 0.0;

  triggerCallback(pci_trigger_request, pci_trigger_response);
  res->success = pci_trigger_response->success;
}

void PlannerControlInterface::stdSrvReplanLastSpecifiedFrontierCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
  (void)req;
  auto pci_global_request =
      std::make_shared<planner_msgs::srv::PciGlobal::Request>();
  auto pci_global_response =
      std::make_shared<planner_msgs::srv::PciGlobal::Response>();
  pci_global_request->not_exe_path = false;
  pci_global_request->set_auto = false;
  pci_global_request->bound_mode = 0;
  pci_global_request->vel_max = 0.0;
  // Use last frontier specified via service call.
  pci_global_request->id = frontier_id_;

  globalPlannerCallback(pci_global_request, pci_global_response);
  res->success = pci_global_response->success;
}

bool PlannerControlInterface::init() {
  planner_iteration_ = 0;
  reference_pub_id_ = 0;
  homing_request_ = false;
  run_en_ = false;
  exe_path_en_ = true;
  pose_is_ready_ = false;
  bound_mode_ = planner_msgs::srv::PlannerSrv::Request::EXTENDED_BOUND;
  force_forward_ = true;
  init_request_ = false;
  search_request_ = false;
  global_request_ = false;
  stop_planner_request_ = false;
  passing_gate_request_ = false;
  passing_gate_success_ = false;
  go_to_waypoint_request_ = false;
  // Waiting for the system to be ready (odometry, planner services) used to
  // happen here with a nested spin; run() does it now without blocking, see
  // |services_connected_| and |initialized_|.
  return true;
}

void PlannerControlInterface::run() {
  if (!services_connected_) {
    // Throttled to the 1 Hz of the ROS 1 sleep(1) retry loops; the tick that
    // replaced them runs at 20 Hz.
    if (!planner_client_->service_is_ready()) {
      if (global_verbosity >= Verbosity::WARN)
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "PCI: service planner_server is not available: waiting...");
      return;
    }
    if (!planner_homing_client_->service_is_ready()) {
      if (global_verbosity >= Verbosity::WARN)
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "PCI: service planner_homing_server is not available: waiting...");
      return;
    }
    RCLCPP_INFO_EXPRESSION(node_->get_logger(),
                           global_verbosity >= Verbosity::INFO,
                           "PCI: connected to service planner_server.");
    RCLCPP_INFO_EXPRESSION(node_->get_logger(),
                           global_verbosity >= Verbosity::INFO,
                           "PCI: connected to service planner_homing_server.");
    services_connected_ = true;
  }

  if (!initialized_) {
    // Both interactive markers are placed relative to current_pose_, so they
    // genuinely need odometry first.
    if (!pose_is_ready_) {
      if (global_verbosity >= Verbosity::DEBUG)
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Waiting for odometry.");
      return;
    }
    RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                           global_verbosity >= Verbosity::DEBUG,
                           "[PCI]:[init()]: recieved odom");
    initIMarker();
    RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                           global_verbosity >= Verbosity::DEBUG,
                           "[PCI]:[init()]: initIMarker");
    initSemanticIMarker();
    RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                           global_verbosity >= Verbosity::DEBUG,
                           "[PCI]:[init()]: initSemanticIMarker");
    if (!pci_manager_->initialize()) {
      RCLCPP_ERROR_EXPRESSION(
          node_->get_logger(), global_verbosity >= Verbosity::ERROR,
          "Can not initialize the node. Shut down ros node.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                           global_verbosity >= Verbosity::DEBUG,
                           "[PCI]:[init()]: pci manager init done");
    initialized_ = true;
  }

  PCIManager::PCIStatus pci_status = pci_manager_->getStatus();
  // TODO: Fix by prioritizing and sequencing exclusive cases (with bad
  // if/else and flags approach)
  if (pci_status == PCIManager::PCIStatus::kReady) {
    // Priority 1: Check if require homing.
    if (homing_request_) {
      homing_request_ = false;
      trigger_mode_ = PlannerTriggerModeType::kManual;  // also unset auto
                                                        // mode
      RCLCPP_INFO_EXPRESSION(node_->get_logger(),
                             global_verbosity >= Verbosity::INFO,
                             "PlannerControlInterface: Running Homing");
      runHoming(exe_path_en_);
    }
    // Priority 2: Check if require initialization step.
    else if (init_request_) {
      init_request_ = false;
      RCLCPP_INFO_EXPRESSION(node_->get_logger(),
                             global_verbosity >= Verbosity::INFO,
                             "PlannerControlInterface: Running Initialization");
      runInitialization();
      // Priority 3: Stop
    } else if (stop_planner_request_) {
      // Stop at current pose.
      RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                             global_verbosity >= Verbosity::WARN,
                             "PCI: run: stop requested");
      stop_planner_request_ = false;
      pci_manager_->goToWaypoint(current_pose_);
    } else if ((trigger_mode_ == PlannerTriggerModeType::kAuto) || (run_en_)) {
      run_en_ = false;
      RCLCPP_INFO_EXPRESSION(node_->get_logger(),
                             global_verbosity >= Verbosity::INFO,
                             "PlannerControlInterface: Running Planner (%s)",
                             std::string(trigger_mode_ ==
                                                 PlannerTriggerModeType::kAuto
                                             ? "kAuto"
                                             : "kManual")
                                 .c_str());
      runPlanner(exe_path_en_);
    } else if (search_request_) {
      search_request_ = false;
      RCLCPP_INFO_EXPRESSION(
          node_->get_logger(), global_verbosity >= Verbosity::INFO,
          "Request the planner to search for connection path.");
      runSearch(exe_path_en_);
    } else if (global_request_) {
      global_request_ = false;
      RCLCPP_INFO_EXPRESSION(node_->get_logger(),
                             global_verbosity >= Verbosity::INFO,
                             "Request the global planner.");
      runGlobalPlanner(exe_path_en_);
    } else if (passing_gate_request_) {
      passing_gate_request_ = false;
      runPassingGate();
    } else if (inspection_srv_request_) {
      inspection_srv_request_ = false;
      runInspection();
    } else if (go_to_waypoint_request_) {
      go_to_waypoint_request_ = false;
      if (!go_to_waypoint_with_checking_)
        pci_manager_->goToWaypoint(set_waypoint_);
      else
        runGlobalRepositioning();
    }
  } else if (pci_status == PCIManager::PCIStatus::kError) {
    // For ANYmal, reset everything to manual then wait for operator.
    resetPlanner();
  }
}

void PlannerControlInterface::runGlobalRepositioning() {
  RCLCPP_INFO_EXPRESSION(node_->get_logger(),
                         global_verbosity >= Verbosity::PLANNER_STATUS,
                         "Global Repositioning %i", planner_iteration_);

  auto planning_mode_req =
      std::make_shared<planner_msgs::srv::PlannerSetPlanningMode::Request>();
  planning_mode_req->planning_mode =
      planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  callService(planner_set_trigger_mode_client_, planning_mode_req);

  auto planner_req =
      std::make_shared<planner_msgs::srv::PlannerGoToWaypoint::Request>();
  planner_req->check_collision = true;
  planner_req->waypoint.header = set_waypoint_stamped_.header;
  planner_req->waypoint.pose = set_waypoint_stamped_.pose;

  auto planner_res = callService(nav_goal_client_, planner_req);
  if (planner_res) {
    if (!planner_res->path.empty()) {
      // Execute path.
      current_path_.clear();
      // resetPlanner();
      std::vector<geometry_msgs::msg::Pose> path_to_be_exe;
      pci_manager_->executePath(planner_res->path, path_to_be_exe,
                                PCIManager::ExecutionPathType::kGlobalPath);
      current_path_ = path_to_be_exe;
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Will not execute the path.");
      node_->get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.5));
    }
    planner_iteration_++;
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Planner service failed");
    node_->get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.5));
  }
}

void PlannerControlInterface::runPassingGate() {
  bool success = true;

  auto planning_mode_req =
      std::make_shared<planner_msgs::srv::PlannerSetPlanningMode::Request>();
  planning_mode_req->planning_mode =
      planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  callService(planner_set_trigger_mode_client_, planning_mode_req);

  auto plan_req =
      std::make_shared<planner_msgs::srv::PlannerRequestPath::Request>();
  auto plan_res = callService(planner_passing_gate_client_, plan_req);
  if ((!plan_res) || (plan_res->path.empty())) success = false;

  std_msgs::msg::Bool planner_success_msg;
  planner_success_msg.data = success;
  planner_status_pub_->publish(planner_success_msg);

  if (success) {
    passing_gate_success_ = true;
    std::vector<geometry_msgs::msg::Pose> path_to_be_exe;
    pci_manager_->executePath(plan_res->path, path_to_be_exe);
  }
}

void PlannerControlInterface::runInspection() {
  auto plan_req = std::make_shared<planner_msgs::srv::PlannerSrv::Request>();
  plan_req->header.stamp = node_->now();
  plan_req->header.frame_id = world_frame_id_;
  plan_req->bound_mode = 0;
  RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                         global_verbosity >= Verbosity::INFO,
                         "[PCI]: Called inspection srv");
  auto plan_res = callService(planner_inspection_srv_client_, plan_req);
  if (plan_res) {
    std::vector<geometry_msgs::msg::Pose> path_to_be_exe;
    pci_manager_->executePath(plan_res->path, path_to_be_exe,
                              PCIManager::ExecutionPathType::kManualPath);
    current_path_ = path_to_be_exe;
  }
  ++planner_iteration_;
}

void PlannerControlInterface::runGlobalPlanner(bool exe_path = false) {
  RCLCPP_INFO_EXPRESSION(node_->get_logger(),
                         global_verbosity >= Verbosity::PLANNER_STATUS,
                         "Planning iteration %i", planner_iteration_);

  auto planning_mode_req =
      std::make_shared<planner_msgs::srv::PlannerSetPlanningMode::Request>();
  planning_mode_req->planning_mode =
      planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  callService(planner_set_trigger_mode_client_, planning_mode_req);

  auto plan_req = std::make_shared<planner_msgs::srv::PlannerGlobal::Request>();
  plan_req->id = pci_global_request_params_.id;
  plan_req->not_check_frontier = pci_global_request_params_.not_check_frontier;
  plan_req->ignore_time = pci_global_request_params_.ignore_time;
  auto plan_res = callService(planner_global_client_, plan_req);
  if (plan_res) {
    if ((exe_path) && (!plan_res->path.empty())) {
      // Execute path.
      std::vector<geometry_msgs::msg::Pose> path_to_be_exe;
      pci_manager_->executePath(plan_res->path, path_to_be_exe,
                                PCIManager::ExecutionPathType::kGlobalPath);
      current_path_ = path_to_be_exe;
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Will not execute the path.");
      node_->get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.5));
    }
    planner_iteration_++;
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Planner service failed");
    node_->get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.5));
  }
}

void PlannerControlInterface::runPlanner(bool exe_path = false) {
  const int kBBoxLevel = 3;
  bool success = false;

  // planner_msgs::srv::PlannerSetPlanningMode planning_mode_srv;
  // if (trigger_mode_ == PlannerTriggerModeType::kAuto) {
  //   planning_mode_srv.request.planning_mode =
  //       planner_msgs::srv::PlannerSetPlanningMode::Request::AUTO;
  // } else {
  //   planning_mode_srv.request.planning_mode =
  //       planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  // }

  // RCLCPP_WARN(node_->get_logger(), "[PCI]: Called trigger mode srv");
  // planner_set_trigger_mode_client_.call(planning_mode_srv);

  for (int ind = 0; ind < kBBoxLevel; ++ind) {
    if (stop_planner_request_) return;

    bound_mode_ = ind;
    RCLCPP_INFO_EXPRESSION(node_->get_logger(),
                           global_verbosity >= Verbosity::PLANNER_STATUS,
                           "Planning iteration %i", planner_iteration_);
    auto plan_req = std::make_shared<planner_msgs::srv::PlannerSrv::Request>();
    plan_req->header.stamp = node_->now();
    plan_req->header.frame_id = world_frame_id_;
    plan_req->bound_mode = bound_mode_;
    plan_req->root_pose = getPoseToStart();
    if (ind > 0) {
      plan_req->root_pose.position.x = 0.0;
      plan_req->root_pose.position.y = 0.0;
      plan_req->root_pose.position.z = 0.0;
      plan_req->root_pose.orientation.x = 0.0;
      plan_req->root_pose.orientation.y = 0.0;
      plan_req->root_pose.orientation.z = 0.0;
      plan_req->root_pose.orientation.w = 1.0;
    }
    RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                           global_verbosity >= Verbosity::ERROR,
                           "[PCI]: Called plan srv");
    auto plan_res = callService(planner_client_, plan_req);
    if (plan_res) {
      if (!plan_res->path.empty()) {
        // Execute path.
        if (exe_path) {
          if ((!force_forward_) ||
              (plan_res->status !=
               planner_msgs::srv::PlannerSrv::Response::BACKWARD) ||
              (ind == (kBBoxLevel - 1))) {
            if (ind == (kBBoxLevel - 1))
              RCLCPP_WARN_EXPRESSION(
                  node_->get_logger(), global_verbosity >= Verbosity::WARN,
                  "Using minimum bound, pick the current best one regardless "
                  "the direction.");
            current_path_.clear();
            std::vector<geometry_msgs::msg::Pose> path_to_be_exe;
            PCIManager::ExecutionPathType path_type =
                PCIManager::ExecutionPathType::kLocalPath;

            RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                                   global_verbosity >= Verbosity::DEBUG,
                                   "[PCI]: returned status: %d",
                                   plan_res->status);
            if (plan_res->status ==
                planner_msgs::srv::PlannerSrv::Response::HOMING) {
              // Perform homing step, set back to manual mode, and stop all
              // current requests.
              resetPlanner();
              path_type = PCIManager::ExecutionPathType::kHomingPath;
            } else if (plan_res->status ==
                       planner_msgs::srv::PlannerSrv::Response::
                           AUTO_CUSTOM_PATH) {
              // Perform homing step, set back to manual mode, and stop all
              // current requests.
              RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                                     global_verbosity >= Verbosity::DEBUG,
                                     "[PCI]: Auto Custom Path");
              path_type = PCIManager::ExecutionPathType::kManualPath;
            } else if (plan_res->status ==
                       planner_msgs::srv::PlannerSrv::Response::
                           MANUAL_CUSTOM_PATH) {
              // Perform homing step, set back to manual mode, and stop all
              // current requests.
              RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                                     global_verbosity >= Verbosity::DEBUG,
                                     "[PCI]: Manual Custom Path");
              resetPlanner();
              path_type = PCIManager::ExecutionPathType::kManualPath;
            } else {
              RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                                     global_verbosity >= Verbosity::DEBUG,
                                     "[PCI]: Local Path");
            }
            v_current_ = pci_manager_->getVelocity(path_type);
            // Publish the status
            publishPlannerStatus(*plan_res, true);
            pci_manager_->executePath(plan_res->path, path_to_be_exe,
                                      path_type);
            success = true;
            current_path_ = path_to_be_exe;
          } else if (ind < (kBBoxLevel - 1)) {
            publishPlannerStatus(*plan_res, false);
            RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                                   global_verbosity >= Verbosity::WARN,
                                   "Attemp to re-plan with smaller bound.");
          }
        }
      } else {
        publishPlannerStatus(*plan_res, false);
        RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                               global_verbosity >= Verbosity::WARN,
                               "Planner returned an empty path");
        if (plan_res->status ==
                planner_msgs::srv::PlannerSrv::Response::HOMING ||
            plan_res->status ==
                planner_msgs::srv::PlannerSrv::Response::MANUAL_CUSTOM_PATH) {
          // Ran out of time budget or already at home. Stop and reset planner
          resetPlanner();
          success = true;
        }
      }
      planner_iteration_++;
      if (success) break;
    } else {
      RCLCPP_ERROR_EXPRESSION(node_->get_logger(),
                              global_verbosity >= Verbosity::ERROR,
                              "Planner service failed");
      node_->get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.5));
    }
  }

  // Reset default mode again.
  bound_mode_ = 0;
}

void PlannerControlInterface::publishPlannerStatus(
    const planner_msgs::srv::PlannerSrv::Response& res, bool success) {
  (void)res;
  std_msgs::msg::Bool planner_success_msg;
  planner_success_msg.data = success;
  planner_status_pub_->publish(planner_success_msg);
}

void PlannerControlInterface::publishGoToWaypointVisualization(
    const geometry_msgs::msg::PoseStamped& poseStamped) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = poseStamped.header.frame_id;
  marker.header.stamp = poseStamped.header.stamp;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = poseStamped.pose;
  marker.pose.position.z += 0.5;
  marker.scale.x = 4.0;
  marker.scale.y = 0.45;
  marker.scale.z = 0.45;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  go_to_waypoint_visualization_pub_->publish(marker);
}

void PlannerControlInterface::runHoming(bool exe_path) {
  auto plan_req = std::make_shared<planner_msgs::srv::PlannerHoming::Request>();
  plan_req->header.stamp = node_->now();
  plan_req->header.frame_id = world_frame_id_;
  trigger_mode_ = PlannerTriggerModeType::kAuto;
  if (callService(planner_homing_client_, plan_req)) {
    runPlanner(exe_path);
  }
}

void PlannerControlInterface::runInitialization() {
  RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                         global_verbosity >= Verbosity::WARN,
                         "Start initialization ...");
  auto planning_mode_req =
      std::make_shared<planner_msgs::srv::PlannerSetPlanningMode::Request>();
  planning_mode_req->planning_mode =
      planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  callService(planner_set_trigger_mode_client_, planning_mode_req);

  pci_manager_->initMotion();
}

void PlannerControlInterface::runSearch(bool exe_path) {
  RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                         global_verbosity >= Verbosity::WARN,
                         "Start searching ...");

  auto planning_mode_req =
      std::make_shared<planner_msgs::srv::PlannerSetPlanningMode::Request>();
  planning_mode_req->planning_mode =
      planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  callService(planner_set_trigger_mode_client_, planning_mode_req);

  auto plan_req = std::make_shared<planner_msgs::srv::PlannerSearch::Request>();
  plan_req->header.stamp = node_->now();
  plan_req->header.frame_id = world_frame_id_;
  plan_req->use_current_state = use_current_state_;
  plan_req->bound_mode = bound_mode_;

  {
    // Held only for the copy, not across the service call below.
    std::lock_guard<std::mutex> lock(setpoint_mutex_);
  if (!use_current_state_) {
    plan_req->source.position.x = source_setpoint_.position.x;
    plan_req->source.position.y = source_setpoint_.position.y;
    plan_req->source.position.z = source_setpoint_.position.z;
    plan_req->source.orientation.x = source_setpoint_.orientation.x;
    plan_req->source.orientation.y = source_setpoint_.orientation.y;
    plan_req->source.orientation.z = source_setpoint_.orientation.z;
    plan_req->source.orientation.w = source_setpoint_.orientation.w;
  } else {
    plan_req->source.position.x = source_setpoint_.position.x;
    plan_req->source.position.y = source_setpoint_.position.y;
    plan_req->source.position.z = source_setpoint_.position.z;
    plan_req->source.orientation.x = source_setpoint_.orientation.x;
    plan_req->source.orientation.y = source_setpoint_.orientation.y;
    plan_req->source.orientation.z = source_setpoint_.orientation.z;
    plan_req->source.orientation.w = source_setpoint_.orientation.w;
  }
  plan_req->target.position.x = target_setpoint_.position.x;
  plan_req->target.position.y = target_setpoint_.position.y;
  plan_req->target.position.z = target_setpoint_.position.z;
  plan_req->target.orientation.x = target_setpoint_.orientation.x;
  plan_req->target.orientation.y = target_setpoint_.orientation.y;
  plan_req->target.orientation.z = target_setpoint_.orientation.z;
  plan_req->target.orientation.w = target_setpoint_.orientation.w;
  }

  auto plan_res = callService(planner_search_client_, plan_req);
  if (plan_res) {
    if (!plan_res->path.empty()) {
      if (exe_path) {
        std::vector<geometry_msgs::msg::Pose> path_to_be_exe;
        pci_manager_->executePath(plan_res->path, path_to_be_exe);
        current_path_ = path_to_be_exe;
      }
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Planner Search returned an empty path");
    }
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Planner Search service failed");
    node_->get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.5));
  }
  planner_iteration_++;
}

geometry_msgs::msg::Pose PlannerControlInterface::getPoseToStart() {
  geometry_msgs::msg::Pose ret;
  // use current state as default
  ret.position.x = 0.0;
  ret.position.y = 0.0;
  ret.position.z = 0.0;
  ret.orientation.x = 0.0;
  ret.orientation.y = 0.0;
  ret.orientation.z = 0.0;
  ret.orientation.w = 1.0;

  // Use the last waypoint as a starting pose if required to plan ahead
  if (pci_manager_->planAhead() && (current_path_.size()))
    ret = current_path_.back();
  return ret;
}

bool PlannerControlInterface::loadParams() {
  RCLCPP_INFO_EXPRESSION(node_->get_logger(),
                         global_verbosity >= Verbosity::INFO, "Loading: %s",
                         node_->get_fully_qualified_name());

  // Required params for robot interface.
  if (!pci_manager_->loadParams()) return false;

  // Other params.
  std::string parse_str;
  getParamOpt(node_, "trigger_mode", parse_str);
  if (!parse_str.compare("kManual"))
    trigger_mode_ = PlannerTriggerModeType::kManual;
  else if (!parse_str.compare("kAuto"))
    trigger_mode_ = PlannerTriggerModeType::kAuto;
  else {
    trigger_mode_ = PlannerTriggerModeType::kManual;
    RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                           global_verbosity >= Verbosity::WARN,
                           "No trigger mode setting, set it to kManual.");
  }

  if (!getParamOpt(node_, "world_frame_id", world_frame_id_)) {
    world_frame_id_ = "world";
    RCLCPP_WARN_EXPRESSION(node_->get_logger(),
                           global_verbosity >= Verbosity::WARN,
                           "No world_frame_id setting, set it to: %s.",
                           world_frame_id_.c_str());
  }

  RCLCPP_INFO_EXPRESSION(node_->get_logger(),
                         global_verbosity >= Verbosity::INFO, "Done.");
  return true;
}

void PlannerControlInterface::odometryCallback(
    const nav_msgs::msg::Odometry& odo) {
  current_pose_.position.x = odo.pose.pose.position.x;
  current_pose_.position.y = odo.pose.pose.position.y;
  current_pose_.position.z = odo.pose.pose.position.z;
  current_pose_.orientation.x = odo.pose.pose.orientation.x;
  current_pose_.orientation.y = odo.pose.pose.orientation.y;
  current_pose_.orientation.z = odo.pose.pose.orientation.z;
  current_pose_.orientation.w = odo.pose.pose.orientation.w;
  pci_manager_->setState(current_pose_);
  pci_manager_->setCurrentVelocity(odo.twist.twist.linear);
  if (!pose_is_ready_) {
    previous_pose_ = current_pose_;
  } else {
    Eigen::Vector3d prev_state(previous_pose_.position.x,
                               previous_pose_.position.y,
                               previous_pose_.position.z);
    Eigen::Vector3d curr_state(current_pose_.position.x,
                               current_pose_.position.y,
                               current_pose_.position.z);
    const double kMinDist = 5.0;
    if ((prev_state - curr_state).norm() > kMinDist) {
      if (menu_initialized) {
        geometry_msgs::msg::Pose new_pose;
        new_pose.position = current_pose_.position;
        new_pose.orientation.x = 0.0;
        new_pose.orientation.y = 0.0;
        new_pose.orientation.z = 0.0;
        new_pose.orientation.w = 1.0;
        semantic_server->setPose("semantic", new_pose);
        semantic_server->applyChanges();
        previous_pose_ = current_pose_;
      }
    }
  }
  pose_is_ready_ = true;
}

void PlannerControlInterface::poseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped& pose) {
  processPose(pose.pose.pose);
}

void PlannerControlInterface::poseStampedCallback(
    const geometry_msgs::msg::PoseStamped& pose) {
  processPose(pose.pose);
}

void PlannerControlInterface::processPose(
    const geometry_msgs::msg::Pose& pose) {
  current_pose_.position.x = pose.position.x;
  current_pose_.position.y = pose.position.y;
  current_pose_.position.z = pose.position.z;
  current_pose_.orientation.x = pose.orientation.x;
  current_pose_.orientation.y = pose.orientation.y;
  current_pose_.orientation.z = pose.orientation.z;
  current_pose_.orientation.w = pose.orientation.w;
  pci_manager_->setState(current_pose_);
  if (!pose_is_ready_) {
    previous_pose_ = current_pose_;
  } else {
    Eigen::Vector3d prev_state(previous_pose_.position.x,
                               previous_pose_.position.y,
                               previous_pose_.position.z);
    Eigen::Vector3d curr_state(current_pose_.position.x,
                               current_pose_.position.y,
                               current_pose_.position.z);
    const double kMinDist = 5.0;
    if ((prev_state - curr_state).norm() > kMinDist) {
      if (menu_initialized) {
        geometry_msgs::msg::Pose new_pose;
        new_pose.position = current_pose_.position;
        new_pose.orientation.x = 0.0;
        new_pose.orientation.y = 0.0;
        new_pose.orientation.z = 0.0;
        new_pose.orientation.w = 1.0;
        semantic_server->setPose("semantic", new_pose);
        semantic_server->applyChanges();
        previous_pose_ = current_pose_;
      }
    }
  }
  pose_is_ready_ = true;
}

void PlannerControlInterface::initIMarker() {
  {
    // create an interactive marker for our server
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = world_frame_id_;
    int_marker.header.stamp = node_->now();
    int_marker.name = source_marker_name;
    int_marker.description = "Source Pose";

    int_marker.pose.position.x = current_pose_.position.x;
    int_marker.pose.position.y = current_pose_.position.y;
    int_marker.pose.position.z = 1.5;

    // create a grey box marker
    visualization_msgs::msg::Marker box_marker;
    box_marker.type = visualization_msgs::msg::Marker::CUBE;
    box_marker.scale.x = 0.45;
    box_marker.scale.y = 0.45;
    box_marker.scale.z = 0.45;
    box_marker.color.r = 0.8;
    box_marker.color.g = 0.1;
    box_marker.color.b = 0.1;
    box_marker.color.a = 1.0;

    // create a non-interactive control which contains the box
    visualization_msgs::msg::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(box_marker);

    // add the control to the interactive marker
    int_marker.controls.push_back(box_control);

    // create a control which will move the box
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::msg::InteractiveMarkerControl control;

    tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "move_xy";
    control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(control);

    orien = tf2::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "move_z";
    control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    imarker_server_->insert(int_marker);
    imarker_server_->setCallback(
        int_marker.name,
        [this](visualization_msgs::msg::InteractiveMarkerFeedback::
                   ConstSharedPtr feedback) { processFeedback(feedback); });
  }
  {
    // create an interactive marker for our server
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = world_frame_id_;
    int_marker.header.stamp = node_->now();
    int_marker.name = target_marker_name;
    int_marker.description = "Target Pose";

    int_marker.pose.position.x = current_pose_.position.x + 1.0;
    int_marker.pose.position.y = current_pose_.position.y;
    int_marker.pose.position.z = 1.5;

    // create a grey box marker
    visualization_msgs::msg::Marker box_marker;
    box_marker.type = visualization_msgs::msg::Marker::CUBE;
    box_marker.scale.x = 0.45;
    box_marker.scale.y = 0.45;
    box_marker.scale.z = 0.45;
    box_marker.color.r = 0.1;
    box_marker.color.g = 0.8;
    box_marker.color.b = 0.1;
    box_marker.color.a = 1.0;

    // create a non-interactive control which contains the box
    visualization_msgs::msg::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(box_marker);

    // add the control to the interactive marker
    int_marker.controls.push_back(box_control);

    // create a control which will move the box
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::msg::InteractiveMarkerControl control;

    tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "move_xy";
    control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(control);

    orien = tf2::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "move_z";
    control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    imarker_server_->insert(int_marker);
    imarker_server_->setCallback(
        int_marker.name,
        [this](visualization_msgs::msg::InteractiveMarkerFeedback::
                   ConstSharedPtr feedback) { processFeedback(feedback); });
  }
  // 'commit' changes and send to all clients
  imarker_server_->applyChanges();
}

void PlannerControlInterface::processFeedback(
    visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
        feedback) {
  // The reader in runSearch copies these field by field; without the lock it can
  // observe a position from before the update and an orientation from after.
  std::lock_guard<std::mutex> lock(setpoint_mutex_);
  if (!feedback->marker_name.compare(source_marker_name)) {
    // update source wp.
    source_setpoint_.position.x = feedback->pose.position.x;
    source_setpoint_.position.y = feedback->pose.position.y;
    source_setpoint_.position.z = feedback->pose.position.z;
    source_setpoint_.orientation.x = feedback->pose.orientation.x;
    source_setpoint_.orientation.y = feedback->pose.orientation.y;
    source_setpoint_.orientation.z = feedback->pose.orientation.z;
    source_setpoint_.orientation.w = feedback->pose.orientation.w;
  } else if (!feedback->marker_name.compare(target_marker_name)) {
    // update target wp.
    target_setpoint_.position.x = feedback->pose.position.x;
    target_setpoint_.position.y = feedback->pose.position.y;
    target_setpoint_.position.z = feedback->pose.position.z;
    target_setpoint_.orientation.x = feedback->pose.orientation.x;
    target_setpoint_.orientation.y = feedback->pose.orientation.y;
    target_setpoint_.orientation.z = feedback->pose.orientation.z;
    target_setpoint_.orientation.w = feedback->pose.orientation.w;
  }
}

// Semantics
void PlannerControlInterface::initSemanticIMarker() {
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = world_frame_id_;
  int_marker.header.stamp = node_->now();
  int_marker.name = "semantic";
  int_marker.description = "";
  int_marker.pose.position.x = current_pose_.position.x + 1.5;
  int_marker.pose.position.y = current_pose_.position.y;
  int_marker.pose.position.z = current_pose_.position.z;
  int_marker.scale = control_size;

  visualization_msgs::msg::InteractiveMarkerControl x_control;
  x_control.name = "x_control";
  x_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(x_control);

  visualization_msgs::msg::InteractiveMarkerControl y_control;
  y_control.name = "y_control";
  y_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  y_control.orientation.x = 0;
  y_control.orientation.y = 0;
  y_control.orientation.z = 0.707;
  y_control.orientation.w = 0.707;
  int_marker.controls.push_back(y_control);

  visualization_msgs::msg::InteractiveMarkerControl z_control;
  z_control.name = "z_control";
  z_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  z_control.orientation.x = 0;
  z_control.orientation.y = 0.707;
  z_control.orientation.z = 0;
  z_control.orientation.w = 0.707;
  int_marker.controls.push_back(z_control);

  visualization_msgs::msg::Marker button_box_marker;
  button_box_marker.type = visualization_msgs::msg::Marker::CUBE;
  button_box_marker.scale.x = 1.0;
  button_box_marker.scale.y = 1.0;
  button_box_marker.scale.z = 1.0;
  button_box_marker.color.r = 0.5;
  button_box_marker.color.g = 0.5;
  button_box_marker.color.b = 0.5;
  button_box_marker.color.a = 0.65;

  visualization_msgs::msg::InteractiveMarkerControl button_control;
  button_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  button_control.name = "button_control";
  button_control.description = "menu_button";
  button_control.markers.push_back(button_box_marker);
  button_control.always_visible = true;

  int_marker.controls.push_back(button_control);

  semantic_server->insert(int_marker);
  semantic_server->setCallback(
      int_marker.name,
      [this](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
                 feedback) { semanticMarkerFeedback(feedback); });

  if (!menu_initialized) {
    class_entry_handle = menu_handler.insert("Class");
    accept_entry_handle = menu_handler.insert(
        "Accept",
        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::
                   ConstSharedPtr& feedback) { acceptButtonFeedback(feedback); });

    sub_class_entry_handle = menu_handler.insert(
        class_entry_handle, kStaircaseStr,
        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::
                   ConstSharedPtr& feedback) {
          selectSemanticsFeedback(feedback);
        });
    menu_handler.setCheckState(sub_class_entry_handle,
                               interactive_markers::MenuHandler::UNCHECKED);
    sub_class_entry_handle = menu_handler.insert(
        class_entry_handle, kDoorStr,
        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::
                   ConstSharedPtr& feedback) {
          selectSemanticsFeedback(feedback);
        });
    menu_handler.setCheckState(sub_class_entry_handle,
                               interactive_markers::MenuHandler::UNCHECKED);

    menu_initialized = true;
  }
  menu_handler.apply(*semantic_server, "semantic");

  semantic_server->applyChanges();
}

void PlannerControlInterface::semanticMarkerFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr&
        feedback) {
  (void)feedback;
  visualization_msgs::msg::InteractiveMarker marker;
  semantic_server->get("semantic", marker);

  semantic_position.x = marker.pose.position.x;
  semantic_position.y = marker.pose.position.y;
  semantic_position.z = marker.pose.position.z;
}

void PlannerControlInterface::acceptButtonFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr&
        feedback) {
  (void)feedback;
  visualization_msgs::msg::InteractiveMarker marker;
  semantic_server->get("semantic", marker);

  semantic_position.x = marker.pose.position.x;
  semantic_position.y = marker.pose.position.y;
  semantic_position.z = marker.pose.position.z;

  semantic_location.point = semantic_position;
  semantic_location.type.value = current_semantic_class_.value;

  semantic_pub->publish(semantic_location);
}

void PlannerControlInterface::selectSemanticsFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr&
        feedback) {
  visualization_msgs::msg::InteractiveMarker marker;
  semantic_server->get("semantic", marker);

  menu_handler.setCheckState(sub_class_entry_handle,
                             interactive_markers::MenuHandler::UNCHECKED);
  sub_class_entry_handle = feedback->menu_entry_id;
  menu_handler.setCheckState(sub_class_entry_handle,
                             interactive_markers::MenuHandler::CHECKED);
  menu_handler.reApply(*semantic_server);
  semantic_server->applyChanges();
  std::string semantic_class;
  menu_handler.getTitle(sub_class_entry_handle, semantic_class);
  if (!semantic_class.compare(kStaircaseStr))
    current_semantic_class_.value =
        planner_semantic_msgs::msg::SemanticClass::STAIRCASE;
  else if (!semantic_class.compare(kDoorStr))
    current_semantic_class_.value =
        planner_semantic_msgs::msg::SemanticClass::DOOR;
  else
    current_semantic_class_.value =
        planner_semantic_msgs::msg::SemanticClass::NONE;
}

}  // namespace explorer
