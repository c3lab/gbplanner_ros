#include "gbplanner/gbplanner.h"

#include <nav_msgs/msg/path.hpp>

// namespace explorer {

Gbplanner::Gbplanner(rclcpp::Node* node) : node_(node) {
  planner_status_ = Gbplanner::PlannerStatus::NOT_READY;

  rrg_ = new Rrg(node_);
  if (!(rrg_->loadParams(false))) {
    RCLCPP_ERROR_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::ERROR, "Could not load all required parameters. Shutdown ROS node.");
    rclcpp::shutdown();
  }

  initializeAttributes();
}

Gbplanner::Gbplanner(rclcpp::Node* node, MapManager* map_manager)
    : node_(node) {
  
  planner_status_ = Gbplanner::PlannerStatus::NOT_READY;
  rrg_ = new Rrg(node_, map_manager);

  if (!(rrg_->loadParams(true))) {
    RCLCPP_ERROR_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::ERROR, "Could not load all required parameters. Shutdown ROS node.");
    rclcpp::shutdown();
  }

  initializeAttributes();
}

void Gbplanner::initializeAttributes() {
  using std::placeholders::_1;
  using std::placeholders::_2;

  planner_service_ = node_->create_service<planner_msgs::srv::PlannerSrv>(
      "gbplanner",
      [this](const std::shared_ptr<planner_msgs::srv::PlannerSrv::Request> req,
             std::shared_ptr<planner_msgs::srv::PlannerSrv::Response> res) {
        plannerServiceCallback(*req, *res);
      });
  global_planner_service_ =
      node_->create_service<planner_msgs::srv::PlannerGlobal>(
          "gbplanner/global",
          std::bind(&Gbplanner::globalPlannerServiceCallback, this, _1, _2));
  planner_homing_service_ =
      node_->create_service<planner_msgs::srv::PlannerHoming>(
          "gbplanner/homing",
          std::bind(&Gbplanner::homingServiceCallback, this, _1, _2));
  planner_set_homing_pos_service_ =
      node_->create_service<planner_msgs::srv::PlannerSetHomingPos>(
          "gbplanner/set_homing_pos",
          std::bind(&Gbplanner::setHomingPosServiceCallback, this, _1, _2));
  planner_search_service_ =
      node_->create_service<planner_msgs::srv::PlannerSearch>(
          "gbplanner/search",
          std::bind(&Gbplanner::plannerSearchServiceCallback, this, _1, _2));
  planner_passing_gate_service_ =
      node_->create_service<planner_msgs::srv::PlannerRequestPath>(
          "gbplanner/passing_gate",
          std::bind(&Gbplanner::passingGateCallback, this, _1, _2));
  planner_set_global_bound_service_ =
      node_->create_service<planner_msgs::srv::PlannerSetGlobalBound>(
          "gbplanner/set_global_bound",
          std::bind(&Gbplanner::setGlobalBound, this, _1, _2));
  planner_set_dynamic_global_bound_service_ =
      node_->create_service<planner_msgs::srv::PlannerDynamicGlobalBound>(
          "gbplanner/set_dynamic_global_bound",
          std::bind(&Gbplanner::setDynamicGlobalBound, this, _1, _2));
  planner_clear_untraversable_zones_service_ =
      node_->create_service<std_srvs::srv::Trigger>(
          "gbplanner/clear_untraversable_zones",
          std::bind(&Gbplanner::clearUntraversableZones, this, _1, _2));
  planner_load_graph_service_ =
      node_->create_service<planner_msgs::srv::PlannerStringTrigger>(
          "gbplanner/load_graph",
          std::bind(&Gbplanner::plannerLoadGraphCallback, this, _1, _2));
  planner_save_graph_service_ =
      node_->create_service<planner_msgs::srv::PlannerStringTrigger>(
          "gbplanner/save_graph",
          std::bind(&Gbplanner::plannerSaveGraphCallback, this, _1, _2));
  planner_goto_wp_service_ =
      node_->create_service<planner_msgs::srv::PlannerGoToWaypoint>(
          "gbplanner/go_to_waypoint",
          std::bind(&Gbplanner::plannerGotoWaypointCallback, this, _1, _2));
  planner_get_frontiers_service_ =
      node_->create_service<planner_msgs::srv::PlannerGetFrontiers>(
          "gbplanner/get_frontiers",
          std::bind(&Gbplanner::plannerGetFrontiersCallback, this, _1, _2));
  planner_get_target_costs_service_ =
      node_->create_service<planner_msgs::srv::PlannerGetTargetCosts>(
          "gbplanner/get_target_costs",
          std::bind(&Gbplanner::plannerGetTargetCostsCallback, this, _1, _2));
  planner_validate_frontiers_service_ =
      node_->create_service<planner_msgs::srv::PlannerValidateFrontiers>(
          "gbplanner/validate_frontiers",
          std::bind(&Gbplanner::plannerValidateFrontiersCallback, this, _1, _2));
  planner_enable_untraversable_polygon_subscriber_service_ =
      node_->create_service<std_srvs::srv::SetBool>(
          "gbplanner/enable_untraversable_polygon_subscriber",
          std::bind(
              &Gbplanner::plannerEnableUntraversablePolygonSubscriberCallback,
              this, _1, _2));
  planner_set_planning_trigger_mode_service_ =
      node_->create_service<planner_msgs::srv::PlannerSetPlanningMode>(
          "gbplanner/set_planning_trigger_mode",
          std::bind(&Gbplanner::plannerSetPlanningTriggerModeCallback, this, _1,
                    _2));

  inspection_path_service_ =
      node_->create_service<planner_msgs::srv::PlannerSrv>(
          "gbplanner/get_inspection_path",
          std::bind(&Gbplanner::inspectionServiceCallback, this, _1, _2));

  force_compartment_transition_service_ =
      node_->create_service<std_srvs::srv::Trigger>(
          "gbplanner/force_compartment_transition",
          std::bind(&Gbplanner::forceCompartmentChangeServiceCallback, this, _1,
                    _2));

  switch_operation_mode_service_ = node_->create_service<std_srvs::srv::SetBool>(
      "gbplanner/switch_operation_mode",
      std::bind(&Gbplanner::switchOperationModeServiceCallback, this, _1, _2));

  pose_subscriber_ =
      node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "pose", 100,
          [this](const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
            poseCallback(msg);
          });
  pose_stamped_subscriber_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "pose_stamped", 100,
          [this](const geometry_msgs::msg::PoseStamped& msg) {
            poseStampedCallback(msg);
          });
  odometry_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 100,
      [this](const nav_msgs::msg::Odometry& msg) { odometryCallback(msg); });
  robot_status_subcriber_ =
      node_->create_subscription<planner_msgs::msg::RobotStatus>(
          "/robot_status", 1, [this](const planner_msgs::msg::RobotStatus& msg) {
            robotStatusCallback(msg);
          });
  subscribeUntraversablePolygon();

  local_nav_goal_subscriber_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "local_navigation_goal", 100,
          [this](const geometry_msgs::msg::PoseStamped& msg) {
            localNavGoalCallback(msg);
          });

  stop_srv_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
      "planner_control_interface/stop_request", 5,
      [this](const std_msgs::msg::Bool& msg) { stopMsgCallback(msg); });

  global_planner_local_goal_pub_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(
          "gbplanner/homing_local_goal", 10);

  planning_params_.loadParams(node_, "PlanningParams");

  RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::DEBUG, "Compartment Centers:");
  if(global_verbosity >= Verbosity::DEBUG) {
    for(auto c : planning_params_.compartment_centers) {
      std::cout << c.transpose() << std::endl;
    }
  }
  
  planner_mode_ = PlannerMode::kExploration;
  

  BoundedSpaceParams rrg_global_bounds;
  rrg_->getGlobalBoundParams(rrg_global_bounds);
  rrg_->setExplorationAndInspectionBounds(rrg_global_bounds, rrg_global_bounds);
}

void Gbplanner::inspectionServiceCallback(
    const std::shared_ptr<planner_msgs::srv::PlannerSrv::Request> req,
    std::shared_ptr<planner_msgs::srv::PlannerSrv::Response> res) {
  //
  (void)req;
  // res->path = rrg_->getInspectionPath();
  if(planning_params_.basic_inspection_viewpoints)
    res->path = rrg_->getInspectionPathBasic();
  else
    res->path = rrg_->getInspectionPath();
}

void Gbplanner::forceCompartmentChangeServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  //
  (void)req;
  (void)res;
  planner_mode_ = PlannerMode::kCompartmentChange;
}

void Gbplanner::plannerGotoWaypointCallback(
    const std::shared_ptr<planner_msgs::srv::PlannerGoToWaypoint::Request> req,
    std::shared_ptr<planner_msgs::srv::PlannerGoToWaypoint::Response> res) {
  res->path.clear();
  res->path = rrg_->getGlobalPath(req->waypoint);
}

void Gbplanner::plannerGetFrontiersCallback(
    const std::shared_ptr<planner_msgs::srv::PlannerGetFrontiers::Request> req,
    std::shared_ptr<planner_msgs::srv::PlannerGetFrontiers::Response> res) {
  res->frontiers.clear();
  if (req->force_update) rrg_->updateFrontiers();
  rrg_->getFrontiers(req->skip_gain_refresh, res->frontiers);
  res->header.stamp = node_->now();
  res->header.frame_id = planning_params_.global_frame_id;
  res->success = !res->frontiers.empty();
}

void Gbplanner::plannerGetTargetCostsCallback(
    const std::shared_ptr<planner_msgs::srv::PlannerGetTargetCosts::Request> req,
    std::shared_ptr<planner_msgs::srv::PlannerGetTargetCosts::Response> res) {
  res->costs.clear();
  rrg_->getTargetCosts(req->targets, res->costs);
  res->header.stamp = node_->now();
  res->header.frame_id = planning_params_.global_frame_id;
  // Costing a target the robot cannot reach is a valid answer, so success
  // tracks whether every request target got an entry, not whether they are
  // all reachable.
  res->success = (res->costs.size() == req->targets.size());
}

void Gbplanner::plannerValidateFrontiersCallback(
    const std::shared_ptr<planner_msgs::srv::PlannerValidateFrontiers::Request> req,
    std::shared_ptr<planner_msgs::srv::PlannerValidateFrontiers::Response> res) {
  res->is_unknown.clear();
  res->gains.clear();
  for (const auto& target : req->positions) {
    Eigen::Vector3d pos(target.x, target.y, target.z);
    // In GBPlanner, a frontier is a viewpoint (in kFree space) that sees enough kUnknown space.
    // So we must use raycasting to check if it still sees kUnknown space.
    auto vgain = rrg_->isStillFrontier(pos);
    res->is_unknown.push_back(vgain.is_frontier);
    res->gains.push_back(vgain.gain);
  }
  res->success = true;
}

void Gbplanner::subscribeUntraversablePolygon() {
  untraversable_polygon_subscriber_ =
      node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
          "/traversability_estimation/untraversable_polygon", 100,
          [this](const geometry_msgs::msg::PolygonStamped& msg) {
            untraversablePolygonCallback(msg);
          });
}

void Gbplanner::plannerEnableUntraversablePolygonSubscriberCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  if (static_cast<bool>(request->data)) {
    RCLCPP_INFO_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::INFO, "Gbplanner checks traversability");
    subscribeUntraversablePolygon();
  } else {
    RCLCPP_INFO_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::INFO, "Gbplanner stops checking traversability");
    // Dropping the handle is the rclcpp equivalent of Subscriber::shutdown().
    untraversable_polygon_subscriber_.reset();
  }
  response->success = static_cast<unsigned char>(true);
}

void Gbplanner::plannerLoadGraphCallback(
    const std::shared_ptr<planner_msgs::srv::PlannerStringTrigger::Request> req,
    std::shared_ptr<planner_msgs::srv::PlannerStringTrigger::Response> res) {
  res->success = rrg_->loadGraph(req->message);
}

void Gbplanner::plannerSaveGraphCallback(
    const std::shared_ptr<planner_msgs::srv::PlannerStringTrigger::Request> req,
    std::shared_ptr<planner_msgs::srv::PlannerStringTrigger::Response> res) {
  res->success = rrg_->saveGraph(req->message);
}

void Gbplanner::setGlobalBound(
    const std::shared_ptr<planner_msgs::srv::PlannerSetGlobalBound::Request> req,
    std::shared_ptr<planner_msgs::srv::PlannerSetGlobalBound::Response> res) {
  if (!req->get_current_bound)
    res->success = rrg_->setGlobalBound(req->bound, req->reset_to_default);
  else
    res->success = true;

  rrg_->getGlobalBound(res->bound_ret);
}

void Gbplanner::setDynamicGlobalBound(
    const std::shared_ptr<planner_msgs::srv::PlannerDynamicGlobalBound::Request> req,
    std::shared_ptr<planner_msgs::srv::PlannerDynamicGlobalBound::Response> res) {
  RCLCPP_INFO_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::INFO, "Calling RRG set dynamic global bound");
  res->success = rrg_->setGlobalBound(*req);
  RCLCPP_INFO_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::INFO, "RRG set dynamic global bound returned");
}

void Gbplanner::setGeofenceManager(
    std::shared_ptr<GeofenceManager> geofence_manager) {
  rrg_->setGeofenceManager(geofence_manager);
}

void Gbplanner::setSharedParams(const RobotParams& robot_params,
                                const BoundedSpaceParams& global_space_params) {
  rrg_->setSharedParams(robot_params, global_space_params);
}

void Gbplanner::setSharedParams(const RobotParams& robot_params,
                                const BoundedSpaceParams& global_space_params,
                                const BoundedSpaceParams& local_space_params) {
  rrg_->setSharedParams(robot_params, global_space_params, local_space_params);
}

void Gbplanner::passingGateCallback(
    const std::shared_ptr<planner_msgs::srv::PlannerRequestPath::Request> req,
    std::shared_ptr<planner_msgs::srv::PlannerRequestPath::Response> res) {
  (void)req;
  res->path = rrg_->searchPathToPassGate();
  res->bound.mode = planner_msgs::msg::BoundMode::EXTENDED_BOUND;
}

void Gbplanner::switchOperationModeServiceCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  bt_states_.operation_mode = static_cast<int>(req->data);
  RCLCPP_WARN(node_->get_logger(), "Switching operation mode to: %d", bt_states_.operation_mode);
  res->success = true;
}

bool Gbplanner::plannerServiceCallback(
    planner_msgs::srv::PlannerSrv::Request& req,
    planner_msgs::srv::PlannerSrv::Response& res) {

  rrg_->reset();
  
  if(planning_params_.exploration_only) {
    if(planning_params_.enable_opening_traversal) {
      if(exploration_counter_ >= planning_params_.max_exploration_iterations) {
        opening_traversal_requested_ = true;
        exploration_counter_ = 0;
        RCLCPP_WARN(node_->get_logger(), "Exploration Iterations Complete, switching to: %d", (int)planner_mode_);
      }
      return getExplorationPath(req, res);
    }
    else {
      return getExplorationPath(req, res);
    }
  }

  RCLCPP_WARN(node_->get_logger(), "Current Planner Mode: %d", (int)planner_mode_);

  bool success = true;
  std::vector<geometry_msgs::msg::Pose> empty_path;
  switch (planner_mode_) {
    case PlannerMode::kExploration:
    {
      if(exploration_counter_ >= planning_params_.max_exploration_iterations) {
        res.path = empty_path;
        res.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
        planner_mode_ = PlannerMode::kInspection;
        success = true;
        exploration_counter_ = 0;
        RCLCPP_WARN(node_->get_logger(), "Exploration Iterations Complete, switching to: %d", (int)planner_mode_);
        break;
      }
      success = getExplorationPath(req, res);
      if(success)
      {
        ++exploration_counter_;
        RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::DEBUG, "Exploration Counter: %d", exploration_counter_);
        // If exploration is completed
        if(res.status != planner_msgs::srv::PlannerSrv::Response::FORWARD) {
          res.path = empty_path;
          res.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
          planner_mode_ = PlannerMode::kInspection;
          success = true;
          exploration_counter_ = 0;
          RCLCPP_WARN(node_->get_logger(), "Exploration Complete, switching to: %d", (int)planner_mode_);
        }
      }
      else
      {
        RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "Exploration Failed trying again:");
      }
      
      break;
    }

    case PlannerMode::kExplorationComplete:
    {
      res.path = empty_path;
      res.status = planner_msgs::srv::PlannerSrv::Response::MANUAL_CUSTOM_PATH;
      planner_mode_ = PlannerMode::kInspection;
      RCLCPP_WARN(node_->get_logger(), "Switching to: %d", (int)planner_mode_);
      break;
    }

    case PlannerMode::kInspection:
    {
      success = getInspectionPath(req, res);
      if(!success) {
        res.path = empty_path;
        res.status = planner_msgs::srv::PlannerSrv::Response::AUTO_CUSTOM_PATH;
        RCLCPP_WARN(node_->get_logger(), "Inspection Failed");
      }
      else {
        res.status = planner_msgs::srv::PlannerSrv::Response::AUTO_CUSTOM_PATH;
        planner_mode_ = PlannerMode::kCompartmentChange;
        RCLCPP_WARN(node_->get_logger(), "Inspection Successful. Switching to: %d", (int)planner_mode_);
      }
      break;
    }

    case PlannerMode::kCompartmentChange:
    {

      if(compartment_counter_ >= planning_params_.compartment_centers.size()-1) {
        res.status = planner_msgs::srv::PlannerSrv::Response::MANUAL_CUSTOM_PATH;
        // res.path = empty_path;
        success = true;
        compartment_change_tries_ = 0;
        RCLCPP_WARN(node_->get_logger(), "All compartments explored");
        if(planning_params_.auto_homing_enable)
          res.path = rrg_->getHomingPath("world");
        else
          res.path = empty_path;
        break;
      }

      bool search_success = getCompartmentTransitionPath(req, res);
      
      if(search_success) {
        compartment_change_tries_ = 0;
        success = true;
      }
      else {
        success = true;
        ++compartment_change_tries_;
        RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity>=Verbosity::WARN, "Path search failed. Try %d", compartment_change_tries_);
        if(compartment_change_tries_ <= max_compartment_change_tries_) {
          --compartment_counter_;  // To make sure that we try to replan
        }
      }
      rrg_->reset();
      
      break;
    }
    
    default:
      success = getExplorationPath(req, res);
      break;
  }

  return success;
}

Rrg::GlobalPlannerStatus Gbplanner::getGlobalExplorationPath()
{
  rrg_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));

  int status;
  out_srv_res_.path = rrg_->runGlobalPlanner(0, false, false, status);
  out_srv_res_.status = status;
  if(status == planner_msgs::srv::PlannerSrv::Response::HOMING)
  {
    bt_states_.global_exp_exhausted = true;
    return Rrg::GlobalPlannerStatus::G_HOMING;
  }

  if(out_srv_res_.path.empty())
  {
    return Rrg::GlobalPlannerStatus::G_ERR;
  }
  else
  {
    if(status == planner_msgs::srv::PlannerSrv::Response::HOMING)
    {
      return Rrg::GlobalPlannerStatus::G_HOMING;
    }
  }
  return Rrg::GlobalPlannerStatus::G_OK;
}

bool Gbplanner::calculateGlobalPath()
{
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    // out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
    return false;
  }

  rrg_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));
  active_global_path_ = rrg_->calculateGlobalPath();
  if(active_global_path_.empty())
  {
    // out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
    return false;  
  }

  rrg_->setLocalNavGoal(Eigen::Vector3d(active_global_path_.back().position.x,
                              active_global_path_.back().position.y,
                              active_global_path_.back().position.z));

  // out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::HOMING;
  return true;
}

bool Gbplanner::updateGlobalGoal()
{
  if(active_global_path_.empty())
  {
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "No active global path to update.");
    return false;  
  }

  Eigen::Vector3d current_position(current_state_[0], current_state_[1], current_state_[2]);
  Eigen::Vector3d global_goal(active_global_path_.back().position.x,
                              active_global_path_.back().position.y,
                              active_global_path_.back().position.z);
  for(size_t i = 0; i < active_global_path_.size()-1; ++i)
  {
    Eigen::Vector3d waypoint(active_global_path_[i].position.x,
                             active_global_path_[i].position.y,
                             active_global_path_[i].position.z);
    double distance = (waypoint - current_position).norm();
    if(distance < planning_params_.active_homing_update_radius)
    {
      // Remove this waypoint
      if(active_global_path_.size() > 1)
      {
        active_global_path_.erase(active_global_path_.begin() + i);
        --i; // Adjust index after erasure
      }
      else
      {
        break;
      }
    }
    else 
    {
      // Since waypoints are ordered, we can break early
      global_goal = waypoint;
      break;
    }
  }

  if(active_global_path_.empty())
  {
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "Global path completed.");
    return false;  
  }
  else
  {
    rrg_->setLocalNavGoal(global_goal);
    // visualize global goal
    geometry_msgs::msg::PoseStamped global_goal_msg;
    global_goal_msg.header.frame_id = planning_params_.global_frame_id;
    global_goal_msg.header.stamp = node_->now();
    global_goal_msg.pose.position.x = global_goal[0];
    global_goal_msg.pose.position.y = global_goal[1];
    global_goal_msg.pose.position.z = global_goal[2];
    tf2::Quaternion goal_quat;
    goal_quat.setRPY(0.0, 0.0, 0.0);
    global_goal_msg.pose.orientation = tf2::toMsg(goal_quat);
    global_planner_local_goal_pub_->publish(global_goal_msg);
    return true;
  }
}

bool Gbplanner::checkGlobalExplorationStatus()
{
  int status;
  std::vector<geometry_msgs::msg::Pose> path = rrg_->runGlobalPlanner(0, false, false, status);
  if(status == planner_msgs::srv::PlannerSrv::Response::HOMING)
  {
    bt_states_.global_exp_exhausted = true;
    return true;
  }

  return false;
}

Rrg::LocalPlannerStatus Gbplanner::getLocalNavigationPath()
{
  // Extract setting from the request.
  // RCLCPP_INFO_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::INFO, "[GBPlanner]: Planner service called");
  rrg_->setGlobalFrame(in_srv_req_.header.frame_id);
  // RCLCPP_INFO_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::INFO, "[GBPlanner]: global frame set");
  rrg_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));
  // RCLCPP_INFO_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::INFO, "[GBPlanner]: bound mode set");
  rrg_->setRootStateForPlanning(in_srv_req_.root_pose);
  // RCLCPP_INFO_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::INFO, "[GBPlanner]: root state set");
  RCLCPP_INFO_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::DEBUG, "Root state: %f %f %f, %f", in_srv_req_.root_pose.position.x, in_srv_req_.root_pose.position.y, in_srv_req_.root_pose.position.z, tf2::getYaw(in_srv_req_.root_pose.orientation));

  Rrg::GraphStatus status;
  Rrg::LocalPlannerStatus ret_status;

  // Start the planner.
  out_srv_res_.path.clear();
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    status = Rrg::GraphStatus::NOT_OK;
    out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
    return Rrg::LocalPlannerStatus::L_ERR;
  }

  rrg_->reset();

  if (planning_params_.graph_building_mode == GraphBuildingModeType::kBasic) {
    status = rrg_->buildGraph();
  } else if (planning_params_.graph_building_mode == GraphBuildingModeType::kBatch) {
    status = rrg_->batchGraph();
  }

  switch (status) {
    case Rrg::GraphStatus::OK:
      ret_status = Rrg::LocalPlannerStatus::L_OK;
      break;
    case Rrg::GraphStatus::ERR_KDTREE:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] An issue occurred with kdtree data.");
      ret_status = Rrg::LocalPlannerStatus::L_ERR;
      break;
    case Rrg::GraphStatus::ERR_NO_FEASIBLE_PATH:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] No feasible path was found.");
      ret_status = Rrg::LocalPlannerStatus::L_ERR;
      break;
    case Rrg::GraphStatus::NOT_OK:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] Graph building: Not ok");
      ret_status = Rrg::LocalPlannerStatus::L_ERR;
      break;
    default:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] Error occurred in building graph.");
      ret_status = Rrg::LocalPlannerStatus::L_ERR;
      break;
  }

  if(status != Rrg::GraphStatus::OK) 
  {
    out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
    return ret_status;
  }

  Rrg::LocalPlannerStatus lp_status = rrg_->evaluateLocalNavigationPath();
  switch (lp_status) {
    case Rrg::LocalPlannerStatus::L_OK:
      ret_status = Rrg::LocalPlannerStatus::L_OK;
      out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
      break;
    case Rrg::LocalPlannerStatus::L_EXHAUSTED:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::PLANNER_STATUS, "[GBPLANNER] Reached local navigation goal");
      ret_status = Rrg::LocalPlannerStatus::L_EXHAUSTED;
      out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::MANUAL_CUSTOM_PATH;
      break;
    case Rrg::LocalPlannerStatus::L_ERR:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] Error occurred in local navigation.");
      ret_status = Rrg::LocalPlannerStatus::L_ERR;
      out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
      break;
    case Rrg::LocalPlannerStatus::L_STUCK:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] Local navigation stuck.");
      ret_status = Rrg::LocalPlannerStatus::L_STUCK;
      out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::MANUAL_CUSTOM_PATH;
      break;
    case Rrg::LocalPlannerStatus::L_TIME_LIMIT_REACHED:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] Homing needed.");
      ret_status = Rrg::LocalPlannerStatus::L_TIME_LIMIT_REACHED;
      out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::HOMING;
      break;
    default:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] Error occurred in local navigation.");
      ret_status = Rrg::LocalPlannerStatus::L_ERR;
      out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
      break;
  }
  if(lp_status != Rrg::GraphStatus::OK) 
  { 
    return ret_status;
  }
  else {
    out_srv_res_.path = rrg_->getBestPathSimplified();
    out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::DEBUG, "[GBPLANNER] Regular Planning");
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::DEBUG, "[GBPLANNER] Path status: %d", out_srv_res_.status);
  }
  return Rrg::LocalPlannerStatus::L_OK;
}

Rrg::LocalPlannerStatus Gbplanner::getExplorationPath()
{
  // Extract setting from the request.
  rrg_->setGlobalFrame(in_srv_req_.header.frame_id);
  rrg_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));
  rrg_->setRootStateForPlanning(in_srv_req_.root_pose);
  RCLCPP_INFO_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::DEBUG, "Root state: %f %f %f, %f", in_srv_req_.root_pose.position.x, in_srv_req_.root_pose.position.y, in_srv_req_.root_pose.position.z, tf2::getYaw(in_srv_req_.root_pose.orientation));

  Rrg::GraphStatus status;
  Rrg::LocalPlannerStatus ret_status;

  // Start the planner.
  out_srv_res_.path.clear();
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    status = Rrg::GraphStatus::NOT_OK;
    out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
    return Rrg::LocalPlannerStatus::L_ERR;
  }

  rrg_->reset();

  if (planning_params_.graph_building_mode == GraphBuildingModeType::kBasic) {
    status = rrg_->buildGraph();
  } else if (planning_params_.graph_building_mode == GraphBuildingModeType::kBatch) {
    status = rrg_->batchGraph();
  }

  switch (status) {
    case Rrg::GraphStatus::OK:
      ret_status = Rrg::LocalPlannerStatus::L_OK;
      break;
    case Rrg::GraphStatus::ERR_KDTREE:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] An issue occurred with kdtree data.");
      ret_status = Rrg::LocalPlannerStatus::L_ERR;
      break;
    case Rrg::GraphStatus::ERR_NO_FEASIBLE_PATH:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] No feasible path was found.");
      ret_status = Rrg::LocalPlannerStatus::L_ERR;
      break;
    case Rrg::GraphStatus::NOT_OK:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] Graph building: Not ok");
      ret_status = Rrg::LocalPlannerStatus::L_ERR;
      break;
    default:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] Error occurred in building graph.");
      ret_status = Rrg::LocalPlannerStatus::L_ERR;
      break;
  }

  if(status != Rrg::GraphStatus::OK) 
  {
    out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
    return ret_status;
  }
  
  status = rrg_->evaluateGraph();
  switch (status) {
    case Rrg::GraphStatus::OK:
      ret_status = Rrg::LocalPlannerStatus::L_OK;
      break;
    case Rrg::GraphStatus::NO_GAIN:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] No positive gain was found.");
      ret_status = Rrg::LocalPlannerStatus::L_OK;
      break;
    case Rrg::GraphStatus::CONSEC_LOW_GAIN:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::PLANNER_STATUS, "[GBPLANNER] Very low local gain. Triggering global planner");
      ret_status = Rrg::LocalPlannerStatus::L_EXHAUSTED;
      break;
    case Rrg::GraphStatus::NOT_OK:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::PLANNER_STATUS, "[PLANNER_ERROR] Error occurred in gain calculation.");
      ret_status = Rrg::LocalPlannerStatus::L_ERR;
      break;
    default:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] Error occurred in gain calculation.");
      ret_status = Rrg::LocalPlannerStatus::L_ERR;
      break;
  }

  if(status != Rrg::GraphStatus::OK) 
  {
    out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
    return ret_status;
  }

  if (status == Rrg::GraphStatus::OK) {
    out_srv_res_.path = rrg_->getBestPathSimplified();
    out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::DEBUG, "[GBPLANNER] Regular Planning");
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::DEBUG, "[GBPLANNER] Path status: %d", out_srv_res_.status);
  }
  return Rrg::LocalPlannerStatus::L_OK;
}

bool Gbplanner::transitionCompartment()
{
  rrg_->setNextCompartmentCenter(planning_params_.compartment_centers[compartment_counter_+1]);
  rrg_->setNextCompartmentIndex(compartment_counter_+1);
  BoundedSpaceParams translated_bound = planning_params_.compartment_dimensions;
  Eigen::Vector3d max_val = planning_params_.compartment_dimensions.max_val + planning_params_.compartment_centers[compartment_counter_];
  Eigen::Vector3d max_extension = planning_params_.compartment_dimensions.max_extension + planning_params_.compartment_centers[compartment_counter_];
  Eigen::Vector3d min_val = planning_params_.compartment_dimensions.min_val + planning_params_.compartment_centers[compartment_counter_];
  Eigen::Vector3d min_extension = planning_params_.compartment_dimensions.min_extension + planning_params_.compartment_centers[compartment_counter_];
  translated_bound.setBound(min_val, max_val);
  RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::DEBUG, "Compartment counter: %d", compartment_counter_);
  rrg_->setExplorationAndInspectionBounds(translated_bound, translated_bound);
  ++compartment_counter_;
  return true;
}

bool Gbplanner::allCompartmentsInspected()
{
  if(compartment_counter_ > planning_params_.compartment_centers.size()-1) 
  {
    return true;
  }
  else 
  {
    return false;
  }
}

bool Gbplanner::getExplorationPath(planner_msgs::srv::PlannerSrv::Request& req,
      planner_msgs::srv::PlannerSrv::Response& res) {
  //
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;
  // Extract setting from the request.
  rrg_->setGlobalFrame(req.header.frame_id);
  t2 = std::chrono::high_resolution_clock::now();
  double reset_time = std::chrono::duration<double, std::milli>(t2 - t1).count();
  RCLCPP_WARN(node_->get_logger(), "Reset time: %f", reset_time);
  rrg_->setBoundMode(static_cast<BoundModeType>(req.bound_mode));
  rrg_->setRootStateForPlanning(req.root_pose);
  RCLCPP_INFO_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::DEBUG, "Root state: %f %f %f, %f", req.root_pose.position.x, req.root_pose.position.y, req.root_pose.position.z, tf2::getYaw(req.root_pose.orientation));


  // Start the planner.
  res.path.clear();
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    return false;
  }

  t1 = std::chrono::high_resolution_clock::now();
  rrg_->reset();

  if(opening_traversal_requested_ || rrg_->openingTraversalOngoing()) {
    if(opening_traversal_requested_) {
      opening_traversal_requested_ = false;
    }
    res.path = rrg_->getOpeningTraversalPath();
    if(rrg_->autoOpeningPathApproval()) {
      res.status = planner_msgs::srv::PlannerSrv::Response::AUTO_CUSTOM_PATH;
    }
    else {
      res.status = planner_msgs::srv::PlannerSrv::Response::MANUAL_CUSTOM_PATH;
    }
    if(res.path.empty()) {
      if(rrg_->openingTraversalOngoing()) {
        return true;
      }
    }
    else {
      return true;
    }
  }

  Rrg::GraphStatus status;
  if (planning_params_.graph_building_mode == GraphBuildingModeType::kBasic) {
    status = rrg_->buildGraph();
  } else if (planning_params_.graph_building_mode == GraphBuildingModeType::kBatch) {
    status = rrg_->batchGraph();
  }

  switch (status) {
    case Rrg::GraphStatus::OK:
      break;
    case Rrg::GraphStatus::ERR_KDTREE:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] An issue occurred with kdtree data.");
      break;
    case Rrg::GraphStatus::ERR_NO_FEASIBLE_PATH:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] No feasible path was found.");
      break;
    case Rrg::GraphStatus::NOT_OK:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[GBPLANNER] Resending global path");
      res.path = rrg_->reRunGlobalPlanner(res.status);
      break;
    default:
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] Error occurred in building graph.");
      break;
  }

  bool global_planner_trig = false;
  if (status == Rrg::GraphStatus::OK) {
    status = rrg_->evaluateGraph();
    switch (status) {
      case Rrg::GraphStatus::OK:
        break;
      case Rrg::GraphStatus::NO_GAIN:
        RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] No positive gain was found.");
        break;
      case Rrg::GraphStatus::NOT_OK: case Rrg::GraphStatus::CONSEC_LOW_GAIN:
        RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::PLANNER_STATUS, "[GBPLANNER] Very low local gain. Triggering global planner");
        int status;
        res.path = rrg_->runGlobalPlanner(0, false, false, status);
        if(status < 0) {
          if(rrg_->autoOpeningPathApproval()) {
            res.status = planner_msgs::srv::PlannerSrv::Response::AUTO_CUSTOM_PATH;
          }
          else {
            res.status = planner_msgs::srv::PlannerSrv::Response::MANUAL_CUSTOM_PATH;
          }
          opening_traversal_ongoing_ = true;
        }
        else {
          res.status = status;
        }
        global_planner_trig = true;
        break;
      default:
        RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] Error occurred in gain calculation.");
        break;
    }
  }
  else 
  {
    res.status = status;
    return false;
  }
  if (global_planner_trig) return true;

  if (status == Rrg::GraphStatus::OK) {
    if(planning_params_.exploration_only) {
      ++exploration_counter_;
    }
    res.path = rrg_->getBestPath(req.header.frame_id, res.status);
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::DEBUG, "[GBPLANNER] Regular Planning");
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::DEBUG, "[GBPLANNER] Path status: %d", res.status);
  }
  return true;
}

bool Gbplanner::getInspectionPath()
{
  rrg_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));
  if(planning_params_.basic_inspection_viewpoints)
    out_srv_res_.path = rrg_->getInspectionPathBasic();
  else
    out_srv_res_.path = rrg_->getInspectionPath();
  out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::AUTO_CUSTOM_PATH;

  if(out_srv_res_.path.size() > 0)
    return true;
  else 
    return false;
}

bool Gbplanner::getInspectionPath(planner_msgs::srv::PlannerSrv::Request& req,
      planner_msgs::srv::PlannerSrv::Response& res) {
  //
  if(planning_params_.basic_inspection_viewpoints)
    res.path = rrg_->getInspectionPathBasic();
  else
    res.path = rrg_->getInspectionPath();

  if(res.path.size() > 0)
    return true;
  else 
    return false;
}

void Gbplanner::getOpeningTraversalPath(OpeningTraversalMode mode, OpeningTraversalStatus &status)
{
  rrg_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));
  out_srv_res_.path = rrg_->getOpeningTraversalPath(mode, status);
  out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::AUTO_CUSTOM_PATH;
  if(status != OpeningTraversalStatus::OK)
  {
    out_srv_res_.path.clear();
  }
}

bool Gbplanner::getCompartmentTransitionPath() {
  //
  rrg_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));

  std::vector<geometry_msgs::msg::Pose> empty_path;

  // ++compartment_counter_;
  BoundedSpaceParams og_global_bb;
  rrg_->getGlobalBoundParams(og_global_bb);
  BoundedSpaceParams translated_bound = planning_params_.compartment_dimensions;
  Eigen::Vector3d max_val = planning_params_.compartment_dimensions.max_val + planning_params_.compartment_centers[compartment_counter_];
  Eigen::Vector3d max_extension = planning_params_.compartment_dimensions.max_extension + planning_params_.compartment_centers[compartment_counter_];
  Eigen::Vector3d min_val = planning_params_.compartment_dimensions.min_val + planning_params_.compartment_centers[compartment_counter_];
  Eigen::Vector3d min_extension = planning_params_.compartment_dimensions.min_extension + planning_params_.compartment_centers[compartment_counter_];
  translated_bound.setBound(min_val, max_val);
  BoundedSpaceParams extended_bound = planning_params_.compartment_dimensions;
  max_val = planning_params_.compartment_dimensions.max_val * 2.0;
  max_extension = planning_params_.compartment_dimensions.max_extension * 2.0;
  min_val = planning_params_.compartment_dimensions.min_val * 2.0;
  min_extension = planning_params_.compartment_dimensions.min_extension * 2.0;
  max_val += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
  max_extension += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
  min_val += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
  min_extension += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
  extended_bound.setBound(min_val, max_val);
  rrg_->setExplorationAndInspectionBounds(extended_bound, translated_bound);
  rrg_->reset();

  // ros::Duration(0.5).sleep();

  geometry_msgs::msg::Pose current_pose;
  tf2::Quaternion quat;
  quat.setEuler(0.0, 0.0, current_state_[3]);
  tf2::Vector3 origin(current_state_[0], current_state_[1], current_state_[2]);
  tf2::Transform poseTF(quat, origin);
  tf2::toMsg(poseTF, current_pose);

  geometry_msgs::msg::Pose target_pose;
  quat.setEuler(0.0, 0.0, 0.0);
  origin = tf2::Vector3(planning_params_.compartment_centers[compartment_counter_][0], planning_params_.compartment_centers[compartment_counter_][1], planning_params_.compartment_centers[compartment_counter_][2]);
  tf2::Transform poseTF_target(quat, origin);
  tf2::toMsg(poseTF_target, target_pose);

  std::vector<geometry_msgs::msg::Pose> connecting_path;
  bool search_success = rrg_->search(current_pose, target_pose, true, connecting_path);
  if(search_success) {
    rrg_->setExplorationAndInspectionBounds(translated_bound, translated_bound);
    out_srv_res_.path = connecting_path;
    out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::AUTO_CUSTOM_PATH;
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::DEBUG, "Compartment counter: %d", compartment_counter_);
  }
  else {
    out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::AUTO_CUSTOM_PATH;
    out_srv_res_.path = empty_path;
  }

  return search_success;
}

bool Gbplanner::getCompartmentTransitionPath(planner_msgs::srv::PlannerSrv::Request& req,
      planner_msgs::srv::PlannerSrv::Response& res) {
  //
  std::vector<geometry_msgs::msg::Pose> empty_path;

  if(planning_params_.enable_opening_traversal) {
    rrg_->setNextCompartmentCenter(planning_params_.compartment_centers[compartment_counter_+1]);
    rrg_->setNextCompartmentIndex(compartment_counter_+1);
    res.path = rrg_->getOpeningTraversalPath();
    if(rrg_->autoOpeningPathApproval()) {
      res.status = planner_msgs::srv::PlannerSrv::Response::AUTO_CUSTOM_PATH;
    }
    else {
      res.status = planner_msgs::srv::PlannerSrv::Response::MANUAL_CUSTOM_PATH;
    }

    if(!rrg_->openingTraversalOngoing()) {  // Opening traversal finished
      planner_mode_ = PlannerMode::kExploration;
      ++compartment_counter_;
      BoundedSpaceParams translated_bound = planning_params_.compartment_dimensions;
      Eigen::Vector3d max_val = planning_params_.compartment_dimensions.max_val + planning_params_.compartment_centers[compartment_counter_];
      Eigen::Vector3d max_extension = planning_params_.compartment_dimensions.max_extension + planning_params_.compartment_centers[compartment_counter_];
      Eigen::Vector3d min_val = planning_params_.compartment_dimensions.min_val + planning_params_.compartment_centers[compartment_counter_];
      Eigen::Vector3d min_extension = planning_params_.compartment_dimensions.min_extension + planning_params_.compartment_centers[compartment_counter_];
      translated_bound.setBound(min_val, max_val);
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::DEBUG, "Compartment counter: %d", compartment_counter_);
      rrg_->setExplorationAndInspectionBounds(translated_bound, translated_bound);
    }
    else {
      planner_mode_ = PlannerMode::kCompartmentChange;
    }

    bool success = false;
    if(res.path.empty()) {
      if(rrg_->openingTraversalOngoing()) {
        success = true;
      }
    }
    else {
      success = true;
    }

    return success;
  }
  else {
    ++compartment_counter_;
    BoundedSpaceParams translated_bound = planning_params_.compartment_dimensions;
    Eigen::Vector3d max_val = planning_params_.compartment_dimensions.max_val + planning_params_.compartment_centers[compartment_counter_];
    Eigen::Vector3d max_extension = planning_params_.compartment_dimensions.max_extension + planning_params_.compartment_centers[compartment_counter_];
    Eigen::Vector3d min_val = planning_params_.compartment_dimensions.min_val + planning_params_.compartment_centers[compartment_counter_];
    Eigen::Vector3d min_extension = planning_params_.compartment_dimensions.min_extension + planning_params_.compartment_centers[compartment_counter_];
    translated_bound.setBound(min_val, max_val);
    BoundedSpaceParams extended_bound = planning_params_.compartment_dimensions;
    max_val = planning_params_.compartment_dimensions.max_val * 2.0;
    max_extension = planning_params_.compartment_dimensions.max_extension * 2.0;
    min_val = planning_params_.compartment_dimensions.min_val * 2.0;
    min_extension = planning_params_.compartment_dimensions.min_extension * 2.0;
    max_val += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
    max_extension += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
    min_val += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
    min_extension += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
    extended_bound.setBound(min_val, max_val);
    rrg_->setExplorationAndInspectionBounds(extended_bound, translated_bound);
    rrg_->reset();

    // ros::Duration(0.5).sleep();

    geometry_msgs::msg::Pose current_pose;
    tf2::Quaternion quat;
    quat.setEuler(0.0, 0.0, current_state_[3]);
    tf2::Vector3 origin(current_state_[0], current_state_[1], current_state_[2]);
    tf2::Transform poseTF(quat, origin);
    tf2::toMsg(poseTF, current_pose);

    geometry_msgs::msg::Pose target_pose;
    quat.setEuler(0.0, 0.0, 0.0);
    origin = tf2::Vector3(planning_params_.compartment_centers[compartment_counter_][0], planning_params_.compartment_centers[compartment_counter_][1], planning_params_.compartment_centers[compartment_counter_][2]);
    tf2::Transform poseTF_target(quat, origin);
    tf2::toMsg(poseTF_target, target_pose);

    std::vector<geometry_msgs::msg::Pose> connecting_path;
    bool search_success = rrg_->search(current_pose, target_pose, true, connecting_path);
    if(search_success) {
      rrg_->setExplorationAndInspectionBounds(translated_bound, translated_bound);
      res.path = connecting_path;
      res.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
      planner_mode_ = PlannerMode::kExploration;
      RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::DEBUG, "Compartment counter: %d", compartment_counter_);
    }
    else {
      res.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
      res.path = empty_path;
    }
    return search_success;
  }
}

bool Gbplanner::homingRequired()
{
  rrg_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));
  out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::HOMING;
  return rrg_->homingRequired(out_srv_res_.path);
}

bool Gbplanner::getHomingPath()
{
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
    return false;
  }

  rrg_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));
  out_srv_res_.path = rrg_->getHomingPath(in_srv_req_.header.frame_id);
  if(out_srv_res_.path.empty())
  {
    out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
    return false;  
  }

  out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::HOMING;
  return true;
}

bool Gbplanner::calculateHomingPath()
{
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    // out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
    return false;
  }

  rrg_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));
  active_homing_path_ = rrg_->getHomingPath(in_srv_req_.header.frame_id);
  if(active_homing_path_.empty())
  {
    // out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::FORWARD;
    return false;  
  }

  rrg_->setLocalNavGoal(Eigen::Vector3d(active_homing_path_.back().position.x,
                              active_homing_path_.back().position.y,
                              active_homing_path_.back().position.z));

  // out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::HOMING;
  return true;
}

bool Gbplanner::updateHomingGoal()
{
  if(active_homing_path_.empty())
  {
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "No active homing path to update.");
    return false;  
  }

  Eigen::Vector3d current_position(current_state_[0], current_state_[1], current_state_[2]);
  Eigen::Vector3d homing_goal(active_homing_path_.back().position.x,
                              active_homing_path_.back().position.y,
                              active_homing_path_.back().position.z);
  for(size_t i = 0; i < active_homing_path_.size()-1; ++i)
  {
    Eigen::Vector3d waypoint(active_homing_path_[i].position.x,
                             active_homing_path_[i].position.y,
                             active_homing_path_[i].position.z);
    double distance = (waypoint - current_position).norm();
    if(distance < planning_params_.active_homing_update_radius)
    {
      // Remove this waypoint
      if(active_homing_path_.size() > 1)
      {
        active_homing_path_.erase(active_homing_path_.begin() + i);
        --i; // Adjust index after erasure
      }
      else
      {
        break;
      }
    }
    else 
    {
      // Since waypoints are ordered, we can break early
      homing_goal = waypoint;
      break;
    }
  }

  if(active_homing_path_.empty())
  {
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "Homing path completed.");
    return false;  
  }
  else
  {
    rrg_->setLocalNavGoal(homing_goal);
    // visualize homing goal
    geometry_msgs::msg::PoseStamped homing_goal_msg;
    homing_goal_msg.header.frame_id = planning_params_.global_frame_id;
    homing_goal_msg.header.stamp = node_->now();
    homing_goal_msg.pose.position.x = homing_goal[0];
    homing_goal_msg.pose.position.y = homing_goal[1];
    homing_goal_msg.pose.position.z = homing_goal[2];
    tf2::Quaternion goal_quat;
    goal_quat.setRPY(0.0, 0.0, 0.0);
    homing_goal_msg.pose.orientation = tf2::toMsg(goal_quat);
    global_planner_local_goal_pub_->publish(homing_goal_msg);
    return true;
  }
}

void Gbplanner::homingServiceCallback(
    const std::shared_ptr<planner_msgs::srv::PlannerHoming::Request> req,
    std::shared_ptr<planner_msgs::srv::PlannerHoming::Response> res) {
  RCLCPP_WARN(node_->get_logger(), "Homing through direct service call");
  res->path.clear();
  // ROS 2 services cannot report a transport-level failure the way returning
  // false did in ROS 1; an empty path is the answer instead.
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    return;
  }
  res->path = rrg_->getHomingPath(req->header.frame_id);
}

void Gbplanner::globalPlannerServiceCallback(
    const std::shared_ptr<planner_msgs::srv::PlannerGlobal::Request> req,
    std::shared_ptr<planner_msgs::srv::PlannerGlobal::Response> res) {
  res->path.clear();
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    return;
  }
  int status;
  res->path =
      rrg_->runGlobalPlanner(req->id, req->not_check_frontier, req->ignore_time, status);
}

void Gbplanner::setHomingPosServiceCallback(
    const std::shared_ptr<planner_msgs::srv::PlannerSetHomingPos::Request> req,
    std::shared_ptr<planner_msgs::srv::PlannerSetHomingPos::Response> res) {
  (void)req;
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    return;
  }
  res->success = rrg_->setHomingPos();
}

void Gbplanner::plannerSearchServiceCallback(
    const std::shared_ptr<planner_msgs::srv::PlannerSearch::Request> req,
    std::shared_ptr<planner_msgs::srv::PlannerSearch::Response> res) {
  rrg_->setBoundMode(static_cast<BoundModeType>(req->bound_mode));
  res->success =
      rrg_->search(req->source, req->target, req->use_current_state, res->path);
}

void Gbplanner::plannerSetPlanningTriggerModeCallback(
    const std::shared_ptr<planner_msgs::srv::PlannerSetPlanningMode::Request> request,
    std::shared_ptr<planner_msgs::srv::PlannerSetPlanningMode::Response> response) {
  PlannerTriggerModeType in_trig_mode;
  if (request->planning_mode == request->AUTO)
    in_trig_mode = PlannerTriggerModeType::kAuto;
  else if (request->planning_mode == request->MANUAL)
    in_trig_mode = PlannerTriggerModeType::kManual;
  rrg_->setPlannerTriggerMode(in_trig_mode);
  response->success = true;
}

void Gbplanner::clearUntraversableZones(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  (void)req;
  RCLCPP_INFO_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::INFO, "Clearing untraversable zones");
  rrg_->clearUntraversableZones();
  res->success = true;
}

void Gbplanner::untraversablePolygonCallback(
    const geometry_msgs::msg::PolygonStamped& polygon_msgs) {
  // Add the new polygon into geofence list
  if (!polygon_msgs.polygon.points.empty()) {
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "Detected untraversable area");
    // Add this to the list
    rrg_->addGeofenceAreas(polygon_msgs);
  }
}

void Gbplanner::setUntraversablePolygon(
    const geometry_msgs::msg::PolygonStamped& polygon_msgs) {
  std::cout << "Untraversable polygon size: "
            << polygon_msgs.polygon.points.size() << std::endl;
  if (!polygon_msgs.polygon.points.empty()) {
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), global_verbosity >= Verbosity::WARN, "Detected untraversable area");
    // Add this to the list
    rrg_->addGeofenceAreas(polygon_msgs);
  }
}

void Gbplanner::poseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped& pose) {
  processPose(pose.pose.pose);
}

void Gbplanner::poseStampedCallback(const geometry_msgs::msg::PoseStamped& pose) {
  processPose(pose.pose);
}

void Gbplanner::processPose(const geometry_msgs::msg::Pose& pose) {
  StateVec state;
  state[0] = pose.position.x;
  state[1] = pose.position.y;
  state[2] = pose.position.z;
  state[3] = tf2::getYaw(pose.orientation);
  rrg_->setState(state);
  current_state_ = state;
}

void Gbplanner::odometryCallback(const nav_msgs::msg::Odometry& odo) {
  StateVec state;
  state[0] = odo.pose.pose.position.x;
  state[1] = odo.pose.pose.position.y;
  state[2] = odo.pose.pose.position.z;
  state[3] = tf2::getYaw(odo.pose.pose.orientation);
  rrg_->setState(state);
  current_state_ = state;
}

void Gbplanner::robotStatusCallback(const planner_msgs::msg::RobotStatus& status) {
  rrg_->setTimeRemaining(status.time_remaining);
}

void Gbplanner::localNavGoalCallback(const geometry_msgs::msg::PoseStamped& goal)
{
  Eigen::Vector3d local_nav_goal;
  local_nav_goal[0] = goal.pose.position.x;
  local_nav_goal[1] = goal.pose.position.y;
  local_nav_goal[2] = goal.pose.position.z;
  rrg_->setLocalNavGoal(local_nav_goal);
  RCLCPP_WARN(node_->get_logger(), "Received local navigation goal: %f, %f, %f", local_nav_goal[0], local_nav_goal[1], local_nav_goal[2]);
}

void Gbplanner::stopMsgCallback(const std_msgs::msg::Bool& msg)
{
  bt_states_.homing_required = false;
}

Gbplanner::PlannerStatus Gbplanner::getPlannerStatus() {


  // Should have a list of checking conditions to set the planner as ready.
  // For examples:
  // + ROS ok
  // + All params loaded properly
  // + Map is ready to use
  if (planner_status_ == Gbplanner::PlannerStatus::READY)
    return Gbplanner::PlannerStatus::READY;

  return Gbplanner::PlannerStatus::READY;
}

// }  // namespace explorer
