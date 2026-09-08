#include "gbplanner/gbplanner_ros.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

GbplannerRos::GbplannerRos(rclcpp::Node* node) : node_(node)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  planner_service_ = node_->create_service<planner_msgs::srv::PlannerSrv>(
      "gbplanner_ros",
      std::bind(&GbplannerRos::plannerServiceCallback, this, _1, _2));
  
  planner_homing_service_ = node_->create_service<planner_msgs::srv::PlannerHoming>(
      "gbplanner_ros_homing",
      std::bind(&GbplannerRos::plannerHomingServiceCallback, this, _1, _2));
  
  gbplanner_.reset(new Gbplanner(node_));

  registerTree();
}

void GbplannerRos::plannerServiceCallback(
    const std::shared_ptr<planner_msgs::srv::PlannerSrv::Request> req,
    std::shared_ptr<planner_msgs::srv::PlannerSrv::Response> res)
{
  gbplanner_->setPlannerSrvReq(*req);
  
  tree_.tickOnce();

  gbplanner_->getPlannerSrvRes(*res);
  std::cout << "Sending response to PCI: " << res->status << std::endl;
}

void GbplannerRos::plannerHomingServiceCallback(
    const std::shared_ptr<planner_msgs::srv::PlannerHoming::Request> req,
    std::shared_ptr<planner_msgs::srv::PlannerHoming::Response> res)
{
  (void)req;
  (void)res;
  RCLCPP_WARN(node_->get_logger(), "Homing through BT");
  gbplanner_->bt_states_.homing_required = true;

  // planner_msgs::srv::PlannerSrv::Request req_planner;
  // planner_msgs::srv::PlannerSrv::Response res_planner;
  // req_planner.header = req.header;
  // req_planner.bound_mode = planner_msgs::msg::BoundMode::EXTENDED_BOUND;

  // gbplanner_->setPlannerSrvReq(req_planner);
  // tree_.tickOnce();
  // gbplanner_->getPlannerSrvRes(res_planner);

  // res->path = res_planner.path;
}

void GbplannerRos::registerTree()
{
  factory_.registerNodeType<LocalExploration>("LocalExploration", gbplanner_);
  factory_.registerNodeType<GlobalExploration>("GlobalExploration", gbplanner_);
  factory_.registerNodeType<LocalExpExhaustedCheck>("LocalExpExhaustedCheck", gbplanner_);
  factory_.registerNodeType<GlobalExpExhaustedCheck>("GlobalExpExhaustedCheck", gbplanner_);
  factory_.registerNodeType<Inspection>("Inspection", gbplanner_);
  factory_.registerNodeType<CompartmentTransition>("CompartmentTransition", gbplanner_);
  factory_.registerNodeType<Homing>("Homing", gbplanner_);
  factory_.registerNodeType<HomingCheck>("HomingCheck", gbplanner_);
  factory_.registerNodeType<OPENINGPhase1>("OPENINGPhase1", gbplanner_);
  factory_.registerNodeType<OPENINGPhaseCheck>("OPENINGPhaseCheck", gbplanner_);
  factory_.registerNodeType<OPENINGPhase2>("OPENINGPhase2", gbplanner_);
  factory_.registerNodeType<LocalExpExhaustedReset>("LocalExpExhaustedReset", gbplanner_);
  factory_.registerNodeType<Idle>("Idle", gbplanner_);
  factory_.registerNodeType<OPENINGP1FailCheck>("OPENINGP1FailCheck", gbplanner_);
  factory_.registerNodeType<SetNextCompartment>("SetNextCompartment", gbplanner_);
  factory_.registerNodeType<AllCompartmentsInspectedCheck>("AllCompartmentsInspectedCheck", gbplanner_);
  factory_.registerNodeType<LocalNavigation>("LocalNavigation", gbplanner_);
  factory_.registerNodeType<LocalNavigationExhaustedCheck>("LocalNavigationExhaustedCheck", gbplanner_);
  factory_.registerNodeType<LocalNavigationExhaustedReset>("LocalNavigationExhaustedReset", gbplanner_);
  factory_.registerNodeType<CalculateHomingPath>("CalculateHomingPath", gbplanner_);
  factory_.registerNodeType<UpdateHomingGoal>("UpdateHomingGoal", gbplanner_);
  factory_.registerNodeType<SwitchToLocalNavigation>("SwitchToLocalNavigation", gbplanner_);
  factory_.registerNodeType<CalculateGlobalPath>("CalculateGlobalPath", gbplanner_);
  factory_.registerNodeType<UpdateGlobalGoal>("UpdateGlobalGoal", gbplanner_);

  // get_package_share_directory throws where ros::package::getPath returned an
  // empty string, so the default has to be built inside the guard.
  std::string default_tree_path;
  try {
    default_tree_path =
        ament_index_cpp::get_package_share_directory("gbplanner") +
        "/config/bt_xml/main_tree.xml";
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Could not locate the gbplanner share directory: %s", e.what());
  }
  std::string tree_path = default_tree_path;
  if (!getParamOpt(node_, "behavior_tree_path", tree_path)) {
    tree_path = default_tree_path;
  }
  std::string trial_tree_path;
  getParamOpt(node_, "tree_path", trial_tree_path);

  RCLCPP_WARN_STREAM(node_->get_logger(), "Tree path: " << tree_path << " Trial Tree Path: " << trial_tree_path);
  
  factory_.registerBehaviorTreeFromFile(tree_path);
  tree_ = factory_.createTree("MainTree");
	std::cout << "Behavior Tree built" << std::endl;

  std::string xml_models = BT::writeTreeNodesModelXML(factory_);

  std::cout << "TreeNodesMode: " << xml_models << std::endl;

  // BT::Groot2Publisher publisher(tree_);
}
