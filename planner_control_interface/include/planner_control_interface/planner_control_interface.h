#ifndef PLANNER_CONTROL_INTERFACE_H_
#define PLANNER_CONTROL_INTERFACE_H_

#include <mutex>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/menu_entry.hpp>

#include "planner_control_interface/pci_manager.h"
#include "planner_msgs/msg/bound_mode.hpp"
#include "planner_msgs/msg/execution_path_mode.hpp"
#include "planner_msgs/msg/planner_status.hpp"
#include "planner_msgs/msg/planning_mode.hpp"
#include "planner_msgs/msg/trigger_mode.hpp"
#include "planner_msgs/srv/pci_geofence.hpp"
#include "planner_msgs/srv/pci_global.hpp"
#include "planner_msgs/srv/pci_homing_trigger.hpp"
#include "planner_msgs/srv/pci_initialization.hpp"
#include "planner_msgs/srv/pci_search.hpp"
#include "planner_msgs/srv/pci_set_homing_pos.hpp"
#include "planner_msgs/srv/pci_stop.hpp"
#include "planner_msgs/srv/pci_to_waypoint.hpp"
#include "planner_msgs/srv/pci_trigger.hpp"
#include "planner_msgs/srv/planner_geofence.hpp"
#include "planner_msgs/srv/planner_global.hpp"
#include "planner_msgs/srv/planner_go_to_waypoint.hpp"
#include "planner_msgs/srv/planner_homing.hpp"
#include "planner_msgs/srv/planner_request_path.hpp"
#include "planner_msgs/srv/planner_search.hpp"
#include "planner_msgs/srv/planner_set_exp_mode.hpp"
#include "planner_msgs/srv/planner_set_homing_pos.hpp"
#include "planner_msgs/srv/planner_set_planning_mode.hpp"
#include "planner_msgs/srv/planner_srv.hpp"
#include "planner_semantic_msgs/msg/semantic_point.hpp"

namespace explorer {

class PlannerControlInterface {
 public:
  enum struct RobotModeType { kAerialRobot = 0, kLeggedRobot };
  enum struct RunModeType {
    kSim = 0,  // Run in simulation.
    kReal,     // Run with real robot.
  };
  enum struct PlannerTriggerModeType {
    kManual = 0,  // Manually trigger the control interface each time.
    kAuto = 1     // Automatic exploration.
  };

  PlannerControlInterface(rclcpp::Node* node,
                          std::shared_ptr<PCIManager> pci_manager);

 protected:
  rclcpp::Node* node_;

 private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr reference_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr planner_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_request_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      pose_stamped_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      nav_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      pose_goal_sub_;
  rclcpp::Client<planner_msgs::srv::PlannerSrv>::SharedPtr planner_client_;
  rclcpp::Client<planner_msgs::srv::PlannerHoming>::SharedPtr
      planner_homing_client_;
  rclcpp::Client<planner_msgs::srv::PlannerSetHomingPos>::SharedPtr
      planner_set_homing_pos_client_;
  rclcpp::Client<planner_msgs::srv::PlannerSearch>::SharedPtr
      planner_search_client_;
  rclcpp::Client<planner_msgs::srv::PlannerGlobal>::SharedPtr
      planner_global_client_;
  rclcpp::Client<planner_msgs::srv::PlannerGeofence>::SharedPtr
      planner_geofence_client_;
  rclcpp::Client<planner_msgs::srv::PlannerRequestPath>::SharedPtr
      planner_passing_gate_client_;
  rclcpp::Client<planner_msgs::srv::PlannerSetExpMode>::SharedPtr
      planner_set_exp_mode_client_;
  rclcpp::Client<planner_msgs::srv::PlannerGoToWaypoint>::SharedPtr
      nav_goal_client_;
  rclcpp::Client<planner_msgs::srv::PlannerSetPlanningMode>::SharedPtr
      planner_set_trigger_mode_client_;
  rclcpp::Client<planner_msgs::srv::PlannerSrv>::SharedPtr
      planner_inspection_srv_client_;

  rclcpp::Service<planner_msgs::srv::PciTrigger>::SharedPtr pci_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      pci_std_automatic_planning_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      pci_std_single_planning_server_;
  rclcpp::Service<planner_msgs::srv::PciHomingTrigger>::SharedPtr
      pci_homing_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      pci_std_set_homing_pos_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pci_std_homing_server_;
  rclcpp::Service<planner_msgs::srv::PciSetHomingPos>::SharedPtr
      pci_set_homing_pos_server_;
  rclcpp::Service<planner_msgs::srv::PciInitialization>::SharedPtr
      pci_initialization_server_;
  rclcpp::Service<planner_msgs::srv::PciSearch>::SharedPtr pci_search_server_;
  rclcpp::Service<planner_msgs::srv::PciGlobal>::SharedPtr pci_global_server_;
  rclcpp::Service<planner_msgs::srv::PciStop>::SharedPtr pci_stop_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pci_std_stop_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      pci_std_go_to_waypoint_server_;
  rclcpp::Service<planner_msgs::srv::PciGeofence>::SharedPtr
      pci_geofence_server_;
  rclcpp::Service<planner_msgs::srv::PciToWaypoint>::SharedPtr
      pci_to_waypoint_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pci_passing_gate_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rotate_180_deg_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      pci_std_global_last_specified_frontier_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pci_inspection_srv_server_;

  // Shared with the PCIManager, see its constructor for why there are two.
  rclcpp::CallbackGroup::SharedPtr main_cb_group_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::TimerBase::SharedPtr run_timer_;
  bool services_connected_ = false;
  bool initialized_ = false;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<PCIManager> pci_manager_;
  uint8_t bound_mode_;
  PlannerTriggerModeType trigger_mode_;
  double v_current_;
  bool run_en_;
  bool exe_path_en_;
  bool force_forward_;
  bool homing_request_;
  bool pose_is_ready_;
  bool init_request_;
  bool global_request_;
  bool stop_planner_request_;
  bool inspection_srv_request_ = false;

  bool passing_gate_success_;
  bool passing_gate_request_;

  geometry_msgs::msg::Pose set_waypoint_;
  geometry_msgs::msg::PoseStamped set_waypoint_stamped_;
  // Visualization Publisher
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      go_to_waypoint_visualization_pub_;

  bool go_to_waypoint_request_;
  bool go_to_waypoint_with_checking_;
  bool received_first_waypoint_to_go_ = false;

  int reference_pub_id_;
  planner_msgs::srv::PciGlobal::Request pci_global_request_params_;
  int frontier_id_;

  std::string world_frame_name = "world";

  // Semantics marker
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> semantic_server;

  interactive_markers::MenuHandler menu_handler;
  interactive_markers::MenuHandler::EntryHandle accept_entry_handle;
  interactive_markers::MenuHandler::EntryHandle class_entry_handle;
  interactive_markers::MenuHandler::EntryHandle sub_class_entry_handle;

  const std::string kStaircaseStr = "Stairs";
  const std::string kDoorStr = "Door";
  planner_semantic_msgs::msg::SemanticPoint semantic_location;
  planner_semantic_msgs::msg::SemanticClass current_semantic_class_;
  geometry_msgs::msg::Point32 semantic_position;

  rclcpp::Publisher<planner_semantic_msgs::msg::SemanticPoint>::SharedPtr
      semantic_pub;

  double control_size = 1.0;

  bool menu_initialized = false;

  // Current following path.
  std::vector<geometry_msgs::msg::Pose> current_path_;

  int planner_iteration_;
  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Pose previous_pose_;
  std::string world_frame_id_;

  void odometryCallback(const nav_msgs::msg::Odometry& odo);
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& pose);
  void navGoalCallback(const geometry_msgs::msg::PoseStamped& nav_msg);
  void poseGoalCallback(const geometry_msgs::msg::PoseStamped& pose_msg);
  void setGoal(const geometry_msgs::msg::PoseStamped& pose);
  void poseStampedCallback(const geometry_msgs::msg::PoseStamped& pose);
  void processPose(const geometry_msgs::msg::Pose& pose);

  void setHomingPosCallback(
      const planner_msgs::srv::PciSetHomingPos::Request::SharedPtr req,
      planner_msgs::srv::PciSetHomingPos::Response::SharedPtr res);
  void stdSrvSetHomingPositionHereCallback(
      const std_srvs::srv::Trigger::Request::SharedPtr req,
      std_srvs::srv::Trigger::Response::SharedPtr res);

  void homingCallback(
      const planner_msgs::srv::PciHomingTrigger::Request::SharedPtr req,
      planner_msgs::srv::PciHomingTrigger::Response::SharedPtr res);
  void stdSrvHomingCallback(
      const std_srvs::srv::Trigger::Request::SharedPtr req,
      std_srvs::srv::Trigger::Response::SharedPtr res);
  void triggerCallback(
      const planner_msgs::srv::PciTrigger::Request::SharedPtr req,
      planner_msgs::srv::PciTrigger::Response::SharedPtr res);
  void stdSrvsAutomaticPlanningCallback(
      const std_srvs::srv::Trigger::Request::SharedPtr req,
      std_srvs::srv::Trigger::Response::SharedPtr res);

  void stdSrvGoToWaypointCallback(
      const std_srvs::srv::Trigger::Request::SharedPtr req,
      std_srvs::srv::Trigger::Response::SharedPtr res);

  void stdSrvsSinglePlanningCallback(
      const std_srvs::srv::Trigger::Request::SharedPtr req,
      std_srvs::srv::Trigger::Response::SharedPtr res);

  void initializationCallback(
      const planner_msgs::srv::PciInitialization::Request::SharedPtr req,
      planner_msgs::srv::PciInitialization::Response::SharedPtr res);

  void searchCallback(
      const planner_msgs::srv::PciSearch::Request::SharedPtr req,
      planner_msgs::srv::PciSearch::Response::SharedPtr res);

  void globalPlannerCallback(
      const planner_msgs::srv::PciGlobal::Request::SharedPtr req,
      planner_msgs::srv::PciGlobal::Response::SharedPtr res);

  void stopPlannerCallback(
      const planner_msgs::srv::PciStop::Request::SharedPtr req,
      planner_msgs::srv::PciStop::Response::SharedPtr res);
  void stdSrvStopPlannerCallback(
      const std_srvs::srv::Trigger::Request::SharedPtr req,
      std_srvs::srv::Trigger::Response::SharedPtr res);

  void addGeofenceCallback(
      const planner_msgs::srv::PciGeofence::Request::SharedPtr req,
      planner_msgs::srv::PciGeofence::Response::SharedPtr res);
  void goToWaypointCallback(
      const planner_msgs::srv::PciToWaypoint::Request::SharedPtr req,
      planner_msgs::srv::PciToWaypoint::Response::SharedPtr res);
  void passingGateCallback(
      const std_srvs::srv::Trigger::Request::SharedPtr req,
      std_srvs::srv::Trigger::Response::SharedPtr res);

  void rotate180DegCallback(
      const std_srvs::srv::Trigger::Request::SharedPtr req,
      std_srvs::srv::Trigger::Response::SharedPtr res);

  void inspectionSrvCallback(
      const std_srvs::srv::Trigger::Request::SharedPtr req,
      std_srvs::srv::Trigger::Response::SharedPtr res);

  void stdSrvReplanLastSpecifiedFrontierCallback(
      const std_srvs::srv::Trigger::Request::SharedPtr req,
      std_srvs::srv::Trigger::Response::SharedPtr res);
  void resetPlanner();

  bool loadParams();
  bool init();
  void run();
  void runPlanner(bool exe_path);
  void runGlobalPlanner(bool exe_path);
  void runHoming(bool exe_path);
  void runInitialization();
  void runSearch(bool exe_path);
  void runPassingGate();
  void runGlobalRepositioning();
  geometry_msgs::msg::Pose getPoseToStart();
  void runInspection();

  bool search_request_;
  bool use_current_state_;
  const std::string source_marker_name = "wp_source";
  const std::string target_marker_name = "wp_target";
  // Written field by field from the interactive-marker feedback callback and
  // read field by field from runSearch. InteractiveMarkerServer takes node
  // interfaces, not a callback group, so its feedback subscription lands in the
  // node's default group and can run while the run timer is in flight - the two
  // could not overlap under ROS 1's single spin thread. A callback group is the
  // wrong tool here: it would serialise the marker, which has to stay responsive
  // while the planner works. Guard the data instead.
  std::mutex setpoint_mutex_;
  geometry_msgs::msg::Pose source_setpoint_;
  geometry_msgs::msg::Pose target_setpoint_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> imarker_server_;
  // Waypoints i-markers
  void initIMarker();
  void processFeedback(
      visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
          feedback);
  void publishPlannerStatus(const planner_msgs::srv::PlannerSrv::Response& res,
                            bool success);
  void publishGoToWaypointVisualization(
      const geometry_msgs::msg::PoseStamped& poseStamped);
  // Semantics i-marker
  void initSemanticIMarker();
  void semanticMarkerFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr&
          feedback);
  void acceptButtonFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr&
          feedback);
  void selectSemanticsFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr&
          feedback);
};
}  // namespace explorer

#endif
