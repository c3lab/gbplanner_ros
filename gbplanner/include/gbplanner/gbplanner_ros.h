#pragma once

#include <rclcpp/rclcpp.hpp>

#include "gbplanner/gbplanner.h"
#include "gbplanner/gbplanner_bt_nodes.h"

class GbplannerRos
{
private:
  rclcpp::Node* node_;

  rclcpp::Service<planner_msgs::srv::PlannerSrv>::SharedPtr planner_service_;
  rclcpp::Service<planner_msgs::srv::PlannerHoming>::SharedPtr planner_homing_service_;

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::shared_ptr<Gbplanner> gbplanner_;
public:
  explicit GbplannerRos(rclcpp::Node* node);
  void plannerServiceCallback(
      const std::shared_ptr<planner_msgs::srv::PlannerSrv::Request> req,
      std::shared_ptr<planner_msgs::srv::PlannerSrv::Response> res);
  void plannerHomingServiceCallback(
      const std::shared_ptr<planner_msgs::srv::PlannerHoming::Request> req,
      std::shared_ptr<planner_msgs::srv::PlannerHoming::Response> res);

  void registerTree();

};
