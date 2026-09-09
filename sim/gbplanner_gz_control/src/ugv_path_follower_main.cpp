#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "gbplanner_gz_control/ugv_path_follower_node.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<gbplanner_gz_control::UGVPathFollowerNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
