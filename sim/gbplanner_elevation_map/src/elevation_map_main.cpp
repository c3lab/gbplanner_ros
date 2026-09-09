#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "gbplanner_elevation_map/elevation_map_node.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<gbplanner_elevation_map::ElevationMapNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
