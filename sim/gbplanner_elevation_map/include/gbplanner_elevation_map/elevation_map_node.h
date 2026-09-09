#ifndef GBPLANNER_ELEVATION_MAP__ELEVATION_MAP_NODE_H_
#define GBPLANNER_ELEVATION_MAP__ELEVATION_MAP_NODE_H_

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace gbplanner_elevation_map
{

/// Builds a 2.5D height map from the robot's point cloud and publishes it as
/// grid_map_msgs/GridMap with an "elevation" layer.
///
/// This is the input gbplanner's ground-robot mode needs and that nothing in
/// gbplanner_ros provides. Rrg::projectSampleEleMap reads the layer at the
/// sample and at the four corners of the robot's footprint: every one of them
/// has to be inside the map and non-NaN, the centre sets the sample's height to
/// elevation + PlanningParams.max_ground_height, and a corner steeper than
/// PlanningParams.max_inclination relative to the centre rejects the sample.
/// So the two things that matter here are that observed ground is accurate and
/// that *unobserved ground stays NaN* - an unobserved cell is what makes the
/// planner treat a vertex as hanging instead of walking the robot over a ledge
/// it has never seen.
///
/// The cell value is the highest point in the cell below the robot's head,
/// which is the convention traversability layers use: the floor gives the floor
/// height, and a rock in the cell raises it, so the inclination check refuses
/// to step onto it. Points above the robot are dropped, or the ceiling would
/// become the terrain.
///
/// Heights accumulate in a global sparse grid and a window of it is published
/// around the robot, so ground the robot has driven past stays known when it
/// comes back - the planner's global graph outlives the local window.
class ElevationMapNode : public rclcpp::Node
{
public:
  explicit ElevationMapNode(const rclcpp::NodeOptions & options);

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2 & cloud);
  void odometryCallback(const nav_msgs::msg::Odometry & odom);
  void publishWindow();

  /// Mark the ground the robot is standing on as observed.
  ///
  /// No sensor sees underneath a robot, but a robot standing up is direct
  /// evidence of ground at its own feet, and elevation_mapping uses exactly
  /// that. Without it the root vertex's own cell is NaN and the planner refuses
  /// the robot's current position.
  void seedFootprint();

  /// One cell of the global accumulation grid.
  struct Cell
  {
    float height{0.0F};
    uint32_t hits{0};
    /// Written by seedFootprint rather than by a sensor. Kept apart because a
    /// seeded cell has to stay correctable - the first odometry arrives while
    /// the robot is still falling onto its wheels - while an observed one must
    /// never be overwritten by an assumption.
    bool seeded{false};
  };

  static int64_t key(int ix, int iy)
  {
    return (static_cast<int64_t>(ix) << 32) ^ static_cast<uint32_t>(iy);
  }
  int index(double v) const { return static_cast<int>(std::floor(v / resolution_)); }
  double centre(int i) const { return (static_cast<double>(i) + 0.5) * resolution_; }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  /// Close-range ground sensing. The lidar's fan leaves a blind disc around the
  /// robot that no edge can cross; a downward camera fills it, which is how the
  /// real robot's elevation_mapping gets its near field too.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unordered_map<int64_t, Cell> cells_;

  std::string world_frame_;
  double resolution_{0.0};
  double map_length_{0.0};
  double ceiling_margin_{0.0};
  double min_range_{0.0};
  double max_range_{0.0};
  int min_hits_{0};
  double self_filter_radius_{0.0};
  bool seed_footprint_{false};
  double seed_radius_{0.0};
  double robot_height_{0.0};

  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_z_{0.0};
  bool have_odometry_{false};
};

}  // namespace gbplanner_elevation_map

#endif  // GBPLANNER_ELEVATION_MAP__ELEVATION_MAP_NODE_H_
