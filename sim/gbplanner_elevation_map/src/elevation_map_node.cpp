#include "gbplanner_elevation_map/elevation_map_node.h"

#include <cmath>
#include <limits>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace gbplanner_elevation_map
{

ElevationMapNode::ElevationMapNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("elevation_map_node", options)
{
  world_frame_ = declare_parameter<std::string>("world_frame", "world");
  resolution_ = declare_parameter<double>("resolution", 0.2);
  map_length_ = declare_parameter<double>("map_length", 44.0);
  ceiling_margin_ = declare_parameter<double>("ceiling_margin", 1.0);
  min_range_ = declare_parameter<double>("min_range", 1.0);
  max_range_ = declare_parameter<double>("max_range", 20.0);
  min_hits_ = declare_parameter<int>("min_hits", 2);
  self_filter_radius_ = declare_parameter<double>("self_filter_radius", 0.75);
  seed_footprint_ = declare_parameter<bool>("seed_footprint", true);
  seed_radius_ = declare_parameter<double>("seed_radius", 0.6);
  robot_height_ = declare_parameter<double>("robot_height", 0.6);
  const double publish_rate = declare_parameter<double>("publish_rate", 4.0);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

  // Transient local so a planner that starts late still gets the map, and depth
  // 1 because only the newest one is ever of any use.
  map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
    "elevation_map", rclcpp::QoS(1).transient_local());

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2 & msg) {cloudCallback(msg);});
  ground_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud_ground", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2 & msg) {cloudCallback(msg);});
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odometry", 10,
    [this](const nav_msgs::msg::Odometry & msg) {odometryCallback(msg);});

  publish_timer_ = rclcpp::create_timer(
    this, get_clock(),
    rclcpp::Duration::from_seconds(1.0 / std::max(0.1, publish_rate)),
    [this]() {publishWindow();});

  RCLCPP_INFO(
    get_logger(),
    "Elevation map: %.2f m cells, %.0f m window, frame '%s'.",
    resolution_, map_length_, world_frame_.c_str());
}

void ElevationMapNode::odometryCallback(const nav_msgs::msg::Odometry & odom)
{
  robot_x_ = odom.pose.pose.position.x;
  robot_y_ = odom.pose.pose.position.y;
  robot_z_ = odom.pose.pose.position.z;
  have_odometry_ = true;
  seedFootprint();
}

void ElevationMapNode::seedFootprint()
{
  if (!seed_footprint_) {
    return;
  }
  // The robot is standing on it, so it is ground, at the height of its feet.
  const float ground = static_cast<float>(robot_z_ - robot_height_);
  const int reach = static_cast<int>(std::ceil(seed_radius_ / resolution_));
  const int cx = index(robot_x_);
  const int cy = index(robot_y_);
  for (int dx = -reach; dx <= reach; ++dx) {
    for (int dy = -reach; dy <= reach; ++dy) {
      if (std::hypot(centre(cx + dx) - robot_x_, centre(cy + dy) - robot_y_) >
        seed_radius_)
      {
        continue;
      }
      Cell & cell = cells_[key(cx + dx, cy + dy)];
      // Refreshed on every odometry message, not written once. The first
      // odometry arrives while the robot is still dropping onto its wheels
      // after being spawned, and a seed taken then is wrong by exactly that
      // drop: measured, the disc under the robot read 0.30 m on a floor at
      // zero, and every sample failed the inclination check against its own
      // neighbours. A cell a sensor has seen is never touched.
      if (!cell.seeded && cell.hits > 0) {
        continue;
      }
      cell.seeded = true;
      cell.height = ground;
      // Seeded cells count as observed immediately: min_hits guards against a
      // stray return, and this is not one.
      cell.hits = static_cast<uint32_t>(min_hits_);
    }
  }
}

void ElevationMapNode::cloudCallback(const sensor_msgs::msg::PointCloud2 & cloud)
{
  if (!have_odometry_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Waiting for odometry before integrating clouds.");
    return;
  }

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      world_frame_, cloud.header.frame_id, cloud.header.stamp,
      tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "No transform %s -> %s: %s", cloud.header.frame_id.c_str(),
      world_frame_.c_str(), e.what());
    return;
  }

  const Eigen::Isometry3d sensor_to_world = tf2::transformToEigen(tf);
  // Above this a point is ceiling, wall or overhang, not terrain. Anchored to
  // the robot rather than to the terrain, which is the thing being measured.
  const double ceiling = robot_z_ + ceiling_margin_;

  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");

  size_t used = 0;
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    const float x = *it_x;
    const float y = *it_y;
    const float z = *it_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    // In the sensor frame, so this is the real beam length: near returns are
    // the robot seeing itself, far ones are past the sensor's useful range.
    const double range = std::sqrt(
      static_cast<double>(x) * x + static_cast<double>(y) * y +
      static_cast<double>(z) * z);
    if (range < min_range_ || range > max_range_) {
      continue;
    }

    const Eigen::Vector3d p = sensor_to_world * Eigen::Vector3d(x, y, z);
    if (p.z() > ceiling) {
      continue;
    }
    // The robot does not map itself. Both sensors raycast the render scene, so
    // they see the leg visuals and the wheels, and taking the highest return in
    // a cell then turns the robot's own legs into 0.30 m of terrain directly
    // underneath it - measured, and it made every sample fail the inclination
    // check against its neighbours. A radius rather than a body model because
    // that is all the accuracy this needs: what is inside it is known ground
    // already, from seedFootprint.
    if (std::hypot(p.x() - robot_x_, p.y() - robot_y_) < self_filter_radius_) {
      continue;
    }

    Cell & cell = cells_[key(index(p.x()), index(p.y()))];
    if (cell.seeded) {
      // An observation beats the standing-on-it assumption.
      cell.seeded = false;
      cell.hits = 0;
    }
    // The highest return in the cell, below the robot's head: the floor gives
    // the floor, and anything standing in the cell raises it, so the planner's
    // inclination check refuses to step onto it.
    if (cell.hits == 0 || p.z() > cell.height) {
      cell.height = static_cast<float>(p.z());
    }
    ++cell.hits;
    ++used;
  }

  if (used == 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Cloud of %u points contributed nothing: all outside [%.2f, %.2f] m or "
      "above the robot.", cloud.width * cloud.height, min_range_, max_range_);
  }
}

void ElevationMapNode::publishWindow()
{
  if (!have_odometry_ || cells_.empty()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Nothing to publish yet: odometry %s, %zu cells accumulated.",
      have_odometry_ ? "received" : "MISSING", cells_.size());
    return;
  }

  grid_map::GridMap map({"elevation"});
  map.setFrameId(world_frame_);
  map.setGeometry(
    grid_map::Length(map_length_, map_length_), resolution_,
    grid_map::Position(robot_x_, robot_y_));
  // NaN is the whole point: Rrg::projectSampleEleMap calls isValid() on every
  // probe, and an unobserved cell has to fail it. A zero-filled map would tell
  // the planner that ground it has never seen is flat and at the world origin's
  // height, which is exactly how a robot walks off a ledge.
  map["elevation"].setConstant(NAN);

  size_t filled = 0;
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    grid_map::Position pos;
    map.getPosition(*it, pos);
    const auto found = cells_.find(key(index(pos.x()), index(pos.y())));
    if (found == cells_.end() ||
      found->second.hits < static_cast<uint32_t>(min_hits_))
    {
      continue;
    }
    map.at("elevation", *it) = found->second.height;
    ++filled;
  }

  if (filled == 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "%zu cells accumulated but none within the %.0f m window around the "
      "robot, or none with %d hits yet.", cells_.size(), map_length_, min_hits_);
    return;
  }

  RCLCPP_INFO_ONCE(
    get_logger(), "Publishing elevation_map: %zu of %.0f x %.0f cells observed.",
    filled, map_length_ / resolution_, map_length_ / resolution_);
  map.setTimestamp(now().nanoseconds());
  auto msg = grid_map::GridMapRosConverter::toMessage(map);
  map_pub_->publish(std::move(*msg));
}

}  // namespace gbplanner_elevation_map
