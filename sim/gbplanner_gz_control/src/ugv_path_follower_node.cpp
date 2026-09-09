#include "gbplanner_gz_control/ugv_path_follower_node.h"

#include <algorithm>
#include <cmath>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace gbplanner_gz_control
{
namespace
{

double wrapPi(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double clampAbs(double value, double limit)
{
  return std::max(-limit, std::min(limit, value));
}

}  // namespace

UGVPathFollowerNode::UGVPathFollowerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("ugv_path_follower_node", options),
  last_odometry_stamp_(0, 0, RCL_ROS_TIME)
{
  world_frame_ = declare_parameter<std::string>("world_frame", "world");
  const double control_rate = declare_parameter<double>("control_rate", 20.0);
  trans_tolerance_ = declare_parameter<double>("trans_tolerance", 0.5);
  rot_tolerance_ = declare_parameter<double>("rot_tolerance", 0.2);
  kp_lin_ = declare_parameter<double>("kp_lin", 1.0);
  kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
  v_max_ = declare_parameter<double>("v_max", 1.0);
  yaw_rate_max_ = declare_parameter<double>("yaw_rate_max", 0.9);
  heading_align_threshold_ =
    declare_parameter<double>("heading_align_threshold", 0.7);
  goal_yaw_enable_ = declare_parameter<bool>("goal_yaw_enable", true);
  odometry_timeout_ = declare_parameter<double>("odometry_timeout", 1.0);

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("command/velocity", 1);
  cmd_pose_vis_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("command/pose_vis", 1);

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "command/path", 1, [this](const nav_msgs::msg::Path & msg) {pathCallback(msg);});
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odometry", 1, [this](const nav_msgs::msg::Odometry & msg) {odometryCallback(msg);});

  // ROS time, not wall time: under use_sim_time a real-time factor below 1
  // would otherwise thin the loop out in the simulated seconds the robot
  // actually feels. Same reason as the UAV follower.
  control_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / control_rate),
    [this]() {controlStep();});
}

void UGVPathFollowerNode::pathCallback(const nav_msgs::msg::Path & path)
{
  poses_.clear();
  poses_.reserve(path.poses.size());
  for (const auto & stamped : path.poses) {
    poses_.push_back(stamped.pose);
  }
  current_pose_index_ = 0;
  // Debug, not info: with pub_singple_wp the control interface republishes a
  // one-pose carrot path every 50 ms.
  RCLCPP_DEBUG(get_logger(), "New path with %zu poses.", poses_.size());
}

void UGVPathFollowerNode::odometryCallback(const nav_msgs::msg::Odometry & odom)
{
  RCLCPP_INFO_ONCE(get_logger(), "Got first odometry message.");
  current_pose_ = odom.pose.pose;
  last_odometry_stamp_ = now();
  have_odometry_ = true;
}

bool UGVPathFollowerNode::isAtPosition(const geometry_msgs::msg::Pose & target) const
{
  const double dx = current_pose_.position.x - target.position.x;
  const double dy = current_pose_.position.y - target.position.y;
  return std::hypot(dx, dy) < trans_tolerance_;
}

void UGVPathFollowerNode::controlStep()
{
  geometry_msgs::msg::Twist cmd;

  // Stop on stale or missing odometry. The opposite of the UAV follower, which
  // commands a hover, and for the same underlying reason: a zero command is
  // what "stay put" means for the actuator underneath. DiffDrive holds the last
  // Twist it was given, so publishing nothing would leave the robot driving.
  const bool odometry_fresh = have_odometry_ &&
    (now() - last_odometry_stamp_).seconds() < odometry_timeout_;
  if (!odometry_fresh) {
    if (have_odometry_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Odometry is stale; stopping.");
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for odometry; stopping.");
    }
    cmd_vel_pub_->publish(cmd);
    return;
  }

  if (poses_.empty()) {
    cmd_vel_pub_->publish(cmd);
    return;
  }

  while (current_pose_index_ + 1 < poses_.size() &&
    isAtPosition(poses_[current_pose_index_]))
  {
    ++current_pose_index_;
  }
  const geometry_msgs::msg::Pose & target = poses_[current_pose_index_];

  const double current_yaw = tf2::getYaw(current_pose_.orientation);
  const double dx = target.position.x - current_pose_.position.x;
  const double dy = target.position.y - current_pose_.position.y;
  const double distance = std::hypot(dx, dy);

  if (distance < trans_tolerance_) {
    // The last waypoint, reached. Turn to the pose the planner asked for and
    // then hold: the planner's next path starts from this heading, and leaving
    // the robot pointing the wrong way costs it a turn-in-place at the start of
    // every leg.
    if (goal_yaw_enable_) {
      const double yaw_error = wrapPi(tf2::getYaw(target.orientation) - current_yaw);
      if (std::abs(yaw_error) > rot_tolerance_) {
        cmd.angular.z = clampAbs(kp_yaw_ * yaw_error, yaw_rate_max_);
      }
    }
  } else {
    const double heading_error = wrapPi(std::atan2(dy, dx) - current_yaw);
    cmd.angular.z = clampAbs(kp_yaw_ * heading_error, yaw_rate_max_);
    if (std::abs(heading_error) < heading_align_threshold_) {
      // cos() rather than a hard gate: the speed then falls off continuously as
      // the heading error grows, instead of stepping to zero at the threshold
      // and making the robot stutter along a curve.
      cmd.linear.x =
        std::min(kp_lin_ * distance, v_max_) * std::cos(heading_error);
    }
  }

  cmd_vel_pub_->publish(cmd);

  geometry_msgs::msg::PoseStamped vis;
  vis.header.frame_id = world_frame_;
  vis.header.stamp = now();
  vis.pose = target;
  cmd_pose_vis_pub_->publish(vis);
}

}  // namespace gbplanner_gz_control
