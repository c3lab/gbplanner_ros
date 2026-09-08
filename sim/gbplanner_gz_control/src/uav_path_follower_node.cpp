#include "gbplanner_gz_control/uav_path_follower_node.h"

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

UAVPathFollowerNode::UAVPathFollowerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("uav_path_follower_node", options),
  last_odometry_stamp_(0, 0, RCL_ROS_TIME)
{
  world_frame_ = declare_parameter<std::string>("world_frame", "world");
  const std::string command_frame = declare_parameter<std::string>("command_frame", "body");
  const double control_rate = declare_parameter<double>("control_rate", 20.0);
  trans_tolerance_ = declare_parameter<double>("trans_tolerance", 0.4);
  rot_tolerance_ = declare_parameter<double>("rot_tolerance", 0.5);
  kp_xy_ = declare_parameter<double>("kp_xy", 1.0);
  kp_z_ = declare_parameter<double>("kp_z", 1.0);
  kp_yaw_ = declare_parameter<double>("kp_yaw", 1.0);
  v_max_xy_ = declare_parameter<double>("v_max_xy", 1.5);
  v_max_z_ = declare_parameter<double>("v_max_z", 1.0);
  yaw_rate_max_ = declare_parameter<double>("yaw_rate_max", 0.9);
  odometry_timeout_ = declare_parameter<double>("odometry_timeout", 1.0);
  const double enable_rate = declare_parameter<double>("enable_rate", 1.0);

  command_in_body_frame_ = (command_frame != "world");
  if (command_frame != "body" && command_frame != "world") {
    RCLCPP_WARN(
      get_logger(), "command_frame '%s' is neither 'body' nor 'world'; assuming 'body'.",
      command_frame.c_str());
  }

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("command/velocity", 1);
  cmd_pose_vis_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("command/pose_vis", 1);
  enable_pub_ = create_publisher<std_msgs::msg::Bool>("enable", 1);

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "command/path", 1, [this](const nav_msgs::msg::Path & msg) {pathCallback(msg);});
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odometry", 1, [this](const nav_msgs::msg::Odometry & msg) {odometryCallback(msg);});

  // A ROS-time timer, not a wall timer: under use_sim_time a real-time factor
  // below 1 would otherwise thin the control loop out to a fraction of its
  // nominal rate in simulated seconds, which is the rate the vehicle feels.
  control_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / control_rate),
    [this]() {controlStep();});

  // MulticopterVelocityControl ignores every twist until something publishes
  // true on its enable topic. Nothing else in the stack does, and the bridge
  // drops messages sent before Gazebo's subscriber exists, so republish it
  // rather than sending one and hoping.
  if (enable_rate > 0.0) {
    enable_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / enable_rate),
      [this]() {publishEnable();});
  }
}

void UAVPathFollowerNode::pathCallback(const nav_msgs::msg::Path & path)
{
  poses_.clear();
  poses_.reserve(path.poses.size());
  for (const auto & stamped : path.poses) {
    poses_.push_back(stamped.pose);
  }
  current_pose_index_ = 0;
  // Debug, not info. With pub_singple_wp set - which the shipped configs do -
  // the control interface republishes a one-pose carrot path every 50 ms, so an
  // info line here buries every other message in the launch at 20 Hz. The ROS 1
  // follower has the equivalent line commented out for the same reason.
  RCLCPP_DEBUG(get_logger(), "New path with %zu poses.", poses_.size());
}

void UAVPathFollowerNode::odometryCallback(const nav_msgs::msg::Odometry & odom)
{
  RCLCPP_INFO_ONCE(get_logger(), "Got first odometry message.");
  current_pose_ = odom.pose.pose;
  last_odometry_stamp_ = now();
  have_odometry_ = true;
}

bool UAVPathFollowerNode::isAtPose(const geometry_msgs::msg::Pose & target) const
{
  const double dx = current_pose_.position.x - target.position.x;
  const double dy = current_pose_.position.y - target.position.y;
  const double dz = current_pose_.position.z - target.position.z;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double dyaw = wrapPi(tf2::getYaw(current_pose_.orientation) -
    tf2::getYaw(target.orientation));
  return distance < trans_tolerance_ && std::abs(dyaw) < rot_tolerance_;
}

void UAVPathFollowerNode::controlStep()
{
  geometry_msgs::msg::Twist cmd;

  // Hover on stale or missing odometry. Publishing nothing is not the safe
  // option: the Lee controller holds its last command, so a silent follower
  // leaves the vehicle flying blind at whatever it was last told.
  const bool odometry_fresh = have_odometry_ &&
    (now() - last_odometry_stamp_).seconds() < odometry_timeout_;
  if (!odometry_fresh) {
    if (have_odometry_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Odometry is stale; commanding hover.");
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for odometry; commanding hover.");
    }
    cmd_vel_pub_->publish(cmd);
    return;
  }

  if (poses_.empty()) {
    cmd_vel_pub_->publish(cmd);
    return;
  }

  // Walk forward past every waypoint already reached, then station-keep on the
  // last one. Holding the final pose rather than falling back to a zero
  // velocity matters: zero velocity is not zero drift once the path ends.
  while (current_pose_index_ + 1 < poses_.size() && isAtPose(poses_[current_pose_index_])) {
    ++current_pose_index_;
  }
  const geometry_msgs::msg::Pose & target = poses_[current_pose_index_];

  double vx = kp_xy_ * (target.position.x - current_pose_.position.x);
  double vy = kp_xy_ * (target.position.y - current_pose_.position.y);
  double vz = clampAbs(kp_z_ * (target.position.z - current_pose_.position.z), v_max_z_);

  // Saturate the horizontal command as a vector, not per axis: clamping x and y
  // independently rotates the command away from the direction of the error.
  const double speed_xy = std::hypot(vx, vy);
  if (speed_xy > v_max_xy_ && speed_xy > 0.0) {
    const double scale = v_max_xy_ / speed_xy;
    vx *= scale;
    vy *= scale;
  }

  const double current_yaw = tf2::getYaw(current_pose_.orientation);
  const double yaw_error = wrapPi(tf2::getYaw(target.orientation) - current_yaw);

  if (command_in_body_frame_) {
    const double c = std::cos(current_yaw);
    const double s = std::sin(current_yaw);
    const double bx = c * vx + s * vy;
    const double by = -s * vx + c * vy;
    vx = bx;
    vy = by;
  }

  cmd.linear.x = vx;
  cmd.linear.y = vy;
  cmd.linear.z = vz;
  cmd.angular.z = clampAbs(kp_yaw_ * yaw_error, yaw_rate_max_);
  cmd_vel_pub_->publish(cmd);

  geometry_msgs::msg::PoseStamped vis;
  vis.header.frame_id = world_frame_;
  vis.header.stamp = now();
  vis.pose = target;
  cmd_pose_vis_pub_->publish(vis);
}

void UAVPathFollowerNode::publishEnable()
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  enable_pub_->publish(msg);
}

}  // namespace gbplanner_gz_control
