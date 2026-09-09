#include "gbplanner_gz_control/ugv_path_follower_node.h"

#include <algorithm>
#include <cmath>
#include <limits>

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
  last_odometry_stamp_(0, 0, RCL_ROS_TIME),
  stuck_reference_stamp_(0, 0, RCL_ROS_TIME),
  recovery_until_(0, 0, RCL_ROS_TIME)
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
  lookahead_distance_ = declare_parameter<double>("lookahead_distance", 1.2);
  stuck_timeout_ = declare_parameter<double>("stuck_timeout", 6.0);
  stuck_progress_ = declare_parameter<double>("stuck_progress", 0.15);
  recovery_duration_ = declare_parameter<double>("recovery_duration", 2.0);
  recovery_speed_ = declare_parameter<double>("recovery_speed", 0.4);
  recovery_yaw_rate_ = declare_parameter<double>("recovery_yaw_rate", 0.4);
  recovery_attempts_max_ = declare_parameter<int>("recovery_attempts_max", 3);
  world_z_limit_ = declare_parameter<double>("world_z_limit", 50.0);

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
  have_stuck_reference_ = false;
  in_recovery_ = false;
  recovery_attempts_ = 0;
  turn_direction_ = 1.0;
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
  if (!have_start_z_) {
    start_z_ = current_pose_.position.z;
    have_start_z_ = true;
  }
}

bool UGVPathFollowerNode::isAtPosition(const geometry_msgs::msg::Pose & target) const
{
  const double dx = current_pose_.position.x - target.position.x;
  const double dy = current_pose_.position.y - target.position.y;
  return std::hypot(dx, dy) < trans_tolerance_;
}

bool UGVPathFollowerNode::handleStuck(const rclcpp::Time & stamp, double commanded_speed)
{
  if (in_recovery_) {
    if (stamp < recovery_until_) {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = -recovery_speed_;
      // A little yaw with the reverse, so a second attempt does not retrace the
      // first one straight back into whatever stopped the robot.
      cmd.angular.z = (recovery_attempts_ % 2 == 0) ? recovery_yaw_rate_ : -recovery_yaw_rate_;
      cmd_vel_pub_->publish(cmd);
      return true;
    }
    in_recovery_ = false;
    have_stuck_reference_ = false;
  }

  // Only meaningful while the robot is being asked to drive. Turning on the
  // spot moves the position by nothing at all and is not being stuck.
  if (std::abs(commanded_speed) < 1e-3) {
    have_stuck_reference_ = false;
    return false;
  }

  if (!have_stuck_reference_) {
    stuck_reference_ = current_pose_.position;
    stuck_reference_stamp_ = stamp;
    have_stuck_reference_ = true;
    return false;
  }

  const double moved = std::hypot(
    current_pose_.position.x - stuck_reference_.x,
    current_pose_.position.y - stuck_reference_.y);
  if (moved > stuck_progress_) {
    stuck_reference_ = current_pose_.position;
    stuck_reference_stamp_ = stamp;
    return false;
  }

  if ((stamp - stuck_reference_stamp_).seconds() < stuck_timeout_) {
    return false;
  }

  // Commanded to drive, and has not covered stuck_progress_ in stuck_timeout_.
  if (recovery_attempts_ >= recovery_attempts_max_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Stuck after %d recovery attempts; dropping the path and holding. The "
      "control interface will not replan until it sees the path end reached, "
      "so this run needs a new trigger.",
      recovery_attempts_);
    poses_.clear();
    have_stuck_reference_ = false;
    geometry_msgs::msg::Twist cmd;
    cmd_vel_pub_->publish(cmd);
    return true;
  }

  ++recovery_attempts_;
  in_recovery_ = true;
  recovery_until_ = stamp + rclcpp::Duration::from_seconds(recovery_duration_);
  RCLCPP_WARN(
    get_logger(),
    "Commanded %.2f m/s but moved %.3f m in %.1f s; reversing for %.1f s "
    "(attempt %d of %d).",
    commanded_speed, moved, stuck_timeout_, recovery_duration_,
    recovery_attempts_, recovery_attempts_max_);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -recovery_speed_;
  cmd.angular.z = (recovery_attempts_ % 2 == 0) ? recovery_yaw_rate_ : -recovery_yaw_rate_;
  cmd_vel_pub_->publish(cmd);
  return true;
}

void UGVPathFollowerNode::advanceToNearest()
{
  // Nearest point on the path from here forward, and never a step back: a
  // waypoint the robot has already passed is done with, whether or not it ever
  // came within trans_tolerance of it.
  size_t best = current_pose_index_;
  double best_distance = std::numeric_limits<double>::max();
  for (size_t i = current_pose_index_; i < poses_.size(); ++i) {
    const double dx = poses_[i].position.x - current_pose_.position.x;
    const double dy = poses_[i].position.y - current_pose_.position.y;
    const double distance = std::hypot(dx, dy);
    if (distance < best_distance) {
      best_distance = distance;
      best = i;
    }
  }
  current_pose_index_ = best;
}

size_t UGVPathFollowerNode::lookaheadIndex() const
{
  for (size_t i = current_pose_index_; i < poses_.size(); ++i) {
    const double dx = poses_[i].position.x - current_pose_.position.x;
    const double dy = poses_[i].position.y - current_pose_.position.y;
    if (std::hypot(dx, dy) >= lookahead_distance_) {
      return i;
    }
  }
  // Nothing that far ahead left: the end of the path is closer than the
  // lookahead, so steer at the end and let the goal-yaw branch finish it.
  return poses_.size() - 1;
}

void UGVPathFollowerNode::controlStep()
{
  geometry_msgs::msg::Twist cmd;

  // Left the world. gz worlds built from a single mesh have nothing outside it
  // -- niosh_osrf is one model, niosh_seg01 -- so a robot that drives past the
  // end of the segment falls, and keeps falling. Measured on a live run:
  // z = -850 km and still accelerating, the planner still planning around it,
  // the control interface still waiting, and nothing anywhere saying so. Stop
  // driving and say it once, loudly, rather than steering a falling robot.
  if (have_start_z_ && std::abs(current_pose_.position.z - start_z_) > world_z_limit_) {
    if (!left_the_world_) {
      left_the_world_ = true;
      RCLCPP_ERROR(
        get_logger(),
        "Robot is %.0f m from the height it started at (z = %.1f). It has left "
        "the world -- these gz worlds are a single mesh with nothing outside "
        "it. Not commanding any further; this run is over.",
        current_pose_.position.z - start_z_, current_pose_.position.z);
    }
    cmd_vel_pub_->publish(cmd);
    return;
  }

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
    have_stuck_reference_ = false;
    cmd_vel_pub_->publish(cmd);
    return;
  }

  advanceToNearest();
  const geometry_msgs::msg::Pose & target = poses_[lookaheadIndex()];

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
    double heading_error = wrapPi(std::atan2(dy, dx) - current_yaw);
    // Away from the tie, remember which way round the robot is turning; at the
    // tie, use it. kAmbiguous is generous on purpose - a couple of degrees
    // either side of straight behind is already enough for noise to flip the
    // sign from one cycle to the next.
    constexpr double kAmbiguous = M_PI - 0.15;
    if (std::abs(heading_error) < kAmbiguous) {
      turn_direction_ = (heading_error >= 0.0) ? 1.0 : -1.0;
    } else {
      heading_error = turn_direction_ * std::abs(heading_error);
    }

    if (std::abs(heading_error) >= heading_align_threshold_) {
      // Too far off to drive out of: turn on the spot.
      cmd.angular.z = clampAbs(kp_yaw_ * heading_error, yaw_rate_max_);
    } else {
      // Pure pursuit proper. The arc through the lookahead point has curvature
      // 2 sin(alpha) / L, and driving that arc is what puts the robot ON the
      // point rather than near it.
      //
      // The clamp underneath is the part that matters. A differential drive
      // cannot turn tighter than v / yaw_rate_max, so a lookahead closer than
      // that radius is unreachable: steering at it saturates the yaw rate and
      // the robot circles it forever, driving and turning and arriving nowhere.
      // Measured before this: 24.3 m travelled and 2.0 m from the start, an
      // orbit of about 1.7 m radius, which is exactly v_max / yaw_rate_max.
      // Capping the speed to what the achievable arc allows trades forward
      // progress for actually closing on the point.
      const double curvature = 2.0 * std::sin(heading_error) / distance;
      double speed = std::min(kp_lin_ * distance, v_max_);
      if (std::abs(curvature) > 1e-6) {
        speed = std::min(speed, yaw_rate_max_ / std::abs(curvature));
      }
      cmd.linear.x = speed;
      cmd.angular.z = clampAbs(speed * curvature, yaw_rate_max_);
    }
  }

  if (handleStuck(now(), cmd.linear.x)) {
    return;
  }

  cmd_vel_pub_->publish(cmd);

  geometry_msgs::msg::PoseStamped vis;
  vis.header.frame_id = world_frame_;
  vis.header.stamp = now();
  vis.pose = target;
  cmd_pose_vis_pub_->publish(vis);
}

}  // namespace gbplanner_gz_control
