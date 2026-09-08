#ifndef GBPLANNER_GZ_CONTROL__UAV_PATH_FOLLOWER_NODE_H_
#define GBPLANNER_GZ_CONTROL__UAV_PATH_FOLLOWER_NODE_H_

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace gbplanner_gz_control
{

/// Flies the planner's path on a Gazebo Harmonic multicopter.
///
/// The ROS 1 simulation had two halves: a 158-line follower that walked the
/// path and published a setpoint on `command/pose`, and gz-sim's
/// MulticopterPositionControl, which turned that setpoint into rotor speeds.
/// Harmonic does not ship the second half -- it was an NTNU addition -- so the
/// position loop moved here. What is left in Gazebo is the upstream
/// MulticopterVelocityControl (a Lee velocity/attitude controller), and this
/// node feeds it `command/velocity`.
///
/// The loop is a saturated proportional outer loop, the standard cascade over a
/// velocity controller: v_cmd = clamp(kp * (p_target - p_now)). There is
/// deliberately no derivative term -- the Lee controller underneath is already
/// a damped velocity tracker, and differentiating position here would only add
/// a second, badly tuned, damping path.
class UAVPathFollowerNode : public rclcpp::Node
{
public:
  explicit UAVPathFollowerNode(const rclcpp::NodeOptions & options);

private:
  void pathCallback(const nav_msgs::msg::Path & path);
  void odometryCallback(const nav_msgs::msg::Odometry & odom);
  void controlStep();
  void publishEnable();

  bool isAtPose(const geometry_msgs::msg::Pose & target) const;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_pose_vis_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr enable_timer_;

  std::vector<geometry_msgs::msg::Pose> poses_;
  size_t current_pose_index_{0};

  geometry_msgs::msg::Pose current_pose_;
  bool have_odometry_{false};
  rclcpp::Time last_odometry_stamp_;

  std::string world_frame_;
  bool command_in_body_frame_{true};
  double trans_tolerance_{0.0};
  double rot_tolerance_{0.0};
  double kp_xy_{0.0};
  double kp_z_{0.0};
  double kp_yaw_{0.0};
  double v_max_xy_{0.0};
  double v_max_z_{0.0};
  double yaw_rate_max_{0.0};
  double odometry_timeout_{0.0};
};

}  // namespace gbplanner_gz_control

#endif  // GBPLANNER_GZ_CONTROL__UAV_PATH_FOLLOWER_NODE_H_
