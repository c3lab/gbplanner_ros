#ifndef GBPLANNER_GZ_CONTROL__UGV_PATH_FOLLOWER_NODE_H_
#define GBPLANNER_GZ_CONTROL__UGV_PATH_FOLLOWER_NODE_H_

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gbplanner_gz_control
{

/// Drives the planner's path on a Gazebo Harmonic differential-drive robot.
///
/// The ROS 1 stack used smb_path_tracker's pure-pursuit node, which consumed
/// `gbplanner_path` and published a Twist at the SMB's velocity controller.
/// Nothing had to be replaced on the Gazebo side this time -- Harmonic ships
/// DiffDrive, and it takes exactly that Twist -- so this node is only the
/// tracker.
///
/// The difference from the UAV follower next to it is the constraint, not the
/// structure: a differential drive cannot move sideways, so the position error
/// cannot simply be turned into a velocity vector. What is controlled instead
/// is the bearing to the current waypoint, and forward speed is given up while
/// the robot is not pointing at it -- below `heading_align_threshold` the speed
/// is scaled by cos(heading error), above it the robot turns on the spot.
/// Driving at full speed with a large heading error is what produces the wide
/// arcs that walk a ground robot into the wall the planner routed it around.
///
/// Everything here is planar. The planner's z for a ground robot is a constant
/// the sampler was pinned to, not something the wheels can act on, so including
/// it in the waypoint-acceptance distance would only stop the robot advancing
/// whenever the two disagreed.
class UGVPathFollowerNode : public rclcpp::Node
{
public:
  explicit UGVPathFollowerNode(const rclcpp::NodeOptions & options);

private:
  void pathCallback(const nav_msgs::msg::Path & path);
  void odometryCallback(const nav_msgs::msg::Odometry & odom);
  void controlStep();

  bool isAtPosition(const geometry_msgs::msg::Pose & target) const;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_pose_vis_pub_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  std::vector<geometry_msgs::msg::Pose> poses_;
  size_t current_pose_index_{0};

  geometry_msgs::msg::Pose current_pose_;
  bool have_odometry_{false};
  rclcpp::Time last_odometry_stamp_;

  std::string world_frame_;
  double trans_tolerance_{0.0};
  double rot_tolerance_{0.0};
  double kp_lin_{0.0};
  double kp_yaw_{0.0};
  double v_max_{0.0};
  double yaw_rate_max_{0.0};
  double heading_align_threshold_{0.0};
  bool goal_yaw_enable_{false};
  double odometry_timeout_{0.0};
};

}  // namespace gbplanner_gz_control

#endif  // GBPLANNER_GZ_CONTROL__UGV_PATH_FOLLOWER_NODE_H_
