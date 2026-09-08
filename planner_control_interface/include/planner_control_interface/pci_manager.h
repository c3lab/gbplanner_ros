#ifndef PCI_MANAGER_H_
#define PCI_MANAGER_H_

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <planner_common/params.h>

// planner_common/params.h declares the same two verbosity gates at global
// scope, but at different levels (INFO / PLANNER_STATUS).  This package keeps
// the levels it shipped with, so drop the other definitions before redefining
// them; every use below resolves against explorer::Verbosity.
#undef global_verbosity
#undef param_verbosity

namespace explorer {

enum Verbosity { SILENT = 0, PLANNER_STATUS = 1, ERROR = 2, WARN = 3, INFO = 4, DEBUG = 5 };

typedef Eigen::Matrix<double, 5, 1> StateVec;

#define global_verbosity Verbosity::WARN
#define param_verbosity Verbosity::SILENT
class PCIManager {
 public:
  explicit PCIManager(rclcpp::Node* node)
      : node_(node),
        pci_status_(PCIStatus::kReady),
        force_stop_(false) {
    trajectory_vis_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "pci_command_trajectory_vis", rclcpp::QoS(10));
  }

  enum struct PCIStatus { kReady = 0, kRunning = 1, kError = 2 };

  enum struct RunModeType {
    kSim = 0,  // Run in simulation.
    kReal,     // Run with real robot.
  };

  enum struct ExecutionPathType {
    kLocalPath = 0,
    kHomingPath = 1,
    kGlobalPath = 2,
    kNarrowEnvPath =
        3,           // For special case, to slow down the copter in narrow env.
    kManualPath = 4,  // Manually set path.
    kAutoCustomPath = 5
  };

  enum struct RobotType { kAerial = 0, kGround };

  // ROS 2 parameters are node-local, so the node-name prefix that the ROS 1
  // version passed in as |ns| no longer exists.
  virtual bool loadParams() = 0;
  virtual bool initialize() = 0;
  virtual bool initMotion() = 0;

  virtual bool goToWaypoint(geometry_msgs::msg::Pose& pose) = 0;

  // Send a path to be executed by the robot, return a new path if the
  // PCI modified the original path based on robot's dynamics.
  // is_global_path: indicate this is local or global path.
  virtual bool executePath(
      const std::vector<geometry_msgs::msg::Pose>& path,
      std::vector<geometry_msgs::msg::Pose>& modified_path,
      ExecutionPathType path_type = ExecutionPathType::kLocalPath) = 0;

  // Set the current state of the robot based on odometry or pose.
  virtual void setState(const geometry_msgs::msg::Pose& pose) = 0;

  // Set max linear velocity allowed.
  virtual void setVelocity(double v) = 0;

  virtual void setCurrentVelocity(const geometry_msgs::msg::Vector3 &vel) = 0;

  // Check if we should trigger planner in advance.
  virtual bool planAhead() = 0;

  // Smoothly allocate yaw angle along the whole path to prevent sudden change
  // in yaw. Fix the heading from the root node, relax them over time.
  virtual void allocateYawAlongPath(
      std::vector<geometry_msgs::msg::Pose>& path) const = 0;
  // Fix the commanded heading to match the first segment.
  virtual void allocateYawAlongFistSegment(
      std::vector<geometry_msgs::msg::Pose>& path) const = 0;

  virtual double getVelocity(ExecutionPathType path_type) = 0;

  // Check if the PCI is ready to use.
  bool isReady() { return (pci_status_ == PCIStatus::kReady); }

  const PCIStatus& getStatus() const { return pci_status_; }
  void setStatus(const PCIStatus& status) { pci_status_ = status; }
  const geometry_msgs::msg::Pose& getState() { return current_pose_; }

  void stopPCI() { force_stop_ = true; }

 protected:
  rclcpp::Node* node_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      trajectory_vis_pub_;

  PCIStatus pci_status_;
  geometry_msgs::msg::Pose current_pose_;
  Eigen::Vector3d current_vel_;

  bool force_stop_;
};

}  // namespace explorer

#endif
