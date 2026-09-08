#include <string>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>

#include "gbplanner/gbplanner_ros.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // gflags aborts on any unrecognised "--" flag.  Under ROS 1 the extra tokens
  // on the command line were "name:=value" remappings, which gflags skipped;
  // ROS 2 passes "--ros-args --params-file ..." instead, so the ROS arguments
  // have to be stripped before gflags sees them.
  std::vector<std::string> non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char*> non_ros_argv;
  non_ros_argv.reserve(non_ros_args.size());
  for (std::string& arg : non_ros_args) non_ros_argv.push_back(arg.data());
  int non_ros_argc = static_cast<int>(non_ros_argv.size());
  char** non_ros_argv_data = non_ros_argv.data();
  google::ParseCommandLineFlags(&non_ros_argc, &non_ros_argv_data, false);

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("gbplanner_node");

  GbplannerRos planner(node.get());

  // SingleThreadedExecutor with everything in the node's default (mutually
  // exclusive) callback group: the ROS 1 node was a bare ros::spin(), every
  // graph/map/state mutation here is unguarded, and the voxblox server that
  // MapManager embeds registers its own callbacks on this same node, so any
  // multi-threaded executor would let map integration run concurrently with the
  // planner's map reads.  No callback blocks on another callback -- the only
  // two synchronous service calls in the ROS 1 code (land_srv) discarded their
  // response and are now fire-and-forget.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
