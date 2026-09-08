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

  // Two threads, two groups. Everything except odometry stays in the node's
  // default mutually-exclusive group, which keeps the planner's map reads and
  // the voxblox integration callbacks that MapManager registers on this same
  // node from ever overlapping - the property a single ros::spin() gave for
  // free, and which matters because the map is not thread-safe.
  //
  // Odometry alone sits in its own group. runGlobalPlanner waits three seconds
  // specifically so a fresher pose arrives, and under a single-threaded
  // executor nothing could deliver one: the wait was pure delay. setState only
  // takes state_mutex_ for the pose, so it now runs during that wait without
  // touching anything the planning callback holds.
  //
  // Two threads, not more: a third would have nothing to run, since every other
  // callback is in one mutually-exclusive group.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
