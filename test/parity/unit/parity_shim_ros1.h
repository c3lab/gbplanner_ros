#ifndef GBP_PARITY_SHIM_ROS1_H_
#define GBP_PARITY_SHIM_ROS1_H_

// ROS 1 (Noetic) half of the parity shim. Its ROS 2 twin is parity_shim_ros2.h;
// the two expose the identical `parity::Fixture` surface so that
// parity_body.cpp - the actual test - compiles unchanged against both.
//
// Only three things genuinely differ between the sides and all three live here:
//   * how a node comes into existence (ros::init + a live master vs rclcpp::init),
//   * how a parameter reaches the planner (global param server keyed with '/'
//     vs per-node parameter overrides keyed with '.'),
//   * the MapManagerVoxblox constructor signature.

#include <ros/ros.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gbplanner/rrg.h"
#include "map_manager/map_manager.h"
#include "map_manager/map_manager_voxblox_impl.h"
#include "planner_common/geofence_manager.h"
#include "planner_common/graph_manager.h"
#include "planner_common/params.h"
#include "planner_common/random_sampler.h"
#include "planner_common/trajectory.h"

namespace parity {

inline const char* side() { return "ros1"; }

// Node name, and therefore the private namespace every parameter below lands in.
inline const char* nodeName() { return "gbp_parity"; }

using MapManagerImpl =
    MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>;

class Fixture {
 public:
  Fixture(int argc, char** argv) {
    ros::init(argc, argv, nodeName(),
              ros::init_options::NoSigintHandler |
                  ros::init_options::AnonymousName);
  }

  ~Fixture() {
    // Deliberately leaked rather than destroyed: both own publishers and the
    // voxblox server, and tearing them down after the middleware has been shut
    // down is a crash on either side. The process is about to exit anyway.
    rrg_.release();
    map_.release();
    if (nh_) {
      nh_private_.reset();
      nh_.reset();
      ros::shutdown();
    }
  }

  // Canonical dotted parameter names are translated to this side's convention:
  // "SensorParams.OS064.max_range" -> "/<node>/SensorParams/OS064/max_range".
  template <typename T>
  void setParam(const std::string& dotted, const T& value) {
    ros::param::set(absolute(dotted), value);
  }
  void setParam(const std::string& dotted, const char* value) {
    ros::param::set(absolute(dotted), std::string(value));
  }

  // Brings the node handles up. Parameters must be set before this so that both
  // sides see the same "supplied at construction" picture.
  void realize() {
    nh_.reset(new ros::NodeHandle());
    nh_private_.reset(new ros::NodeHandle("~"));
  }

  // planner_common's loadParams takes the namespace only on this side.
  template <typename T>
  bool loadParams(T& target, const std::string& dotted_ns) {
    return target.loadParams(absolute(dotted_ns));
  }

  MapManager& map() {
    if (!map_) map_.reset(new MapManager(*nh_, *nh_private_));
    return *map_;
  }

  // Rrg's second constructor takes an externally owned map manager, which is
  // what lets the test fill a synthetic layer first and then plan over it.
  Rrg& rrg() {
    if (!rrg_) rrg_.reset(new Rrg(*nh_, *nh_private_, &map()));
    return *rrg_;
  }

 private:
  std::string absolute(const std::string& dotted) const {
    std::string out = ros::this_node::getName() + "/";
    for (char c : dotted) out += (c == '.') ? '/' : c;
    return out;
  }

  std::unique_ptr<ros::NodeHandle> nh_;
  std::unique_ptr<ros::NodeHandle> nh_private_;
  std::unique_ptr<MapManager> map_;
  std::unique_ptr<Rrg> rrg_;
};

}  // namespace parity

#endif  // GBP_PARITY_SHIM_ROS1_H_
