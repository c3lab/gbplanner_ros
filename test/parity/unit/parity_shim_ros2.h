#ifndef GBP_PARITY_SHIM_ROS2_H_
#define GBP_PARITY_SHIM_ROS2_H_

// ROS 2 (Jazzy) half of the parity shim. See parity_shim_ros1.h for the
// contract; the two files are the only place the suite is allowed to know
// which middleware it is running on.

#include <rclcpp/rclcpp.hpp>

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

inline const char* side() { return "ros2"; }

inline const char* nodeName() { return "gbp_parity"; }

using MapManagerImpl =
    MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>;

class Fixture {
 public:
  Fixture(int argc, char** argv) { rclcpp::init(argc, argv); }

  ~Fixture() {
    // Deliberately leaked rather than destroyed: both own publishers and the
    // voxblox server, and tearing them down after the middleware has been shut
    // down is a crash on either side. The process is about to exit anyway.
    rrg_.release();
    map_.release();
    node_.reset();
    rclcpp::shutdown();
  }

  // rclcpp can only be told about a parameter the node did not declare through
  // a construction-time override, so everything is buffered until realize().
  template <typename T>
  void setParam(const std::string& dotted, const T& value) {
    overrides_.emplace_back(dotted, value);
  }
  void setParam(const std::string& dotted, const char* value) {
    overrides_.emplace_back(dotted, std::string(value));
  }

  void realize() {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides_);
    // planner_common declares every key it reads on demand, and voxblox_ros
    // declares its own, so automatic declaration would only mask a key one side
    // reads and the other does not.
    options.automatically_declare_parameters_from_overrides(false);
    node_ = std::make_shared<rclcpp::Node>(nodeName(), options);
  }

  // planner_common's loadParams takes the node as well on this side.
  template <typename T>
  bool loadParams(T& target, const std::string& dotted_ns) {
    return target.loadParams(node_.get(), dotted_ns);
  }

  MapManager& map() {
    if (!map_) map_.reset(new MapManager(node_.get()));
    return *map_;
  }

  // Rrg's second constructor takes an externally owned map manager, which is
  // what lets the test fill a synthetic layer first and then plan over it.
  Rrg& rrg() {
    if (!rrg_) rrg_.reset(new Rrg(node_.get(), &map()));
    return *rrg_;
  }

 private:
  std::vector<rclcpp::Parameter> overrides_;
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<MapManager> map_;
  std::unique_ptr<Rrg> rrg_;
};

}  // namespace parity

#endif  // GBP_PARITY_SHIM_ROS2_H_
