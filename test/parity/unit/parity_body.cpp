// Suite B - the shared parity test body.
//
// This file is compiled twice from the same bytes: once against the ROS 1
// Noetic workspace and once against the ROS 2 Jazzy one. It never mentions ros/
// or rclcpp/; everything middleware-shaped is reached through parity::Fixture,
// which parity_shim_ros1.h and parity_shim_ros2.h implement identically.
//
// Every observable the two sides must agree on is printed as one
// `key = value` line; test/parity/compare_parity.py diffs the two outputs.
// Nothing here asserts on its own - a single-sided run has no notion of "pass".
// The only self-checks in this file are brute-force cross-checks (kd-tree
// nearest neighbour against a linear scan, for instance), and those are emitted
// as numbers too so that a comparator failure can always be attributed.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <boost/version.hpp>
#include <kdtree/kdtree.h>
#include <pcl/pcl_config.h>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/integrator/tsdf_integrator.h>

#include "parity_emit.h"
#include "parity_shim.h"

namespace {

using parity::emit_b;
using parity::emit_d;
using parity::emit_darray;
using parity::emit_i;
using parity::emit_iarray;
using parity::emit_s;

// ---------------------------------------------------------------------------
// Deterministic pseudo-randomness.
//
// std::mt19937 is bit-specified by the standard, but the *distributions* are
// not, so a fixture that used uniform_real_distribution would be comparing
// libstdc++ 9 against libstdc++ 13 rather than comparing gbplanner. This LCG is
// pure integer arithmetic with an exactly representable scaling, so every
// fixture below is the same sequence of doubles on both sides by construction.
// (The RNG *inside* the planner is still exercised - see runRandomSampler.)
// ---------------------------------------------------------------------------
class Lcg {
 public:
  explicit Lcg(std::uint64_t seed) : s_(seed) {}
  std::uint32_t next() {
    s_ = s_ * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<std::uint32_t>(s_ >> 33);  // 31 usable bits
  }
  double uniform(double lo, double hi) {
    return lo + (hi - lo) * (static_cast<double>(next()) * (1.0 / 2147483648.0));
  }

 private:
  std::uint64_t s_;
};

std::string vecKey(const std::string& base, int i) {
  return base + "[" + std::to_string(i) + "]";
}

void emitVec3(const std::string& key, const Eigen::Vector3d& v) {
  emit_darray(key, {v.x(), v.y(), v.z()});
}

// ---------------------------------------------------------------------------
// A. kdtree - kdtree/src/kdtree.c, plain C, no ROS, no floating-point tuning.
// ---------------------------------------------------------------------------
void runKdtree() {
  const int kNumPoints = 2000;
  Lcg rng(0x5eed1234u);
  std::vector<Eigen::Vector3d> pts(kNumPoints);
  for (int i = 0; i < kNumPoints; ++i) {
    pts[i] = Eigen::Vector3d(rng.uniform(-20.0, 20.0), rng.uniform(-20.0, 20.0),
                             rng.uniform(-20.0, 20.0));
  }

  kdtree* tree = kd_create(3);
  std::vector<int> payload(kNumPoints);
  for (int i = 0; i < kNumPoints; ++i) {
    payload[i] = i;
    kd_insert3(tree, pts[i].x(), pts[i].y(), pts[i].z(), &payload[i]);
  }

  // 200 nearest-neighbour queries, each cross-checked against a linear scan.
  const int kNumQueries = 200;
  Lcg qrng(0xc0ffeeu);
  std::vector<long long> nn_idx;
  std::vector<double> nn_dist;
  long long brute_force_disagreements = 0;
  for (int q = 0; q < kNumQueries; ++q) {
    Eigen::Vector3d p(qrng.uniform(-22.0, 22.0), qrng.uniform(-22.0, 22.0),
                      qrng.uniform(-22.0, 22.0));
    kdres* res = kd_nearest3(tree, p.x(), p.y(), p.z());
    int idx = -1;
    double d = -1.0;
    if (res && kd_res_size(res) > 0) {
      idx = *reinterpret_cast<int*>(kd_res_item_data(res));
      d = (pts[idx] - p).norm();
    }
    if (res) kd_res_free(res);

    int best = -1;
    double best_d = 0.0;
    for (int i = 0; i < kNumPoints; ++i) {
      const double di = (pts[i] - p).squaredNorm();
      if (best < 0 || di < best_d) {
        best = i;
        best_d = di;
      }
    }
    if (best != idx) ++brute_force_disagreements;
    nn_idx.push_back(idx);
    nn_dist.push_back(d);
  }
  emit_iarray("kdtree.nearest.index", nn_idx);
  emit_darray("kdtree.nearest.distance", nn_dist);
  emit_i("kdtree.nearest.brute_force_disagreements", brute_force_disagreements);

  // Range queries. 4.0 is planning_params_.nearest_range, the radius the RRG
  // actually asks for when it wires a new sample into the graph.
  const double radii[] = {0.5, 2.0, 4.0, 8.0};
  for (int r = 0; r < 4; ++r) {
    Lcg rrng(0xbeef0000u + r);
    std::vector<long long> sizes;
    std::vector<long long> checksums;
    long long set_disagreements = 0;
    long long iteration_mismatch = 0;
    for (int q = 0; q < 50; ++q) {
      Eigen::Vector3d p(rrng.uniform(-20.0, 20.0), rrng.uniform(-20.0, 20.0),
                        rrng.uniform(-20.0, 20.0));
      kdres* res =
          kd_nearest_range3(tree, p.x(), p.y(), p.z(), radii[r]);
      const int n = res ? kd_res_size(res) : 0;
      std::set<int> got;
      int walked = 0;
      if (res) {
        for (int i = 0; i < n; ++i) {
          got.insert(*reinterpret_cast<int*>(kd_res_item_data(res)));
          ++walked;
          if (kd_res_next(res) <= 0) break;
        }
        kd_res_free(res);
      }
      if (walked != n) ++iteration_mismatch;

      std::set<int> expected;
      for (int i = 0; i < kNumPoints; ++i) {
        if ((pts[i] - p).norm() <= radii[r]) expected.insert(i);
      }
      if (expected != got) ++set_disagreements;

      // The traversal order of kd_res_next is an implementation detail, so the
      // result is compared as a set: sum of ids is order-independent.
      long long sum = 0;
      for (int id : got) sum += id;
      sizes.push_back(n);
      checksums.push_back(sum);
    }
    const std::string base = "kdtree.range[" + std::to_string(r) + "]";
    emit_d(base + ".radius", radii[r]);
    emit_iarray(base + ".size", sizes);
    emit_iarray(base + ".id_sum", checksums);
    emit_i(base + ".brute_force_disagreements", set_disagreements);
    emit_i(base + ".size_vs_iteration_mismatch", iteration_mismatch);
  }

  // kd_clear must leave a tree that behaves like a fresh one.
  kd_clear(tree);
  for (int i = 0; i < kNumPoints / 2; ++i) {
    kd_insert3(tree, pts[i].x(), pts[i].y(), pts[i].z(), &payload[i]);
  }
  std::vector<long long> reuse_idx;
  Lcg qrng2(0xc0ffeeu);
  for (int q = 0; q < 50; ++q) {
    Eigen::Vector3d p(qrng2.uniform(-22.0, 22.0), qrng2.uniform(-22.0, 22.0),
                      qrng2.uniform(-22.0, 22.0));
    kdres* res = kd_nearest3(tree, p.x(), p.y(), p.z());
    reuse_idx.push_back(res && kd_res_size(res) > 0
                            ? *reinterpret_cast<int*>(kd_res_item_data(res))
                            : -1);
    if (res) kd_res_free(res);
  }
  emit_iarray("kdtree.clear_reuse.index", reuse_idx);
  kd_free(tree);
}

// ---------------------------------------------------------------------------
// B. SensorParamsBase - planner_common/src/params.cpp
//
// The parameters come through the real loadParams path on both sides, which is
// the point: params.cpp was rewritten line by line for rclcpp, so the number
// that matters is not only "does the frustum maths agree" but "did the rewrite
// deliver the same fields to it".
// ---------------------------------------------------------------------------
// The four real sensors of the cave configuration plus one pinhole camera.
// The camera is not in that config, but SensorParamsBase::getFrustumEdges and
// the normal-vector branch of isInsideFOV are only ever initialised on the
// kCamera path, so without it those two functions would go untested.
const char* kSensorNames[] = {"OS064", "CamF", "Cam", "CamExp", "CamPinhole"};

void declareSensorParams(parity::Fixture& fx) {
  const std::string ns = "SensorParams";
  fx.setParam(ns + ".sensor_list", std::vector<std::string>{"OS064", "CamF",
                                                            "Cam", "CamExp",
                                                            "CamPinhole"});

  // Exactly the four sensors of gbplanner/config/uav/gz/cave_exploration.
  fx.setParam(ns + ".OS064.type", "kLidar");
  fx.setParam(ns + ".OS064.max_range", 20.0);
  fx.setParam(ns + ".OS064.center_offset", std::vector<double>{0.0, 0.0, 0.0});
  fx.setParam(ns + ".OS064.rotations", std::vector<double>{0.0, 0.0, 0.0});
  fx.setParam(ns + ".OS064.fov",
              std::vector<double>{2.0 * M_PI, 90.0 * M_PI / 180.0});
  fx.setParam(ns + ".OS064.resolution",
              std::vector<double>{10.0 * M_PI / 180.0, 10.0 * M_PI / 180.0});
  fx.setParam(ns + ".OS064.frontier_percentage_threshold", 0.01);

  fx.setParam(ns + ".CamF.type", "kLidar");
  fx.setParam(ns + ".CamF.max_range", 3.5);
  fx.setParam(ns + ".CamF.center_offset", std::vector<double>{0.0, 0.0, 0.0});
  fx.setParam(ns + ".CamF.rotations", std::vector<double>{0.0, 0.0, 0.0});
  fx.setParam(ns + ".CamF.fov",
              std::vector<double>{100.0 * M_PI / 180.0, 60.0 * M_PI / 180.0});
  fx.setParam(ns + ".CamF.resolution",
              std::vector<double>{4.0 * M_PI / 180.0, 4.0 * M_PI / 180.0});
  fx.setParam(ns + ".CamF.frontier_percentage_threshold", 0.01);

  fx.setParam(ns + ".Cam.type", "kLidar");
  fx.setParam(ns + ".Cam.CameraType", "kRotating");
  fx.setParam(ns + ".Cam.rot_lims",
              std::vector<double>{-80.0 * M_PI / 180.0, 80.0 * M_PI / 180.0});
  fx.setParam(ns + ".Cam.max_range", 2.0);
  fx.setParam(ns + ".Cam.min_range", 0.25);
  fx.setParam(ns + ".Cam.center_offset", std::vector<double>{0.0, 0.0, 0.0});
  fx.setParam(ns + ".Cam.rotations", std::vector<double>{0.0, 0.0, 0.0});
  fx.setParam(ns + ".Cam.fov",
              std::vector<double>{80.0 * M_PI / 180.0, 60.0 * M_PI / 180.0});
  fx.setParam(ns + ".Cam.resolution",
              std::vector<double>{5.0 * M_PI / 180.0, 5.0 * M_PI / 180.0});
  fx.setParam(ns + ".Cam.frontier_percentage_threshold", 0.06);

  fx.setParam(ns + ".CamExp.type", "kLidar");
  fx.setParam(ns + ".CamExp.CameraType", "kRotating");
  fx.setParam(ns + ".CamExp.rot_lims", std::vector<double>{-1.57, 1.57});
  fx.setParam(ns + ".CamExp.max_range", 5.0);
  fx.setParam(ns + ".CamExp.min_range", 0.25);
  fx.setParam(ns + ".CamExp.center_offset", std::vector<double>{0.0, 0.0, 0.0});
  fx.setParam(ns + ".CamExp.rotations", std::vector<double>{0.0, 0.0, 0.0});
  fx.setParam(ns + ".CamExp.fov",
              std::vector<double>{84.0 * M_PI / 180.0, 64.0 * M_PI / 180.0});
  fx.setParam(ns + ".CamExp.resolution",
              std::vector<double>{7.0 * M_PI / 180.0, 7.0 * M_PI / 180.0});
  fx.setParam(ns + ".CamExp.frontier_percentage_threshold", 0.06);

  fx.setParam(ns + ".CamPinhole.type", "kCamera");
  fx.setParam(ns + ".CamPinhole.CameraType", "kFixed");
  fx.setParam(ns + ".CamPinhole.max_range", 6.0);
  fx.setParam(ns + ".CamPinhole.min_range", 0.2);
  fx.setParam(ns + ".CamPinhole.center_offset",
              std::vector<double>{0.1, 0.0, -0.05});
  fx.setParam(ns + ".CamPinhole.rotations",
              std::vector<double>{0.2, -0.1, 0.0});
  fx.setParam(ns + ".CamPinhole.fov",
              std::vector<double>{90.0 * M_PI / 180.0, 60.0 * M_PI / 180.0});
  fx.setParam(ns + ".CamPinhole.resolution",
              std::vector<double>{5.0 * M_PI / 180.0, 5.0 * M_PI / 180.0});
  fx.setParam(ns + ".CamPinhole.frontier_percentage_threshold", 0.05);
}

void runSensorParams(parity::Fixture& fx, SensorParams& sensors) {
  const bool ok = fx.loadParams(sensors, "SensorParams");
  emit_b("sensor.load_ok", ok);
  emit_i("sensor.list_size", static_cast<long long>(sensors.sensor_list.size()));
  for (size_t i = 0; i < sensors.sensor_list.size(); ++i) {
    emit_s(vecKey("sensor.list", static_cast<int>(i)), sensors.sensor_list[i]);
  }
  if (!ok) return;

  // Eight poses covering the yaw/pitch combinations the planner samples.
  std::vector<StateVec> poses;
  const double yaws[] = {0.0, M_PI / 4.0, -M_PI / 4.0, M_PI / 2.0, M_PI};
  const double pitches[] = {0.0, 0.3, -0.3};
  for (double yaw : yaws) {
    for (double pitch : pitches) {
      StateVec s;
      s << 1.5, -2.25, 0.75, yaw, pitch;
      poses.push_back(s);
    }
  }

  for (const char* name : kSensorNames) {
    auto it = sensors.sensor.find(name);
    if (it == sensors.sensor.end()) {
      emit_i(std::string("sensor.") + name + ".present", 0);
      continue;
    }
    SensorParamsBase& sp = it->second;
    const std::string base = std::string("sensor.") + name;
    emit_i(base + ".present", 1);
    emit_i(base + ".type", static_cast<long long>(sp.type));
    emit_i(base + ".camera_type", static_cast<long long>(sp.camera_type));
    emit_d(base + ".min_range", sp.min_range);
    emit_d(base + ".max_range", sp.max_range);
    emitVec3(base + ".center_offset", sp.center_offset);
    emitVec3(base + ".rotations", sp.rotations);
    emit_darray(base + ".fov", {sp.fov[0], sp.fov[1]});
    emit_darray(base + ".resolution", {sp.resolution[0], sp.resolution[1]});
    emit_darray(base + ".rot_lims", {sp.rot_lims[0], sp.rot_lims[1]});
    emit_i(base + ".width", sp.width);
    emit_i(base + ".height", sp.height);

    // The endpoint count is the canary for the accumulated-in-double loop bound
    // in updateFrustumEndpoints(): one ULP of drift changes it by a whole ray.
    std::vector<Eigen::Vector3d> ep;
    StateVec zero;
    zero << 0.0, 0.0, 0.0, 0.0, 0.0;
    sp.getFrustumEndpoints(zero, ep);
    emit_i(base + ".frustum_endpoint_count",
           static_cast<long long>(ep.size()));

    for (size_t p = 0; p < poses.size(); ++p) {
      std::vector<Eigen::Vector3d> eps;
      sp.getFrustumEndpoints(poses[p], eps);
      // Full dumps would be ~100k numbers per sensor; the sums and the extrema
      // localise a disagreement to a pose without that cost, and the sampled
      // endpoints below pin down actual coordinates.
      double sx = 0.0, sy = 0.0, sz = 0.0;
      for (const auto& e : eps) {
        sx += e.x();
        sy += e.y();
        sz += e.z();
      }
      const std::string pb = base + ".pose[" + std::to_string(p) + "]";
      emit_i(pb + ".count", static_cast<long long>(eps.size()));
      emit_darray(pb + ".sum", {sx, sy, sz});
      std::vector<double> sampled;
      for (size_t k = 0; k < eps.size(); k += std::max<size_t>(1, eps.size() / 12)) {
        sampled.push_back(eps[k].x());
        sampled.push_back(eps[k].y());
        sampled.push_back(eps[k].z());
      }
      emit_darray(pb + ".sampled", sampled);

      // getFrustumEdges reads edge_points_B, which loadParams only ever fills
      // in on the kCamera path (params.cpp:191). Calling it on a kLidar sensor
      // reads an uninitialised Eigen member, so it is deliberately not compared
      // there - see README, "Findings".
      if (sp.type == SensorType::kCamera) {
        std::vector<Eigen::Vector3d> edges;
        sp.getFrustumEdges(poses[p], edges);
        std::vector<double> edge_vals;
        for (const auto& e : edges) {
          edge_vals.push_back(e.x());
          edge_vals.push_back(e.y());
          edge_vals.push_back(e.z());
        }
        emit_darray(pb + ".edges", edge_vals);
      }
    }

    // isInsideFOV over a fixed cloud, emitted as one character per point.
    Lcg frng(0x1515u);
    std::string codes;
    codes.reserve(3000);
    StateVec st;
    st << 0.0, 0.0, 0.0, 0.4, 0.0;
    for (int i = 0; i < 3000; ++i) {
      Eigen::Vector3d p(frng.uniform(-25.0, 25.0), frng.uniform(-25.0, 25.0),
                        frng.uniform(-25.0, 25.0));
      codes.push_back(sp.isInsideFOV(st, p) ? '1' : '0');
    }
    emit_s(base + ".is_inside_fov", codes);

    // isFrontier flips at frontier_percentage_threshold * num_voxels_full_fov,
    // both of which are private and only ever set by loadParams - so this
    // doubles as a check that the parameter actually arrived.
    std::string frontier;
    for (int i = 0; i <= 200; ++i) {
      frontier.push_back(sp.isFrontier(i * 0.5) ? '1' : '0');
    }
    emit_s(base + ".is_frontier_sweep", frontier);
  }
}

// ---------------------------------------------------------------------------
// C. RobotParams / BoundedSpaceParams
// ---------------------------------------------------------------------------
void declareSpaceParams(parity::Fixture& fx) {
  fx.setParam("RobotParams.type", "kAerialRobot");
  fx.setParam("RobotParams.size", std::vector<double>{0.4, 0.4, 0.4});
  fx.setParam("RobotParams.size_extension_min",
              std::vector<double>{0.0, 0.0, 0.0});
  fx.setParam("RobotParams.size_extension", std::vector<double>{0.2, 0.3, 0.1});
  fx.setParam("RobotParams.center_offset", std::vector<double>{-0.0, 0.0, 0.0});
  fx.setParam("RobotParams.relax_ratio", 0.5);
  fx.setParam("RobotParams.bound_mode", "kExtendedBound");
  fx.setParam("RobotParams.safety_extension",
              std::vector<double>{0.5, 0.5, 0.5});

  fx.setParam("BoundedSpaceParams.Global.type", "kCuboid");
  fx.setParam("BoundedSpaceParams.Global.min_val",
              std::vector<double>{-60.0, -1000.0, -10.5});
  fx.setParam("BoundedSpaceParams.Global.max_val",
              std::vector<double>{1000.0, 1000.0, 30.0});

  fx.setParam("BoundedSpaceParams.Local.type", "kCuboid");
  // The cave config uses +/-15 here; the synthetic corridor is 12 m long, so a
  // box that size would spend the planner's whole sampling budget outside the
  // map. Both sides get the same box, which is all that matters.
  fx.setParam("BoundedSpaceParams.Local.min_val",
              std::vector<double>{-8.0, -8.0, -2.0});
  fx.setParam("BoundedSpaceParams.Local.max_val",
              std::vector<double>{8.0, 8.0, 2.0});
  fx.setParam("BoundedSpaceParams.Local.min_extension",
              std::vector<double>{-20.0, -20.0, -20.0});
  fx.setParam("BoundedSpaceParams.Local.max_extension",
              std::vector<double>{20.0, 20.0, 20.0});

  fx.setParam("BoundedSpaceParams.Ball.type", "kSphere");
  fx.setParam("BoundedSpaceParams.Ball.radius", 7.5);
  fx.setParam("BoundedSpaceParams.Ball.radius_extension", 2.5);
}

void runSpaceParams(parity::Fixture& fx, BoundedSpaceParams& global_space,
                    BoundedSpaceParams& local_space) {
  RobotParams robot;
  const bool rok = fx.loadParams(robot, "RobotParams");
  emit_b("robot.load_ok", rok);
  emit_i("robot.type", static_cast<long long>(robot.type));
  emitVec3("robot.size", robot.size);
  emitVec3("robot.size_extension_min", robot.size_extension_min);
  emitVec3("robot.size_extension", robot.size_extension);
  emitVec3("robot.center_offset", robot.center_offset);
  emitVec3("robot.safety_extension", robot.safety_extension);
  emit_darray("robot.footprint", {robot.footprint[0], robot.footprint[1]});
  emit_d("robot.relax_ratio", robot.relax_ratio);
  emit_i("robot.bound_mode", static_cast<long long>(robot.bound_mode));

  const BoundModeType modes[] = {
      BoundModeType::kExtendedBound, BoundModeType::kRelaxedBound,
      BoundModeType::kMinBound, BoundModeType::kExactBound,
      BoundModeType::kNoBound};
  for (int m = 0; m < 5; ++m) {
    robot.setBoundMode(modes[m]);
    Eigen::Vector3d psize;
    robot.getPlanningSize(psize);
    emitVec3("robot.planning_size[" + std::to_string(m) + "]", psize);
  }

  const bool gok = fx.loadParams(global_space, "BoundedSpaceParams.Global");
  const bool lok = fx.loadParams(local_space, "BoundedSpaceParams.Local");
  BoundedSpaceParams ball;
  const bool bok = fx.loadParams(ball, "BoundedSpaceParams.Ball");
  emit_b("space.global.load_ok", gok);
  emit_b("space.local.load_ok", lok);
  emit_b("space.ball.load_ok", bok);
  emit_i("space.local.type", static_cast<long long>(local_space.type));
  emitVec3("space.local.min_val", local_space.min_val);
  emitVec3("space.local.max_val", local_space.max_val);
  emitVec3("space.local.min_extension", local_space.min_extension);
  emitVec3("space.local.max_extension", local_space.max_extension);
  emit_i("space.ball.type", static_cast<long long>(ball.type));
  emit_d("space.ball.radius", ball.radius);
  emit_d("space.ball.radius_extension", ball.radius_extension);

  // isInsideSpace over a grid, with and without the extension, and after a
  // rotation. Boundary points are included on purpose: min_val and max_val
  // themselves must classify the same way on both sides.
  struct Case {
    const char* name;
    BoundedSpaceParams* sp;
    bool extension;
    double yaw;
  };
  BoundedSpaceParams* spaces[] = {&local_space, &ball};
  const char* space_names[] = {"local", "ball"};
  for (int si = 0; si < 2; ++si) {
    for (int ext = 0; ext < 2; ++ext) {
      for (int ri = 0; ri < 3; ++ri) {
        const double yaw = (ri == 0) ? 0.0 : (ri == 1 ? M_PI / 6.0 : M_PI / 2.0);
        Eigen::Vector3d rot(yaw, 0.0, 0.0);
        spaces[si]->setRotation(rot);
        Eigen::Vector3d centre(1.0, -2.0, 0.5);
        spaces[si]->setCenter(centre, ext != 0);
        std::string codes;
        for (int ix = -12; ix <= 12; ++ix) {
          for (int iy = -12; iy <= 12; ++iy) {
            for (int iz = -6; iz <= 6; ++iz) {
              Eigen::Vector3d p(ix * 1.5, iy * 1.5, iz * 1.5);
              codes.push_back(spaces[si]->isInsideSpace(p) ? '1' : '0');
            }
          }
        }
        // The declared bounds themselves, where an inclusive/exclusive
        // comparison would diverge. Only meaningful for a cuboid: loadParams
        // leaves min_val/max_val untouched on the kSphere path
        // (params.cpp:692-780), so probing them there reads uninitialised
        // memory - see README, "Findings".
        if (spaces[si]->type == BoundedSpaceType::kCuboid) {
          for (int c = 0; c < 2; ++c) {
            Eigen::Vector3d p =
                (c == 0) ? spaces[si]->min_val : spaces[si]->max_val;
            codes.push_back(spaces[si]->isInsideSpace(p) ? '1' : '0');
          }
        }
        const std::string key = std::string("space.inside.") + space_names[si] +
                                ".ext" + std::to_string(ext) + ".rot" +
                                std::to_string(ri);
        emit_s(key, codes);
        emitVec3(key + ".center", spaces[si]->getCenter());
        Eigen::Matrix3d r = spaces[si]->getRotationMatrix();
        emit_darray(key + ".rotation",
                    {r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2),
                     r(2, 0), r(2, 1), r(2, 2)});
      }
    }
  }
  // Leave the spaces in the state the sampler expects.
  Eigen::Vector3d no_rot(0.0, 0.0, 0.0);
  global_space.setRotation(no_rot);
  local_space.setRotation(no_rot);
  Eigen::Vector3d origin(0.0, 0.0, 0.0);
  global_space.setCenter(origin, false);
  local_space.setCenter(origin, false);
}

// ---------------------------------------------------------------------------
// D. Trajectory - planner_common/src/trajectory.cpp, pure Eigen maths.
// ---------------------------------------------------------------------------
Trajectory::PathType makeLine() {
  Trajectory::PathType p;
  for (int i = 0; i <= 10; ++i) {
    p.emplace_back(i * 0.73, i * -0.21, i * 0.11);
  }
  return p;
}

Trajectory::PathType makeBend() {
  Trajectory::PathType p;
  for (int i = 0; i <= 6; ++i) p.emplace_back(i * 1.0, 0.0, 0.0);
  for (int i = 1; i <= 6; ++i) p.emplace_back(6.0, i * 1.0, i * 0.25);
  return p;
}

Trajectory::PathType makeHelix() {
  Trajectory::PathType p;
  for (int i = 0; i <= 40; ++i) {
    const double t = i * 0.25;
    p.emplace_back(3.0 * std::cos(t), 3.0 * std::sin(t), 0.2 * t);
  }
  return p;
}

void runTrajectory() {
  std::vector<Trajectory::PathType> paths = {makeLine(), makeBend(),
                                             makeHelix()};
  const char* names[] = {"line", "bend", "helix"};

  for (int i = 0; i < 3; ++i) {
    const std::string base = std::string("traj.") + names[i];
    emit_d(base + ".length", Trajectory::getPathLength(paths[i]));
    emit_d(base + ".direction", Trajectory::estimateDirectionFromPath(paths[i]));

    Trajectory::PathType intp;
    Trajectory::interpolatePath(paths[i], 0.2, intp);
    emit_i(base + ".interp.count", static_cast<long long>(intp.size()));
    std::vector<double> flat;
    for (const auto& p : intp) {
      flat.push_back(p.x());
      flat.push_back(p.y());
      flat.push_back(p.z());
    }
    emit_darray(base + ".interp.points", flat);

    const double total = Trajectory::getPathLength(paths[i]);
    const double lens[] = {0.5, 2.0, 5.0, total, total + 1.0};
    for (int l = 0; l < 5; ++l) {
      Trajectory::PathType cut = paths[i];
      Trajectory::shortenPath(cut, lens[l]);
      std::vector<double> cflat;
      for (const auto& p : cut) {
        cflat.push_back(p.x());
        cflat.push_back(p.y());
        cflat.push_back(p.z());
      }
      emit_darray(base + ".shorten[" + std::to_string(l) + "]", cflat);
    }

    // The exact call evaluateGraph() makes when it scores a branch against the
    // preferred heading.
    std::vector<double> dirs;
    for (int k = 0; k < 16; ++k) {
      const double heading = -M_PI + k * (2.0 * M_PI / 16.0);
      dirs.push_back(Trajectory::computeDistanceBetweenTrajectoryAndDirection(
          paths[i], heading, 0.2, true));
      dirs.push_back(Trajectory::computeDistanceBetweenTrajectoryAndDirection(
          paths[i], heading, 0.2, false));
    }
    emit_darray(base + ".direction_distance", dirs);
  }

  std::vector<double> dtw;
  std::vector<double> pair_dist;
  std::string compare_codes;
  for (int a = 0; a < 3; ++a) {
    for (int b = 0; b < 3; ++b) {
      dtw.push_back(Trajectory::computeDTWDistance(paths[a], paths[b]));
      pair_dist.push_back(Trajectory::computeDistanceBetweenTwoTrajectories(
          paths[a], paths[b], 0.2, true, true));
      for (int t = 0; t < 20; ++t) {
        compare_codes.push_back(Trajectory::compareTwoTrajectories(
                                    paths[a], paths[b], t * 0.5, 0.2, true,
                                    true)
                                    ? '1'
                                    : '0');
      }
    }
  }
  emit_darray("traj.dtw", dtw);
  emit_darray("traj.pair_distance", pair_dist);
  emit_s("traj.compare_sweep", compare_codes);

  // The Pose overloads travel through geometry_msgs, whose field layout is the
  // same on both sides; Trajectory::WayPointType hides the namespace change.
  Trajectory::TrajectoryType traj;
  for (int i = 0; i <= 12; ++i) {
    Trajectory::WayPointType wp;
    wp.position.x = i * 0.55;
    wp.position.y = std::sin(i * 0.4);
    wp.position.z = 0.3 * i;
    wp.orientation.w = 1.0;
    traj.push_back(wp);
  }
  emit_d("traj.pose.length", Trajectory::getPathLength(traj));
  Trajectory::TrajectoryType tintp;
  Trajectory::interpolatePath(traj, 0.25, tintp);
  emit_i("traj.pose.interp.count", static_cast<long long>(tintp.size()));
  std::vector<double> tflat;
  for (const auto& p : tintp) {
    tflat.push_back(p.position.x);
    tflat.push_back(p.position.y);
    tflat.push_back(p.position.z);
  }
  emit_darray("traj.pose.interp.points", tflat);
  Trajectory::PathType extracted;
  Trajectory::extractPathFromTrajectory(traj, extracted);
  emit_d("traj.pose.extracted_length", Trajectory::getPathLength(extracted));
}

// ---------------------------------------------------------------------------
// E. Graph / GraphManager - Boost adjacency_list + dijkstra_shortest_paths.
//
// Boost moves from 1.71 to 1.83 across the port, so this is where a
// tie-breaking change in Dijkstra would show up. Distances are compared
// directly; parent ids are emitted separately and treated as advisory by the
// comparator, because a tie can legitimately resolve either way.
// ---------------------------------------------------------------------------
// Builds the tied lattice and solves it. `heap_shift` dummy allocations are
// made first: Graph uses boost::adjacency_list<setS, listS, ...>, so a vertex
// descriptor is a pointer into a std::list and the out-edge set is ordered by
// that pointer. Which of several equally short paths Dijkstra returns therefore
// depends on where the allocator happened to put the vertices. Running the same
// construction twice at different heap offsets is what turns that from a
// suspicion into a measurement.
struct LatticeResult {
  std::vector<double> distance;
  std::vector<long long> parent;
  std::vector<long long> leaf_ids;
  long long broken_edges = 0;
  double worst_residual = 0.0;
  int num_vertices = 0;
  int num_edges = 0;
  bool dijkstra_ok = false;
};

LatticeResult solveTiedLattice(size_t heap_shift) {
  std::vector<std::vector<int>*> ballast;
  ballast.reserve(heap_shift);
  for (size_t i = 0; i < heap_shift; ++i) ballast.push_back(new std::vector<int>(7, 1));

  LatticeResult out;
  GraphManager gm;

  // A 6x6x2 lattice with deliberately degenerate weights: every axis-aligned
  // step costs exactly 1.0, so most shortest paths are tied many ways.
  const int nx = 6, ny = 6, nz = 2;
  std::vector<Vertex*> verts;
  auto index = [&](int x, int y, int z) { return (z * ny + y) * nx + x; };
  for (int z = 0; z < nz; ++z) {
    for (int y = 0; y < ny; ++y) {
      for (int x = 0; x < nx; ++x) {
        StateVec s;
        s << x * 1.0, y * 1.0, z * 1.0, 0.0, 0.0;
        Vertex* v = new Vertex(gm.generateVertexID(), s);
        gm.addVertex(v);
        verts.push_back(v);
      }
    }
  }
  for (int z = 0; z < nz; ++z) {
    for (int y = 0; y < ny; ++y) {
      for (int x = 0; x < nx; ++x) {
        if (x + 1 < nx)
          gm.addEdge(verts[index(x, y, z)], verts[index(x + 1, y, z)], 1.0);
        if (y + 1 < ny)
          gm.addEdge(verts[index(x, y, z)], verts[index(x, y + 1, z)], 1.0);
        if (z + 1 < nz)
          gm.addEdge(verts[index(x, y, z)], verts[index(x, y, z + 1)], 1.0);
      }
    }
  }
  out.num_vertices = gm.getNumVertices();
  out.num_edges = gm.getNumEdges();

  ShortestPathsReport rep;
  out.dijkstra_ok = gm.findShortestPaths(0, rep);
  for (int i = 0; i < static_cast<int>(verts.size()); ++i) {
    out.distance.push_back(gm.getShortestDistance(i, rep));
    out.parent.push_back(gm.getParentIDFromShortestPath(i, rep));
  }

  // Whatever parent chain Boost picked, the resulting path must be contiguous
  // in the edge set and its length must equal the reported distance. That holds
  // on both sides even when the chains themselves differ.
  for (int i = 0; i < static_cast<int>(verts.size()); ++i) {
    std::vector<int> path;
    gm.getShortestPath(i, rep, true, path);
    double len = 0.0;
    for (size_t k = 1; k < path.size(); ++k) {
      const auto& nb = gm.edge_map_[path[k - 1]];
      auto it = std::find_if(nb.begin(), nb.end(),
                             [&](const std::pair<int, double>& e) {
                               return e.first == path[k];
                             });
      if (it == nb.end()) {
        ++out.broken_edges;
      } else {
        len += it->second;
      }
    }
    if (!path.empty()) {
      out.worst_residual = std::max(
          out.worst_residual, std::fabs(len - gm.getShortestDistance(i, rep)));
    }
  }

  std::vector<Vertex*> leaves;
  gm.findLeafVertices(rep);
  gm.getLeafVertices(leaves);
  for (auto* v : leaves) out.leaf_ids.push_back(v->id);
  std::sort(out.leaf_ids.begin(), out.leaf_ids.end());

  for (auto* p : ballast) delete p;
  return out;
}

void runGraph() {
  const LatticeResult a = solveTiedLattice(0);
  const LatticeResult b = solveTiedLattice(4099);

  emit_i("graph.tied.num_vertices", a.num_vertices);
  emit_i("graph.tied.num_edges", a.num_edges);
  emit_b("graph.tied.dijkstra_ok", a.dijkstra_ok);
  emit_darray("graph.tied.distance", a.distance);
  emit_iarray("graph.tied.parent", a.parent);
  emit_iarray("graph.tied.leaf_ids", a.leaf_ids);
  emit_i("graph.tied.broken_path_edges", a.broken_edges);
  emit_d("graph.tied.worst_path_length_residual", a.worst_residual);

  // The two runs differ only in where the allocator put the vertices.
  emit_b("graph.tied.distance_stable_under_heap_shift", a.distance == b.distance);
  emit_b("graph.tied.parent_stable_under_heap_shift", a.parent == b.parent);
  emit_b("graph.tied.leaf_ids_stable_under_heap_shift",
         a.leaf_ids == b.leaf_ids);
  emit_i("graph.tied.leaf_count", static_cast<long long>(a.leaf_ids.size()));
  emit_i("graph.tied.leaf_count_shifted",
         static_cast<long long>(b.leaf_ids.size()));

  // Nearest-neighbour lookups through the kd-tree the graph manager owns.
  GraphManager gm;
  std::vector<Vertex*> verts;
  for (int z = 0; z < 2; ++z) {
    for (int y = 0; y < 6; ++y) {
      for (int x = 0; x < 6; ++x) {
        StateVec s;
        s << x * 1.0, y * 1.0, z * 1.0, 0.0, 0.0;
        Vertex* v = new Vertex(gm.generateVertexID(), s);
        gm.addVertex(v);
        verts.push_back(v);
      }
    }
  }
  Lcg rng(0x9a9au);
  std::vector<long long> nearest;
  std::vector<long long> in_range_count;
  for (int q = 0; q < 60; ++q) {
    StateVec s;
    s << rng.uniform(-1.0, 6.0), rng.uniform(-1.0, 6.0), rng.uniform(-1.0, 2.0),
        0.0, 0.0;
    Vertex* nn = nullptr;
    nearest.push_back(gm.getNearestVertex(&s, &nn) && nn ? nn->id : -1);
    std::vector<Vertex*> around;
    in_range_count.push_back(gm.getNearestVertices(&s, 2.0, &around)
                                 ? static_cast<long long>(around.size())
                                 : -1);
  }
  emit_iarray("graph.tied.nearest_vertex", nearest);
  emit_iarray("graph.tied.in_range_count", in_range_count);

  // A second graph whose shortest paths are unique: irrational-ish weights make
  // ties impossible, so parent ids must agree exactly here.
  GraphManager gu;
  std::vector<Vertex*> uverts;
  Lcg wrng(0x4242u);
  const int n_unique = 80;
  for (int i = 0; i < n_unique; ++i) {
    StateVec s;
    s << wrng.uniform(-10.0, 10.0), wrng.uniform(-10.0, 10.0),
        wrng.uniform(-3.0, 3.0), 0.0, 0.0;
    Vertex* v = new Vertex(gu.generateVertexID(), s);
    gu.addVertex(v);
    uverts.push_back(v);
  }
  for (int i = 1; i < n_unique; ++i) {
    for (int j = 0; j < i; ++j) {
      const double d = (uverts[i]->state.head(3) - uverts[j]->state.head(3)).norm();
      if (d < 6.0) gu.addEdge(uverts[j], uverts[i], d);
    }
  }
  ShortestPathsReport urep;
  emit_i("graph.unique.num_vertices", gu.getNumVertices());
  emit_i("graph.unique.num_edges", gu.getNumEdges());
  emit_b("graph.unique.dijkstra_ok", gu.findShortestPaths(0, urep));
  std::vector<double> udist;
  std::vector<long long> uparent;
  std::vector<long long> upath_len;
  for (int i = 0; i < n_unique; ++i) {
    udist.push_back(gu.getShortestDistance(i, urep));
    uparent.push_back(gu.getParentIDFromShortestPath(i, urep));
    std::vector<int> path;
    gu.getShortestPath(i, urep, true, path);
    upath_len.push_back(static_cast<long long>(path.size()));
  }
  emit_darray("graph.unique.distance", udist);
  emit_iarray("graph.unique.parent", uparent);
  emit_iarray("graph.unique.path_length", upath_len);
}

// ---------------------------------------------------------------------------
// F. GeofenceManager - boost::geometry, another Boost 1.71 vs 1.83 surface.
// ---------------------------------------------------------------------------
void runGeofence() {
  GeofenceManager gf;
  std::vector<std::vector<Eigen::Vector2d>> polys = {
      // convex
      {{0.0, 0.0}, {4.0, 0.0}, {4.0, 3.0}, {0.0, 3.0}},
      // concave (L shaped)
      {{6.0, 0.0}, {10.0, 0.0}, {10.0, 4.0}, {8.0, 4.0}, {8.0, 2.0}, {6.0, 2.0}},
      // one that overlaps the first
      {{3.0, 2.0}, {7.0, 2.0}, {7.0, 5.0}, {3.0, 5.0}},
      // a thin sliver
      {{-4.0, -1.0}, {-3.9, -1.0}, {-3.9, 6.0}, {-4.0, 6.0}},
  };
  for (auto& p : polys) {
    Polygon2d poly(p);
    gf.addGeofenceArea(poly);
  }
  std::vector<GeofenceArea> areas;
  gf.getAllGeofenceAreas(areas);
  emit_i("geofence.num_areas", static_cast<long long>(areas.size()));

  std::string coord_codes;
  for (int ix = -60; ix <= 120; ix += 3) {
    for (int iy = -30; iy <= 70; iy += 3) {
      Eigen::Vector2d p(ix * 0.1, iy * 0.1);
      coord_codes.push_back(
          '0' + static_cast<int>(gf.getCoordinateStatus(p)));
    }
  }
  emit_s("geofence.coordinate_status", coord_codes);

  // getBoxStatus with the real robot footprint is the call the sampler makes
  // for every candidate vertex.
  std::string box_codes;
  const Eigen::Vector2d box(0.4, 0.4);
  for (int ix = -60; ix <= 120; ix += 4) {
    for (int iy = -30; iy <= 70; iy += 4) {
      Eigen::Vector2d p(ix * 0.1, iy * 0.1);
      box_codes.push_back('0' + static_cast<int>(gf.getBoxStatus(p, box)));
    }
  }
  emit_s("geofence.box_status", box_codes);

  std::string path_codes;
  Lcg rng(0x7e7eu);
  for (int i = 0; i < 300; ++i) {
    Eigen::Vector2d a(rng.uniform(-6.0, 12.0), rng.uniform(-3.0, 7.0));
    Eigen::Vector2d b(rng.uniform(-6.0, 12.0), rng.uniform(-3.0, 7.0));
    path_codes.push_back('0' + static_cast<int>(gf.getPathStatus(a, b, box)));
  }
  emit_s("geofence.path_status", path_codes);
}

// ---------------------------------------------------------------------------
// G. RandomSampler - the RNG canary.
//
// The sampler's mt19937 is seeded from GBP_PARITY_SEED, see gbp_parity_seed.h.
// If this section disagrees, no downstream comparison of anything that samples
// is meaningful, so it is reported first in the summary.
// ---------------------------------------------------------------------------
void declareSamplerParams(parity::Fixture& fx) {
  const std::string ns = "RandomSamplerParams.SamplerForExploration";
  const char* axes[] = {"X", "Y", "Z"};
  for (const char* a : axes) {
    fx.setParam(ns + "." + a + ".pdf_type", "kUniform");
    fx.setParam(ns + "." + a + ".sample_mode", "kLocal");
  }
  fx.setParam(ns + ".Heading.pdf_type", "kUniform");
  fx.setParam(ns + ".Heading.sample_mode", "kManual");
  fx.setParam(ns + ".Heading.min_val", -M_PI);
  fx.setParam(ns + ".Heading.max_val", M_PI);
  fx.setParam(ns + ".Pitch.pdf_type", "kConst");
  fx.setParam(ns + ".Pitch.sample_mode", "kManual");
  fx.setParam(ns + ".Pitch.const_val", 0.0);

  // A second sampler exercising the normal and Cauchy branches, which the
  // adaptive-exploration configurations use.
  const std::string ns2 = "RandomSamplerParams.SamplerNormal";
  fx.setParam(ns2 + ".X.pdf_type", "kNormal");
  fx.setParam(ns2 + ".X.sample_mode", "kManual");
  fx.setParam(ns2 + ".X.mean_val", 0.0);
  fx.setParam(ns2 + ".X.std_val", 3.0);
  fx.setParam(ns2 + ".X.min_val", -12.0);
  fx.setParam(ns2 + ".X.max_val", 12.0);
  fx.setParam(ns2 + ".Y.pdf_type", "kCauchy");
  fx.setParam(ns2 + ".Y.sample_mode", "kManual");
  fx.setParam(ns2 + ".Y.mean_val", 0.0);
  fx.setParam(ns2 + ".Y.std_val", 2.0);
  fx.setParam(ns2 + ".Y.min_val", -12.0);
  fx.setParam(ns2 + ".Y.max_val", 12.0);
  fx.setParam(ns2 + ".Z.pdf_type", "kUniform");
  fx.setParam(ns2 + ".Z.sample_mode", "kManual");
  fx.setParam(ns2 + ".Z.min_val", -1.0);
  fx.setParam(ns2 + ".Z.max_val", 1.0);
  fx.setParam(ns2 + ".Heading.pdf_type", "kUniform");
  fx.setParam(ns2 + ".Heading.sample_mode", "kManual");
  fx.setParam(ns2 + ".Heading.min_val", -M_PI);
  fx.setParam(ns2 + ".Heading.max_val", M_PI);
  fx.setParam(ns2 + ".Pitch.pdf_type", "kConst");
  fx.setParam(ns2 + ".Pitch.sample_mode", "kManual");
  fx.setParam(ns2 + ".Pitch.const_val", 0.0);
}

void runRandomSampler(parity::Fixture& fx, BoundedSpaceParams& global_space,
                      BoundedSpaceParams& local_space) {
  // Reference sequence for the generator itself. std::mt19937 is bit-specified
  // by the standard; std::uniform_real_distribution is not, so both are
  // reported separately and the comparator explains which one moved.
  {
    std::mt19937 gen(12345u);
    std::vector<long long> raw;
    for (int i = 0; i < 16; ++i) raw.push_back(gen());
    emit_iarray("rng.mt19937_seed12345", raw);
    std::mt19937 gen2(5489u);
    for (int i = 0; i < 9999; ++i) gen2();
    emit_i("rng.mt19937_10000th_seed5489", gen2());

    std::mt19937 g3(12345u);
    std::uniform_real_distribution<double> u(-15.0, 15.0);
    std::vector<double> uv;
    for (int i = 0; i < 16; ++i) uv.push_back(u(g3));
    emit_darray("rng.uniform_real_seed12345", uv);

    std::mt19937 g4(12345u);
    std::normal_distribution<double> nd(0.0, 3.0);
    std::vector<double> nv;
    for (int i = 0; i < 16; ++i) nv.push_back(nd(g4));
    emit_darray("rng.normal_seed12345", nv);

    std::mt19937 g5(12345u);
    std::cauchy_distribution<double> cd(0.0, 2.0);
    std::vector<double> cv;
    for (int i = 0; i < 16; ++i) cv.push_back(cd(g5));
    emit_darray("rng.cauchy_seed12345", cv);

    std::mt19937 g6(12345u);
    std::chi_squared_distribution<double> xd(1);
    std::vector<double> xv;
    for (int i = 0; i < 16; ++i) xv.push_back(xd(g6));
    emit_darray("rng.chi_squared_seed12345", xv);
  }

  RandomSampler sampler;
  const bool ok =
      fx.loadParams(sampler, "RandomSamplerParams.SamplerForExploration");
  emit_b("sampler.load_ok", ok);
  if (ok) {
    emit_b("sampler.set_params_ok",
           sampler.setParams(global_space, local_space, true));
    sampler.reset();
    StateVec root;
    root << 1.0, -2.0, 0.5, 0.25, 0.0;
    std::vector<double> flat;
    for (int i = 0; i < 400; ++i) {
      StateVec out;
      sampler.generate(root, out);
      flat.push_back(out[0]);
      flat.push_back(out[1]);
      flat.push_back(out[2]);
      flat.push_back(out[3]);
      flat.push_back(out[4]);
    }
    emit_darray("sampler.exploration.samples", flat);

    // reset() re-seeds from the same constant, so the second run must repeat
    // the first exactly. This is what makes the fixed-seed contract testable.
    sampler.reset();
    std::vector<double> repeat;
    for (int i = 0; i < 20; ++i) {
      StateVec out;
      sampler.generate(root, out);
      repeat.push_back(out[0]);
      repeat.push_back(out[1]);
      repeat.push_back(out[2]);
    }
    emit_darray("sampler.exploration.after_reset", repeat);
  }

  RandomSampler normal_sampler;
  const bool nok =
      fx.loadParams(normal_sampler, "RandomSamplerParams.SamplerNormal");
  emit_b("sampler.normal.load_ok", nok);
  if (nok) {
    normal_sampler.reset();
    StateVec root;
    root << 0.0, 0.0, 0.0, 0.0, 0.0;
    std::vector<double> flat;
    for (int i = 0; i < 200; ++i) {
      StateVec out;
      normal_sampler.generate(root, out);
      flat.push_back(out[0]);
      flat.push_back(out[1]);
      flat.push_back(out[2]);
      flat.push_back(out[3]);
    }
    emit_darray("sampler.normal.samples", flat);
  }
}

// ---------------------------------------------------------------------------
// H. map_manager over a synthetic voxblox layer.
//
// MapManagerVoxblox owns its TSDF server by value and exposes no way to reach
// the layer, so the layer pointer is fetched with the explicit-instantiation
// access idiom below: [temp.spec] exempts the names in an explicit
// instantiation from access checking, which is the only ISO-legal way to touch
// a private member of a class neither repository is allowed to change.
// ---------------------------------------------------------------------------
using MMV = parity::MapManagerImpl;

template <typename Tag, typename Tag::type M>
struct GbpParityRob {
  friend typename Tag::type gbpParityMember(Tag) { return M; }
};

struct SdfLayerTag {
  using type = voxblox::Layer<MapManagerVoxbloxVoxel>* MMV::*;
  friend type gbpParityMember(SdfLayerTag);
};
template struct GbpParityRob<SdfLayerTag, &MMV::sdf_layer_>;

// MapManager is a thin forwarding facade; the layer lives one level below it.
struct MapImplTag {
  using type = std::shared_ptr<MMV> MapManager::*;
  friend type gbpParityMember(MapImplTag);
};
template struct GbpParityRob<MapImplTag, &MapManager::map_manager_impl_>;

// The planner's two graphs. Everything the RRG produces is read out of these.
struct LocalGraphTag {
  using type = std::shared_ptr<GraphManager> Rrg::*;
  friend type gbpParityMember(LocalGraphTag);
};
template struct GbpParityRob<LocalGraphTag, &Rrg::local_graph_>;

struct GlobalGraphTag {
  using type = std::shared_ptr<GraphManager> Rrg::*;
  friend type gbpParityMember(GlobalGraphTag);
};
template struct GbpParityRob<GlobalGraphTag, &Rrg::global_graph_>;

// The synthetic world: a straight corridor closed by a cross wall with a
// doorway, plus a pillar. Everything is an axis-aligned box so the ray casts
// that build the TSDF are exact rational arithmetic, identical on both sides.
struct Box {
  Eigen::Vector3d lo, hi;
};

const std::vector<Box>& scene() {
  static const std::vector<Box> boxes = {
      {{0.0, -2.5, -0.4}, {12.0, 2.5, 0.0}},    // floor
      {{0.0, -2.5, 3.0}, {12.0, 2.5, 3.4}},     // ceiling
      {{0.0, -2.5, 0.0}, {12.0, -2.0, 3.0}},    // left wall
      {{0.0, 2.0, 0.0}, {12.0, 2.5, 3.0}},      // right wall
      {{-0.4, -2.5, 0.0}, {0.0, 2.5, 3.0}},     // back wall
      {{8.0, -2.5, 0.0}, {8.4, -0.5, 3.0}},     // cross wall, lower half
      {{8.0, 0.5, 0.0}, {8.4, 2.5, 3.0}},       // cross wall, upper half
      {{4.0, -0.3, 0.0}, {4.6, 0.3, 3.0}},      // pillar
  };
  return boxes;
}

// Slab method; returns the smallest positive t at which the ray enters a box.
bool castRay(const Eigen::Vector3d& o, const Eigen::Vector3d& d,
             double max_range, double* t_hit) {
  double best = max_range;
  bool hit = false;
  for (const Box& b : scene()) {
    double tmin = 0.0, tmax = max_range;
    bool ok = true;
    for (int a = 0; a < 3; ++a) {
      if (std::fabs(d[a]) < 1e-12) {
        if (o[a] < b.lo[a] || o[a] > b.hi[a]) {
          ok = false;
          break;
        }
        continue;
      }
      const double inv = 1.0 / d[a];
      double t1 = (b.lo[a] - o[a]) * inv;
      double t2 = (b.hi[a] - o[a]) * inv;
      if (t1 > t2) std::swap(t1, t2);
      tmin = std::max(tmin, t1);
      tmax = std::min(tmax, t2);
      if (tmin > tmax) {
        ok = false;
        break;
      }
    }
    if (ok && tmin > 1e-6 && tmin < best) {
      best = tmin;
      hit = true;
    }
  }
  *t_hit = best;
  return hit;
}

void buildSyntheticLayer(voxblox::Layer<voxblox::TsdfVoxel>* layer,
                         const std::string& key) {
  // Every field is spelled out so that a default changing in one fork cannot
  // silently change what the two sides integrate.
  voxblox::TsdfIntegratorBase::Config cfg;
  cfg.default_truncation_distance = 0.8f;
  cfg.voxel_carving_enabled = true;
  cfg.min_ray_length_m = 0.1f;
  cfg.max_ray_length_m = 30.0f;
  cfg.allow_clear = true;
  cfg.max_weight = 10000.0f;
  cfg.clearing_ray_weight_factor = 1.0f;
  cfg.weight_ray_by_range = false;
  cfg.use_symmetric_weight_dropoff = false;
  cfg.use_weight_dropoff = true;
  cfg.use_const_weight = false;
  // Multi-threaded integration accumulates the same weighted averages in a
  // scheduler-dependent order, which is not reproducible even ROS 1 to ROS 1.
  cfg.integrator_threads = 1;
  cfg.integration_order_mode = "sorted";

  voxblox::SimpleTsdfIntegrator integrator(cfg, layer);

  const double poses[][4] = {{1.0, 0.0, 1.0, 0.0},
                             {3.0, 0.0, 1.0, 0.0},
                             {5.0, 0.5, 1.2, 0.3},
                             {7.0, -0.5, 1.0, -0.4}};
  long long total_points = 0;
  for (int p = 0; p < 4; ++p) {
    const Eigen::Vector3d origin(poses[p][0], poses[p][1], poses[p][2]);
    const double yaw = poses[p][3];
    voxblox::Pointcloud cloud;
    voxblox::Colors colors;
    for (int ia = 0; ia < 72; ++ia) {
      const double az = ia * (2.0 * M_PI / 72.0);
      for (int ie = -6; ie <= 6; ++ie) {
        const double el = ie * (5.0 * M_PI / 180.0);
        const Eigen::Vector3d dir(std::cos(el) * std::cos(az),
                                  std::cos(el) * std::sin(az), std::sin(el));
        double t = 0.0;
        if (!castRay(origin, dir, 25.0, &t)) continue;
        // The pointcloud is expressed in the sensor frame; the yaw goes into
        // T_G_C so the integrator sees a non-trivial rotation.
        const Eigen::Vector3d p_w = dir * t;
        const double c = std::cos(-yaw), s = std::sin(-yaw);
        cloud.emplace_back(static_cast<float>(c * p_w.x() - s * p_w.y()),
                           static_cast<float>(s * p_w.x() + c * p_w.y()),
                           static_cast<float>(p_w.z()));
        colors.emplace_back(voxblox::Color(120, 120, 120));
      }
    }
    total_points += static_cast<long long>(cloud.size());

    Eigen::Matrix<float, 4, 4> T = Eigen::Matrix<float, 4, 4>::Identity();
    const float c = static_cast<float>(std::cos(yaw));
    const float s = static_cast<float>(std::sin(yaw));
    T(0, 0) = c;
    T(0, 1) = -s;
    T(1, 0) = s;
    T(1, 1) = c;
    T(0, 3) = static_cast<float>(origin.x());
    T(1, 3) = static_cast<float>(origin.y());
    T(2, 3) = static_cast<float>(origin.z());
    integrator.integratePointCloud(voxblox::Transformation(T), cloud, colors);
  }
  emit_i(key, total_points);
}

char statusChar(VoxelStatus s) {
  switch (s) {
    case VoxelStatus::kUnknown:
      return 'U';
    case VoxelStatus::kFree:
      return 'F';
    case VoxelStatus::kOccupied:
      return 'O';
  }
  return '?';
}

void runMapManager(parity::Fixture& fx, SensorParams& sensors) {
  MMV& mm = *(fx.map().*gbpParityMember(MapImplTag{}));
  voxblox::Layer<MapManagerVoxbloxVoxel>* layer = mm.*gbpParityMember(SdfLayerTag{});

  emit_d("map.resolution", mm.getResolution());
  emit_i("map.voxels_per_side", static_cast<long long>(layer->voxels_per_side()));
  emit_d("map.truncation_distance", mm.getTruncationDistance());
  emit_b("map.status", mm.getStatus());

  // These four are uninitialised in the shipped constructor, so the test sets
  // them explicitly rather than comparing whatever the stack happened to hold.
  mm.setRobotRadius(0.3);
  mm.setBoxCheckMethod(0);
  mm.setLineCheckMethod(0);
  mm.setRaycastingParams(true, 1.0);

  buildSyntheticLayer(layer, "map.integrated_points");

  // Layer statistics, walked in lexicographic global-index order: voxblox's
  // block hash map iterates in an order that is not portable, so anything that
  // depends on iteration order has to be sorted before it is compared.
  voxblox::BlockIndexList blocks;
  layer->getAllAllocatedBlocks(&blocks);
  std::vector<std::array<long long, 3>> sorted_blocks;
  for (const auto& b : blocks) {
    sorted_blocks.push_back({b.x(), b.y(), b.z()});
  }
  std::sort(sorted_blocks.begin(), sorted_blocks.end());
  emit_i("map.num_blocks", static_cast<long long>(sorted_blocks.size()));

  const int vps = layer->voxels_per_side();
  long long observed = 0, occupied = 0, free_voxels = 0;
  double dist_sum = 0.0, weight_sum = 0.0, abs_dist_sum = 0.0;
  const double occ_thresh = mm.getResolution();  // factor is 1.0 by default
  std::map<std::array<long long, 3>, std::pair<double, double>> sampled;
  for (const auto& bi : sorted_blocks) {
    voxblox::BlockIndex idx(static_cast<int>(bi[0]), static_cast<int>(bi[1]),
                            static_cast<int>(bi[2]));
    const auto& block = layer->getBlockByIndex(idx);
    for (int lx = 0; lx < vps; ++lx) {
      for (int ly = 0; ly < vps; ++ly) {
        for (int lz = 0; lz < vps; ++lz) {
          const voxblox::VoxelIndex vi(lx, ly, lz);
          const voxblox::TsdfVoxel& v = block.getVoxelByVoxelIndex(vi);
          if (v.weight < 1e-6) continue;
          ++observed;
          dist_sum += v.distance;
          abs_dist_sum += std::fabs(v.distance);
          weight_sum += v.weight;
          if (v.distance <= occ_thresh)
            ++occupied;
          else
            ++free_voxels;
          const std::array<long long, 3> g = {bi[0] * vps + lx, bi[1] * vps + ly,
                                              bi[2] * vps + lz};
          // 400-ish voxels spread over the whole map, so a divergence can be
          // located rather than merely detected.
          if (((g[0] * 7 + g[1] * 13 + g[2] * 17) % 173) == 0) {
            sampled[g] = {v.distance, v.weight};
          }
        }
      }
    }
  }
  emit_i("map.observed_voxels", observed);
  emit_i("map.occupied_voxels", occupied);
  emit_i("map.free_voxels", free_voxels);
  emit_d("map.distance_sum", dist_sum);
  emit_d("map.abs_distance_sum", abs_dist_sum);
  emit_d("map.weight_sum", weight_sum);
  emit_i("map.sampled_voxel_count", static_cast<long long>(sampled.size()));
  {
    std::vector<double> flat;
    for (const auto& kv : sampled) {
      flat.push_back(static_cast<double>(kv.first[0]));
      flat.push_back(static_cast<double>(kv.first[1]));
      flat.push_back(static_cast<double>(kv.first[2]));
      flat.push_back(kv.second.first);
      flat.push_back(kv.second.second);
    }
    emit_darray("map.sampled_voxels", flat);
  }

  // A wall must read as occupied and the corridor as free: these three probes
  // are the whole point of the exercise, stated as a value rather than a hash.
  {
    Eigen::Vector3d in_wall(6.0, 2.2, 1.5);
    Eigen::Vector3d in_corridor(6.0, 0.0, 1.5);
    Eigen::Vector3d beyond(10.5, 0.0, 1.5);
    emit_s("map.probe.wall", std::string(1, statusChar(mm.getVoxelStatus(in_wall))));
    emit_s("map.probe.corridor",
           std::string(1, statusChar(mm.getVoxelStatus(in_corridor))));
    emit_s("map.probe.behind_cross_wall",
           std::string(1, statusChar(mm.getVoxelStatus(beyond))));
    emit_d("map.probe.wall_distance", mm.getPointDistance(in_wall));
    emit_d("map.probe.corridor_distance", mm.getPointDistance(in_corridor));
  }

  // getVoxelStatus over a regular grid spanning the corridor.
  {
    std::string codes;
    for (int ix = -2; ix <= 66; ++ix) {
      for (int iy = -16; iy <= 16; ++iy) {
        for (int iz = -2; iz <= 18; ++iz) {
          Eigen::Vector3d p(ix * 0.2, iy * 0.2, iz * 0.2);
          codes.push_back(statusChar(mm.getVoxelStatus(p)));
        }
      }
    }
    emit_i("map.voxel_status.count", static_cast<long long>(codes.size()));
    emit_s("map.voxel_status", codes);
  }

  // Distances and gradients at fixed probes.
  {
    Lcg rng(0xd1d1u);
    std::vector<double> vdist, pdist, grads;
    for (int i = 0; i < 250; ++i) {
      Eigen::Vector3d p(rng.uniform(-0.5, 12.5), rng.uniform(-2.8, 2.8),
                        rng.uniform(-0.5, 3.5));
      vdist.push_back(mm.getVoxelDistance(p));
      pdist.push_back(mm.getPointDistance(p));
      Eigen::Vector3d g = mm.getPointGradient(p);
      grads.push_back(g.x());
      grads.push_back(g.y());
      grads.push_back(g.z());
    }
    emit_darray("map.voxel_distance", vdist);
    emit_darray("map.point_distance", pdist);
    emit_darray("map.point_gradient", grads);
  }

  // getBoxStatus with the real robot bounding box.
  {
    const Eigen::Vector3d box(0.4, 0.4, 0.4);
    Lcg rng(0xb0b0u);
    std::string codes_stop, codes_nostop;
    for (int i = 0; i < 600; ++i) {
      Eigen::Vector3d c(rng.uniform(-0.5, 12.5), rng.uniform(-2.8, 2.8),
                        rng.uniform(-0.5, 3.5));
      codes_stop.push_back(statusChar(mm.getBoxStatus(c, box, true)));
      codes_nostop.push_back(statusChar(mm.getBoxStatus(c, box, false)));
    }
    emit_s("map.box_status.stop_at_unknown", codes_stop);
    emit_s("map.box_status.no_stop", codes_nostop);
  }

  // getPathStatus is the edge-collision check the RRG runs on every candidate
  // edge, so it is the single most load-bearing map query in the planner.
  {
    const Eigen::Vector3d box(0.4, 0.4, 0.4);
    Lcg rng(0x5151u);
    std::string codes_stop, codes_nostop;
    for (int i = 0; i < 400; ++i) {
      Eigen::Vector3d a(rng.uniform(0.0, 12.0), rng.uniform(-2.5, 2.5),
                        rng.uniform(0.0, 3.0));
      Eigen::Vector3d b(rng.uniform(0.0, 12.0), rng.uniform(-2.5, 2.5),
                        rng.uniform(0.0, 3.0));
      codes_stop.push_back(statusChar(mm.getPathStatus(a, b, box, true)));
      codes_nostop.push_back(statusChar(mm.getPathStatus(a, b, box, false)));
    }
    emit_s("map.path_status.stop_at_unknown", codes_stop);
    emit_s("map.path_status.no_stop", codes_nostop);
  }

  // getRayStatus, including the overload that reports where the ray stopped.
  {
    Lcg rng(0x3737u);
    std::string codes_stop, codes_nostop;
    std::vector<double> ends, tsdf;
    for (int i = 0; i < 400; ++i) {
      Eigen::Vector3d a(rng.uniform(0.5, 11.5), rng.uniform(-2.0, 2.0),
                        rng.uniform(0.2, 2.8));
      Eigen::Vector3d b(rng.uniform(0.0, 13.0), rng.uniform(-3.0, 3.0),
                        rng.uniform(-0.5, 3.5));
      codes_nostop.push_back(statusChar(mm.getRayStatus(a, b, false)));
      Eigen::Vector3d end(0, 0, 0);
      double d = 0.0;
      codes_stop.push_back(statusChar(mm.getRayStatus(a, b, true, end, d)));
      ends.push_back(end.x());
      ends.push_back(end.y());
      ends.push_back(end.z());
      tsdf.push_back(d);
    }
    emit_s("map.ray_status.stop_at_unknown", codes_stop);
    emit_s("map.ray_status.no_stop", codes_nostop);
    emit_darray("map.ray_status.end_voxel", ends);
    emit_darray("map.ray_status.tsdf_dist", tsdf);
  }

  // getScanStatus is the map half of the volumetric gain: the gain_log triple
  // it returns is literally what Rrg::computeVolumetricGainRayModel counts.
  {
    const double poses[][5] = {{1.0, 0.0, 1.0, 0.0, 0.0},
                               {5.0, 0.0, 1.5, 0.7, 0.0},
                               {7.6, 0.0, 1.5, 0.0, 0.0},
                               {7.6, 0.0, 1.5, M_PI, 0.0},
                               {3.0, 1.5, 2.0, -1.2, 0.2},
                               {11.0, 0.0, 1.5, 0.0, 0.0}};
    for (const char* name : kSensorNames) {
      auto it = sensors.sensor.find(name);
      if (it == sensors.sensor.end()) continue;
      SensorParamsBase& sp = it->second;
      for (int p = 0; p < 6; ++p) {
        StateVec st;
        st << poses[p][0], poses[p][1], poses[p][2], poses[p][3], poses[p][4];
        std::vector<Eigen::Vector3d> ep;
        sp.getFrustumEndpoints(st, ep);
        Eigen::Vector3d pos(st[0], st[1], st[2]);
        std::tuple<int, int, int> gain_log;
        std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> voxel_log;
        mm.getScanStatus(pos, ep, gain_log, voxel_log, sp);

        const std::string base = std::string("map.scan.") + name + ".pose" +
                                 std::to_string(p);
        emit_iarray(base + ".gain_log",
                    {std::get<0>(gain_log), std::get<1>(gain_log),
                     std::get<2>(gain_log)});
        emit_i(base + ".voxel_log_size",
               static_cast<long long>(voxel_log.size()));
        std::string codes;
        double sx = 0.0, sy = 0.0, sz = 0.0;
        codes.reserve(voxel_log.size());
        for (const auto& e : voxel_log) {
          codes.push_back(statusChar(e.second));
          sx += e.first.x();
          sy += e.first.y();
          sz += e.first.z();
        }
        emit_s(base + ".voxel_log_status", codes);
        emit_darray(base + ".voxel_log_centre_sum", {sx, sy, sz});
        std::vector<double> head;
        for (size_t k = 0; k < voxel_log.size() && k < 24; ++k) {
          head.push_back(voxel_log[k].first.x());
          head.push_back(voxel_log[k].first.y());
          head.push_back(voxel_log[k].first.z());
        }
        emit_darray(base + ".voxel_log_head", head);
      }
    }
  }

  // extractLocalMap and getLocalPointcloud: both feed the planner's local
  // occupancy reasoning and both walk the layer, so they are order sensitive.
  {
    Eigen::Vector3d centre(5.0, 0.0, 1.5);
    Eigen::Vector3d bbox(8.0, 4.0, 3.0);
    std::vector<Eigen::Vector3d> occ, fr;
    mm.extractLocalMap(centre, bbox, occ, fr);
    emit_i("map.local.occupied_count", static_cast<long long>(occ.size()));
    emit_i("map.local.free_count", static_cast<long long>(fr.size()));
    auto sum = [](const std::vector<Eigen::Vector3d>& v) {
      Eigen::Vector3d s(0, 0, 0);
      for (const auto& p : v) s += p;
      return s;
    };
    emitVec3("map.local.occupied_sum", sum(occ));
    emitVec3("map.local.free_sum", sum(fr));

    // Sorted so that a difference in layer traversal order is not reported as a
    // difference in content.
    auto sorted_flat = [](std::vector<Eigen::Vector3d> v) {
      std::sort(v.begin(), v.end(), [](const Eigen::Vector3d& a,
                                       const Eigen::Vector3d& b) {
        if (a.x() != b.x()) return a.x() < b.x();
        if (a.y() != b.y()) return a.y() < b.y();
        return a.z() < b.z();
      });
      std::vector<double> out;
      for (size_t i = 0; i < v.size(); i += std::max<size_t>(1, v.size() / 40)) {
        out.push_back(v[i].x());
        out.push_back(v[i].y());
        out.push_back(v[i].z());
      }
      return out;
    };
    emit_darray("map.local.occupied_sample", sorted_flat(occ));
    emit_darray("map.local.free_sample", sorted_flat(fr));

    pcl::PointCloud<pcl::PointXYZI> cloud;
    mm.getLocalPointcloud(centre, 6.0, 0.0, cloud, false);
    emit_i("map.local_cloud.size", static_cast<long long>(cloud.points.size()));
    double cs = 0.0;
    for (const auto& p : cloud.points) cs += p.x + p.y + p.z + p.intensity;
    emit_d("map.local_cloud.checksum", cs);

    pcl::PointCloud<pcl::PointXYZI> cloud_z;
    Eigen::Vector2d zlim(0.5, 2.5);
    mm.getLocalPointcloud(centre, 6.0, 0.0, zlim, cloud_z, true);
    emit_i("map.local_cloud_z.size",
           static_cast<long long>(cloud_z.points.size()));
    double czs = 0.0;
    for (const auto& p : cloud_z.points) czs += p.x + p.y + p.z + p.intensity;
    emit_d("map.local_cloud_z.checksum", czs);
  }
}


// ---------------------------------------------------------------------------
// I. PlanningParams - the largest hand-rewritten block in params.cpp.
//
// It is loaded twice: once from an empty namespace, so that every
// "read failed, install the derived default" branch fires, and once from the
// cave configuration. The empty case is the sharp one: ROS 2 refuses to read an
// undeclared parameter, and the obvious fix - declaring with a typed default -
// makes the read *succeed* with an empty value and silently zeroes about a
// hundred fields. If the port got that wrong, the two dumps below disagree on
// nearly every key.
// ---------------------------------------------------------------------------
void emitPlanningParams(const std::string& b, PlanningParams& p) {
  emit_s(b + ".global_frame_id", p.global_frame_id);
  emit_d(b + ".v_max", p.v_max);
  emit_d(b + ".v_homing_max", p.v_homing_max);
  emit_d(b + ".yaw_rate_max", p.yaw_rate_max);
  emit_b(b + ".yaw_tangent_correction", p.yaw_tangent_correction);
  emit_i(b + ".type", static_cast<long long>(p.type));
  emit_i(b + ".graph_building_mode",
         static_cast<long long>(p.graph_building_mode));
  emit_i(b + ".rr_mode", static_cast<long long>(p.rr_mode));
  emit_d(b + ".edge_length_min", p.edge_length_min);
  emit_d(b + ".edge_length_max", p.edge_length_max);
  emit_d(b + ".edge_overshoot", p.edge_overshoot);
  emit_d(b + ".num_vertices_max", p.num_vertices_max);
  emit_d(b + ".num_edges_max", p.num_edges_max);
  emit_d(b + ".num_loops_cutoff", p.num_loops_cutoff);
  emit_d(b + ".num_loops_max", p.num_loops_max);
  emit_d(b + ".nearest_range", p.nearest_range);
  emit_d(b + ".nearest_range_min", p.nearest_range_min);
  emit_d(b + ".nearest_range_max", p.nearest_range_max);
  emit_b(b + ".use_current_state", p.use_current_state);
  emit_b(b + ".geofence_checking_enable", p.geofence_checking_enable);
  emit_d(b + ".max_ground_height", p.max_ground_height);
  emit_d(b + ".robot_height", p.robot_height);
  emit_d(b + ".max_inclination", p.max_inclination);
  emit_b(b + ".interpolate_projection_distance", p.interpolate_projection_distance);
  emit_d(b + ".augment_free_voxels_time", p.augment_free_voxels_time);
  // augment_free_frustum_en is declared in params.h:299 and never written by
  // loadParams, nor read anywhere in the tree. Dumping it compares
  // uninitialised memory - see README, "Findings".
  emit_b(b + ".free_frustum_before_planning", p.free_frustum_before_planning);
  emit_i(b + ".exp_sensor_list.size",
         static_cast<long long>(p.exp_sensor_list.size()));
  for (size_t i = 0; i < p.exp_sensor_list.size(); ++i)
    emit_s(vecKey(b + ".exp_sensor_list", static_cast<int>(i)),
           p.exp_sensor_list[i]);
  emit_i(b + ".inspection_sensor_list.size",
         static_cast<long long>(p.inspection_sensor_list.size()));
  for (size_t i = 0; i < p.inspection_sensor_list.size(); ++i)
    emit_s(vecKey(b + ".inspection_sensor_list", static_cast<int>(i)),
           p.inspection_sensor_list[i]);
  emit_i(b + ".no_gain_zones_list.size",
         static_cast<long long>(p.no_gain_zones_list.size()));
  emit_d(b + ".exp_gain_voxel_size", p.exp_gain_voxel_size);
  emit_b(b + ".use_ray_model_for_volumetric_gain",
         p.use_ray_model_for_volumetric_gain);
  emit_d(b + ".free_voxel_gain", p.free_voxel_gain);
  emit_d(b + ".occupied_voxel_gain", p.occupied_voxel_gain);
  emit_d(b + ".unknown_voxel_gain", p.unknown_voxel_gain);
  emit_d(b + ".path_length_penalty", p.path_length_penalty);
  emit_d(b + ".path_direction_penalty", p.path_direction_penalty);
  emit_d(b + ".hanging_vertex_penalty", p.hanging_vertex_penalty);
  emit_b(b + ".leafs_only_for_volumetric_gain", p.leafs_only_for_volumetric_gain);
  emit_b(b + ".cluster_vertices_for_gain", p.cluster_vertices_for_gain);
  emit_d(b + ".clustering_radius", p.clustering_radius);
  emit_d(b + ".ray_cast_step_size_multiplier", p.ray_cast_step_size_multiplier);
  emit_b(b + ".nonuniform_ray_cast", p.nonuniform_ray_cast);
  emit_d(b + ".traverse_length_max", p.traverse_length_max);
  emit_d(b + ".traverse_time_max", p.traverse_time_max);
  emit_b(b + ".planning_backward", p.planning_backward);
  emit_b(b + ".path_safety_enhance_enable", p.path_safety_enhance_enable);
  emit_d(b + ".path_interpolation_distance", p.path_interpolation_distance);
  emit_d(b + ".relaxed_corridor_multiplier", p.relaxed_corridor_multiplier);
  emit_b(b + ".auto_global_planner_enable", p.auto_global_planner_enable);
  emit_b(b + ".go_home_if_fully_explored", p.go_home_if_fully_explored);
  emit_b(b + ".auto_homing_enable", p.auto_homing_enable);
  emit_b(b + ".homing_backward", p.homing_backward);
  emit_d(b + ".time_budget_limit", p.time_budget_limit);
  emit_b(b + ".auto_landing_enable", p.auto_landing_enable);
  emit_d(b + ".time_budget_before_landing", p.time_budget_before_landing);
  emit_b(b + ".use_camera_gain", p.use_camera_gain);
  emit_b(b + ".annotate_map_with_camera", p.annotate_map_with_camera);
  emit_b(b + ".inspection_planning", p.inspection_planning);
  emit_b(b + ".keep_leaf_yaw_only", p.keep_leaf_yaw_only);
  emit_b(b + ".enable_opening_traversal", p.enable_opening_traversal);
  emit_d(b + ".opening_traversal_path_edge_length",
         p.opening_traversal_path_edge_length);
  emit_d(b + ".opening_alignment_z_offset", p.opening_alignment_z_offset);
  emit_b(b + ".only_opening_traversal", p.only_opening_traversal);
  emit_b(b + ".auto_opening_path_approval", p.auto_opening_path_approval);
  emit_d(b + ".min_coverage_percentage", p.min_coverage_percentage);
  emit_i(b + ".max_inspection_vertices", p.max_inspection_vertices);
  emit_d(b + ".inspection_xy_spacing", p.inspection_xy_spacing);
  emit_d(b + ".inspection_z_spacing", p.inspection_z_spacing);
  emit_d(b + ".inspection_thr_esdf_dist", p.inspection_thr_esdf_dist);
  emit_d(b + ".inspection_target_viewing_range", p.inspection_target_viewing_range);
  emit_i(b + ".inspection_graph_vertices", p.inspection_graph_vertices);
  emit_i(b + ".max_exploration_iterations", p.max_exploration_iterations);
  emit_b(b + ".exploration_only", p.exploration_only);
  emit_b(b + ".basic_inspection_viewpoints", p.basic_inspection_viewpoints);
  emit_i(b + ".compartment_centers.size",
         static_cast<long long>(p.compartment_centers.size()));
  for (size_t i = 0; i < p.compartment_centers.size(); ++i)
    emitVec3(vecKey(b + ".compartment_centers", static_cast<int>(i)),
             p.compartment_centers[i]);
  emit_i(b + ".box_check_method", p.box_check_method);
  emit_i(b + ".line_check_method", p.line_check_method);
  emit_b(b + ".add_only_frontiers_to_global_graph",
         p.add_only_frontiers_to_global_graph);
  emit_b(b + ".use_flipped_yaw", p.use_flipped_yaw);
  emit_d(b + ".max_opening_height", p.max_opening_height);
  emit_d(b + ".max_surface_distance", p.max_surface_distance);
  emit_b(b + ".limit_vertices_to_surface", p.limit_vertices_to_surface);
  emit_d(b + ".min_occ_surface", p.min_occ_surface);
  emit_i(b + ".max_opening_attempts", p.max_opening_attempts);
  emit_d(b + ".global_graph_odom_dist", p.global_graph_odom_dist);
  emit_d(b + ".global_graph_odom_connect_radius",
         p.global_graph_odom_connect_radius);
  emit_b(b + ".allow_sudden_dir_change", p.allow_sudden_dir_change);
  emit_b(b + ".select_closest_frontier", p.select_closest_frontier);
  emit_d(b + ".local_navigation_reaching_radius",
         p.local_navigation_reaching_radius);
  emit_i(b + ".local_navigation_max_fail_iters",
         p.local_navigation_max_fail_iters);
  emit_d(b + ".active_homing_update_radius", p.active_homing_update_radius);
  emit_i(b + ".max_num_low_gain_iters", p.max_num_low_gain_iters);
  emit_b(b + ".freespace_cloud_enable", p.freespace_cloud_enable);
}

void declarePlanningParams(parity::Fixture& fx) {
  const std::string ns = "PlanningParams";
  fx.setParam(ns + ".type", "kBasicExploration");
  fx.setParam(ns + ".global_frame_id", "world");
  fx.setParam(ns + ".rr_mode", "kGraph");
  fx.setParam(ns + ".graph_building_mode", "kBatch");
  fx.setParam(ns + ".exp_sensor_list", std::vector<std::string>{"OS064"});
  fx.setParam(ns + ".inspection_sensor_list", std::vector<std::string>{"Cam"});
  fx.setParam(ns + ".v_max", 1.0);
  fx.setParam(ns + ".v_homing_max", 0.9);
  fx.setParam(ns + ".yaw_rate_max", 0.9);
  fx.setParam(ns + ".yaw_tangent_correction", false);
  fx.setParam(ns + ".edge_length_min", 0.2);
  fx.setParam(ns + ".edge_length_max", 4.0);
  fx.setParam(ns + ".edge_overshoot", 0.0);
  fx.setParam(ns + ".num_vertices_max", 400.0);
  fx.setParam(ns + ".num_edges_max", 7000.0);
  fx.setParam(ns + ".num_loops_cutoff", 2000.0);
  fx.setParam(ns + ".num_loops_max", 7000.0);
  fx.setParam(ns + ".nearest_range", 4.0);
  fx.setParam(ns + ".nearest_range_min", 0.2);
  fx.setParam(ns + ".nearest_range_max", 4.0);
  fx.setParam(ns + ".use_current_state", true);
  fx.setParam(ns + ".use_ray_model_for_volumetric_gain", true);
  fx.setParam(ns + ".exp_gain_voxel_size", 0.8);
  fx.setParam(ns + ".path_length_penalty", 0.01);
  fx.setParam(ns + ".path_direction_penalty", 0.2);
  fx.setParam(ns + ".occupied_voxel_gain", 0.0);
  fx.setParam(ns + ".free_voxel_gain", 0.0);
  fx.setParam(ns + ".unknown_voxel_gain", 60.0);
  fx.setParam(ns + ".global_graph_odom_dist", 1.0);
  fx.setParam(ns + ".select_closest_frontier", false);
  fx.setParam(ns + ".traverse_length_max", 5.0);
  fx.setParam(ns + ".traverse_time_max", 5000.0);
  fx.setParam(ns + ".path_safety_enhance_enable", false);
  fx.setParam(ns + ".augment_free_voxels_time", 3.0);
  fx.setParam(ns + ".free_frustum_before_planning", false);
  fx.setParam(ns + ".freespace_cloud_enable", false);
  fx.setParam(ns + ".leafs_only_for_volumetric_gain", true);
  fx.setParam(ns + ".cluster_vertices_for_gain", false);
  fx.setParam(ns + ".clustering_radius", 0.5);
  fx.setParam(ns + ".path_interpolation_distance", -1.0);
  fx.setParam(ns + ".time_budget_limit", 5000.0);
  fx.setParam(ns + ".auto_homing_enable", false);
  fx.setParam(ns + ".go_home_if_fully_explored", false);
  fx.setParam(ns + ".nonuniform_ray_cast", false);
  fx.setParam(ns + ".ray_cast_step_size_multiplier", 1.0);
  fx.setParam(ns + ".use_camera_gain", false);
  fx.setParam(ns + ".annotate_map_with_camera", true);
  fx.setParam(ns + ".inspection_planning", false);
  fx.setParam(ns + ".keep_leaf_yaw_only", true);
  fx.setParam(ns + ".enable_opening_traversal", false);
  fx.setParam(ns + ".opening_alignment_z_offset", 0.0);
  fx.setParam(ns + ".only_opening_traversal", false);
  fx.setParam(ns + ".auto_opening_path_approval", true);
  fx.setParam(ns + ".max_opening_height", 20.0);
  fx.setParam(ns + ".box_check_method", 0);
  fx.setParam(ns + ".line_check_method", 1);
  fx.setParam(ns + ".add_only_frontiers_to_global_graph", true);
  fx.setParam(ns + ".min_coverage_percentage", 0.9);
  fx.setParam(ns + ".inspection_xy_spacing", 1.0);
  fx.setParam(ns + ".inspection_z_spacing", 0.5);
  fx.setParam(ns + ".inspection_thr_esdf_dist", 0.75);
  fx.setParam(ns + ".inspection_target_viewing_range", 1.5);
  fx.setParam(ns + ".inspection_graph_vertices", 500);
  fx.setParam(ns + ".max_inspection_vertices", 100);
  fx.setParam(ns + ".max_surface_distance", 1.5);
  fx.setParam(ns + ".limit_vertices_to_surface", false);
  fx.setParam(ns + ".min_occ_surface", 0.0);
  fx.setParam(ns + ".local_navigation_reaching_radius", 2.0);
  fx.setParam(ns + ".max_exploration_iterations", 5);
  fx.setParam(ns + ".exploration_only", false);
  fx.setParam(ns + ".compartment_centers",
              std::vector<double>{0.0, -5.0, 2.0});
  fx.setParam(ns + ".compartment_dimensions.type", "kCuboid");
  fx.setParam(ns + ".compartment_dimensions.min_val",
              std::vector<double>{-7.0, -7.0, -2.5});
  fx.setParam(ns + ".compartment_dimensions.max_val",
              std::vector<double>{7.0, 7.0, 2.5});
  fx.setParam(ns + ".geofence_checking_enable", true);
}

void runPlanningParams(parity::Fixture& fx) {
  PlanningParams full;
  emit_b("planning.full.load_ok", fx.loadParams(full, "PlanningParams"));
  emitPlanningParams("planning.full", full);

  // Nothing is declared under this namespace at all.
  PlanningParams empty;
  emit_b("planning.defaults.load_ok",
         fx.loadParams(empty, "PlanningParamsNotConfigured"));
  emitPlanningParams("planning.defaults", empty);

  const PlanningModeType modes[] = {PlanningModeType::kBasicExploration,
                                    PlanningModeType::kNarrowEnvExploration,
                                    PlanningModeType::kAdaptiveExploration};
  for (int m = 0; m < 3; ++m) {
    PlanningParams p = full;
    p.setPlanningMode(modes[m]);
    emitPlanningParams("planning.mode[" + std::to_string(m) + "]", p);
  }
}


// ---------------------------------------------------------------------------
// J. One complete RRG planning iteration over the synthetic map.
//
// This is the part that makes rrg.cpp execute at all. Rrg's second constructor
// takes an externally owned MapManager, so the test can fill the synthetic TSDF
// first and then hand the same object to the planner. The four ros::Timer /
// rclcpp::TimerBase the constructor creates never fire because nothing spins,
// so the whole iteration runs synchronously and the wall-clock-budgeted global
// graph expansion (rrg.cpp's kGlobalGraphUpdateTimeBudget loop) is never
// entered - which is what makes a single iteration comparable at all.
//
// The sampler is seeded from GBP_PARITY_SEED, so batchGraph() draws the same
// sequence on both sides. Vertex IDs are never compared: they are assigned in
// the order an unordered_map is walked. Positions are compared as a sorted
// multiset instead.
// ---------------------------------------------------------------------------
void declareRrgExtraParams(parity::Fixture& fx) {
  // Rrg::loadParams hard-fails without these two, and warns without the rest.
  fx.setParam("BoundedSpaceParams.LocalSearch.type", "kCuboid");
  fx.setParam("BoundedSpaceParams.LocalSearch.min_val",
              std::vector<double>{-50.0, -50.0, -1.0});
  fx.setParam("BoundedSpaceParams.LocalSearch.max_val",
              std::vector<double>{50.0, 50.0, 1.0});
  fx.setParam("BoundedSpaceParams.LocalAdaptiveExp.type", "kCuboid");
  fx.setParam("BoundedSpaceParams.LocalAdaptiveExp.min_val",
              std::vector<double>{-10.0, -10.0, -0.75});
  fx.setParam("BoundedSpaceParams.LocalAdaptiveExp.max_val",
              std::vector<double>{10.0, 10.0, 0.75});

  const char* samplers[] = {"SamplerForSearching", "SamplerForAdaptiveExp"};
  for (const char* sm : samplers) {
    const std::string ns = std::string("RandomSamplerParams.") + sm;
    for (const char* a : {"X", "Y", "Z"}) {
      fx.setParam(ns + "." + a + ".pdf_type", "kUniform");
      fx.setParam(ns + "." + a + ".sample_mode", "kLocal");
    }
    fx.setParam(ns + ".Heading.pdf_type", "kUniform");
    fx.setParam(ns + ".Heading.sample_mode", "kManual");
    fx.setParam(ns + ".Heading.min_val", -M_PI);
    fx.setParam(ns + ".Heading.max_val", M_PI);
    fx.setParam(ns + ".Pitch.pdf_type", "kConst");
    fx.setParam(ns + ".Pitch.sample_mode", "kManual");
    fx.setParam(ns + ".Pitch.const_val", 0.0);
  }

  fx.setParam("CameraAnnotationParams.sensor_list",
              std::vector<std::string>{"Cam"});
  fx.setParam("CameraAnnotationParams.Cam.type", "kLidar");
  fx.setParam("CameraAnnotationParams.Cam.rot_lims",
              std::vector<double>{-1.55, 1.55});
  fx.setParam("CameraAnnotationParams.Cam.max_range", 2.0);
  fx.setParam("CameraAnnotationParams.Cam.center_offset",
              std::vector<double>{0.0, 0.0, 0.0});
  fx.setParam("CameraAnnotationParams.Cam.rotations",
              std::vector<double>{0.0, 0.0, 0.0});
  fx.setParam("CameraAnnotationParams.Cam.fov",
              std::vector<double>{84.0 * M_PI / 180.0, 64.0 * M_PI / 180.0});
  fx.setParam("CameraAnnotationParams.Cam.resolution",
              std::vector<double>{2.0 * M_PI / 180.0, 2.0 * M_PI / 180.0});
  fx.setParam("CameraAnnotationParams.Cam.frontier_percentage_threshold", 0.01);

  fx.setParam("AdaptiveObbParams.type", "kPca");
  fx.setParam("AdaptiveObbParams.local_pointcloud_range", 50.0);
  fx.setParam("AdaptiveObbParams.bounding_box_size_max", 35.0);

  fx.setParam("RobotDynamics.v_max", 1.0);
  fx.setParam("RobotDynamics.v_homing_max", 0.9);
  fx.setParam("RobotDynamics.yaw_rate_max", 0.9);
}

void dumpGraph(const std::string& base, GraphManager& g) {
  emit_i(base + ".num_vertices", g.getNumVertices());
  emit_i(base + ".num_edges", g.getNumEdges());

  // Sorted multiset of (x, y, z, yaw, pitch). IDs and iteration order are not
  // portable; the geometry is.
  std::vector<std::array<double, 5>> states;
  for (const auto& kv : g.vertices_map_) {
    if (!kv.second) continue;
    states.push_back({kv.second->state[0], kv.second->state[1],
                      kv.second->state[2], kv.second->state[3],
                      kv.second->state[4]});
  }
  std::sort(states.begin(), states.end());
  std::vector<double> flat;
  for (const auto& v : states)
    for (double c : v) flat.push_back(c);
  emit_darray(base + ".vertices", flat);

  // Edges as sorted pairs of sorted endpoint positions, plus their weights.
  std::vector<std::array<double, 7>> edges;
  for (const auto& kv : g.edge_map_) {
    Vertex* a = g.vertices_map_.count(kv.first) ? g.vertices_map_.at(kv.first)
                                                : nullptr;
    if (!a) continue;
    for (const auto& e : kv.second) {
      Vertex* b = g.vertices_map_.count(e.first) ? g.vertices_map_.at(e.first)
                                                 : nullptr;
      if (!b) continue;
      std::array<double, 3> pa = {a->state[0], a->state[1], a->state[2]};
      std::array<double, 3> pb = {b->state[0], b->state[1], b->state[2]};
      if (pb < pa) std::swap(pa, pb);
      edges.push_back({pa[0], pa[1], pa[2], pb[0], pb[1], pb[2], e.second});
    }
  }
  std::sort(edges.begin(), edges.end());
  edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
  std::vector<double> eflat;
  for (const auto& e : edges)
    for (double c : e) eflat.push_back(c);
  emit_i(base + ".unique_edge_count", static_cast<long long>(edges.size()));
  emit_darray(base + ".edges", eflat);

  // Gains, keyed by position rather than by id for the same reason.
  std::vector<std::array<double, 8>> gains;
  for (const auto& kv : g.vertices_map_) {
    Vertex* v = kv.second;
    if (!v) continue;
    gains.push_back({v->state[0], v->state[1], v->state[2], v->vol_gain.gain,
                     static_cast<double>(v->vol_gain.num_unknown_voxels),
                     static_cast<double>(v->vol_gain.num_free_voxels),
                     static_cast<double>(v->vol_gain.num_occupied_voxels),
                     v->vol_gain.is_frontier ? 1.0 : 0.0});
  }
  std::sort(gains.begin(), gains.end());
  std::vector<double> gflat;
  for (const auto& v : gains)
    for (double c : v) gflat.push_back(c);
  emit_darray(base + ".vertex_gains", gflat);
}


// ---------------------------------------------------------------------------
// Quaternion integrity of the poses the planner hands back.
//
// A PlannerSrv response was observed in a live ROS 2 run carrying quaternions
// whose z and w were correct while x and y held subnormal doubles around
// 1e-310 - the signature of a read of uninitialised heap memory. The functions
// that build those poses are getBestPath, getBestPathSimplified and
// getHomingPath, and all three are callable here, so the question is answered
// by measurement rather than by reading the sources.
//
// Every component is emitted three ways: as a value, as its raw IEEE-754 bits,
// and as a one-character class. A subnormal cannot hide in any of those.
// ---------------------------------------------------------------------------
char classifyDouble(double v) {
  switch (std::fpclassify(v)) {
    case FP_ZERO:
      return 'Z';
    case FP_SUBNORMAL:
      return 'S';   // the reported symptom
    case FP_NORMAL:
      return 'N';
    case FP_INFINITE:
      return 'I';
    default:
      return 'A';   // NaN
  }
}

// Dirties a few MB of heap with a pattern that reads back as a subnormal double
// (~1.5e-310, the same magnitude as the reported garbage) and frees it again.
// Anything downstream that reads uninitialised heap picks this up instead of a
// fresh zero page, which is what makes a clean result meaningful rather than
// lucky.
void poisonHeap() {
  const std::uint64_t kPoison = 0x0000ABCDEF012345ULL;
  std::vector<std::uint64_t*> blocks;
  blocks.reserve(4096);
  for (int i = 0; i < 4096; ++i) {
    std::uint64_t* b = static_cast<std::uint64_t*>(std::malloc(4096));
    if (!b) break;
    for (int k = 0; k < 512; ++k) b[k] = kPoison;
    blocks.push_back(b);
  }
  for (auto* b : blocks) std::free(b);
}

template <typename PoseVec>
void emitPoseQuaternions(const std::string& base, const PoseVec& path) {
  emit_i(base + ".size", static_cast<long long>(path.size()));
  std::string classes;
  std::string bits;
  std::vector<double> vals;
  char buf[24];
  for (const auto& p : path) {
    const double comps[4] = {p.orientation.x, p.orientation.y, p.orientation.z,
                             p.orientation.w};
    for (double c : comps) {
      classes.push_back(classifyDouble(c));
      std::uint64_t u;
      std::memcpy(&u, &c, sizeof(u));
      std::snprintf(buf, sizeof(buf), "%016llx",
                    static_cast<unsigned long long>(u));
      bits += buf;
      vals.push_back(c);
    }
  }
  emit_s(base + ".class", classes);
  emit_s(base + ".bits", bits);
  emit_darray(base + ".values", vals);
}

void runRrg(parity::Fixture& fx) {
  Rrg& rrg = fx.rrg();

  emit_b("rrg.load_params_ok", rrg.loadParams(false));
  rrg.setGlobalFrame("world");
  rrg.setBoundMode(BoundModeType::kExtendedBound);
  // setRootMode is declared in rrg.h but has no definition on either side, so
  // it is not callable; use_current_state (true) already selects the root.

  // Inside the corridor, clear of the pillar and of both walls.
  StateVec state;
  state << 1.0, 0.0, 1.0, 0.0, 0.0;

  // The first setState is treated as the first odometry and wipes the map
  // (rrg.cpp:7855-7860), so the synthetic scene has to be integrated after it,
  // not before. setState also augments a free box around the start pose, which
  // is exactly what the running planner does.
  rrg.setState(state);
  MMV& mm = *(fx.map().*gbpParityMember(MapImplTag{}));
  buildSyntheticLayer(mm.*gbpParityMember(SdfLayerTag{}),
                      "rrg.integrated_points");
  rrg.reset();

  const Rrg::GraphStatus gs = rrg.batchGraph();
  emit_i("rrg.batch.graph_status", static_cast<long long>(gs));
  dumpGraph("rrg.batch.local_graph", *(rrg.*gbpParityMember(LocalGraphTag{})));

  const Rrg::GraphStatus es = rrg.evaluateGraph();
  emit_i("rrg.batch.evaluate_status", static_cast<long long>(es));
  dumpGraph("rrg.batch.evaluated", *(rrg.*gbpParityMember(LocalGraphTag{})));

  // getBestPath assumes evaluateGraph found something; calling it otherwise
  // walks a best_vertex_ that was never set. Both sides take this branch on the
  // same status, so the comparison stays honest.
  int srv_status = -1;
  std::vector<double> wp;
  long long path_size = -1;
  // The heap is poisoned first: getBestPath is exactly the call whose result
  // becomes the PlannerSrv response path.
  poisonHeap();
  decltype(rrg.getBestPath("world", srv_status)) best_path;
  if (es == Rrg::GraphStatus::OK) {
    best_path = rrg.getBestPath("world", srv_status);
    const auto& path = best_path;
    path_size = static_cast<long long>(path.size());
    for (const auto& p : path) {
      wp.push_back(p.position.x);
      wp.push_back(p.position.y);
      wp.push_back(p.position.z);
      wp.push_back(p.orientation.x);
      wp.push_back(p.orientation.y);
      wp.push_back(p.orientation.z);
      wp.push_back(p.orientation.w);
    }
  }
  emit_i("rrg.batch.srv_status", srv_status);
  emit_i("rrg.batch.path_size", path_size);
  emit_darray("rrg.batch.path", wp);
  emitPoseQuaternions("rrg.quat.best_path", best_path);
  parity::flush();

  // The same graph, re-read through the other two producers, each preceded by a
  // heap poison so an uninitialised component cannot come back as a zero page.
  poisonHeap();
  emitPoseQuaternions("rrg.quat.best_path_simplified",
                      rrg.getBestPathSimplified());
  parity::flush();

  poisonHeap();
  emitPoseQuaternions("rrg.quat.homing_path", rrg.getHomingPath("world"));
  parity::flush();

  // Trajectory::interpolatePath is the one shared helper that constructs fresh
  // Pose objects rather than copying them, so it is the other place a partially
  // written quaternion could come from. The shipped configurations disable it
  // (path_interpolation_distance = -1), so it is called here directly.
  poisonHeap();
  {
    decltype(best_path) interp;
    Trajectory::interpolatePath(best_path, 0.25, interp);
    emitPoseQuaternions("rrg.quat.interpolated_best_path", interp);
  }
  parity::flush();
  dumpGraph("rrg.batch.global_graph",
            *(rrg.*gbpParityMember(GlobalGraphTag{})));

  // The other graph-building mode, which is a completely different code path
  // (buildGraph vs batchGraph) and the one the custom_robot config selects.
  rrg.setState(state);
  rrg.reset();
  const Rrg::GraphStatus bs = rrg.buildGraph();
  emit_i("rrg.basic.graph_status", static_cast<long long>(bs));
  dumpGraph("rrg.basic.local_graph", *(rrg.*gbpParityMember(LocalGraphTag{})));
  const Rrg::GraphStatus bes = rrg.evaluateGraph();
  emit_i("rrg.basic.evaluate_status", static_cast<long long>(bes));
  int bsrv = -1;
  long long bpath_size = -1;
  std::vector<double> bwp;
  poisonHeap();
  if (bes == Rrg::GraphStatus::OK) {
    auto bpath = rrg.getBestPath("world", bsrv);
    emitPoseQuaternions("rrg.quat.basic_best_path", bpath);
    bpath_size = static_cast<long long>(bpath.size());
    for (const auto& p : bpath) {
      bwp.push_back(p.position.x);
      bwp.push_back(p.position.y);
      bwp.push_back(p.position.z);
    }
  }
  emit_i("rrg.basic.srv_status", bsrv);
  emit_i("rrg.basic.path_size", bpath_size);
  emit_darray("rrg.basic.path", bwp);
}

}  // namespace

int parityMain(int argc, char** argv) {
  emit_s("meta.side", parity::side());
  const char* seed = std::getenv("GBP_PARITY_SEED");
  emit_s("meta.seed", seed ? seed : "<unset>");

  // Recorded, not compared: these are the axes along which the two sides are
  // allowed to differ, and having them in the result file is what turns "the
  // numbers moved" into "the numbers moved and here is the only thing that
  // changed underneath them".
  emit_s("meta.compiler", __VERSION__);
  emit_i("meta.libstdcxx", _GLIBCXX_RELEASE);
  emit_i("meta.boost_version", BOOST_VERSION);
  emit_s("meta.eigen_version",
         std::to_string(EIGEN_WORLD_VERSION) + "." +
             std::to_string(EIGEN_MAJOR_VERSION) + "." +
             std::to_string(EIGEN_MINOR_VERSION));
  emit_s("meta.pcl_version", PCL_VERSION_PRETTY);

  parity::Fixture fx(argc, argv);

  // Voxblox reads its map geometry from parameters; both sides must be told the
  // same thing or nothing downstream is comparable.
  fx.setParam("tsdf_voxel_size", 0.2);
  fx.setParam("tsdf_voxels_per_side", 16);
  fx.setParam("truncation_distance", 0.8);
  fx.setParam("max_ray_length_m", 30.0);
  fx.setParam("min_ray_length_m", 0.1);
  fx.setParam("voxel_carving_enabled", true);
  fx.setParam("max_weight", 10000.0);
  fx.setParam("use_const_weight", false);
  fx.setParam("use_weight_dropoff", true);
  fx.setParam("use_symmetric_weight_dropoff", false);
  fx.setParam("allow_clear", true);
  fx.setParam("method", "simple");
  fx.setParam("verbose", false);
  fx.setParam("world_frame", "world");
  fx.setParam("update_mesh_every_n_sec", 0.0);
  fx.setParam("publish_map_every_n_sec", 0.0);
  fx.setParam("occupancy_distance_voxelsize_factor", 1.0);

  declareSensorParams(fx);
  declareSpaceParams(fx);
  declareSamplerParams(fx);
  declarePlanningParams(fx);
  declareRrgExtraParams(fx);
  fx.realize();

  runKdtree();
  runTrajectory();
  runGraph();
  runGeofence();

  SensorParams sensors;
  runSensorParams(fx, sensors);

  BoundedSpaceParams global_space, local_space;
  runSpaceParams(fx, global_space, local_space);

  runPlanningParams(fx);
  runRandomSampler(fx, global_space, local_space);
  runMapManager(fx, sensors);
  runRrg(fx);

  emit_i("meta.done", 1);
  return 0;
}
