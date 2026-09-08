# `gbplanner_node` — external interface (ROS 2 Jazzy)

Executable: `ros2 run gbplanner gbplanner_node`
Node name: `gbplanner_node` (hard-coded in `src/gbplanner_ros_node.cpp`).
Executor: `rclcpp::executors::SingleThreadedExecutor`, every callback in the node's
default (mutually exclusive) callback group.

All names below are given as the C++ code spells them. Relative names resolve
against the node's namespace; names starting with `/` are absolute and are **not**
affected by a launch-file namespace. Every publisher and subscription in this
package uses the default `rclcpp::QoS(KeepLast(depth))` — reliable, volatile.
There were **no latched publishers in the ROS 1 source**, so nothing here needs
`transient_local`.

One node process hosts three sets of interfaces:

* **A** — `GbplannerRos` / `Gbplanner` / `Rrg`, i.e. this package.
* **B** — `voxblox::TsdfServer`, constructed by `MapManager` on the *same* node.
  Listed here because it is part of the node's observable surface and the launch
  files have to remap it, but it is owned by `map_manager` / `voxblox_ros`.

---

## 1. Published topics

### 1.A Planner (this package)

| Topic | Type | QoS | Source |
|---|---|---|---|
| `planner_control_interface/msg/reset` | `std_msgs/msg/Bool` | KeepLast(10) | `rrg.cpp` — published when auto-landing engages |
| `gbplanner/local_free_cloud` | `sensor_msgs/msg/PointCloud2` | KeepLast(10) | created, never published (dead in ROS 1 too) |
| `/gbplanner_path` | `nav_msgs/msg/Path` | KeepLast(10) | `Rrg::queryCallback` (debug service only). **Absolute name.** |
| `freespace_pointcloud` | `sensor_msgs/msg/PointCloud2` | KeepLast(10) | `Rrg::freePointCloudtimerCallback` — the timer that would drive it is never created, so this is dead in ROS 1 and here |
| `gbplanner/entry_point_viz` | `geometry_msgs/msg/PoseStamped` | KeepLast(10) | created, never published |
| `gbplanner/local_target_viz` | `geometry_msgs/msg/PointStamped` | KeepLast(10) | `Rrg::evaluateLocalNavigationPath` |
| `gbp_time_log` | `std_msgs/msg/Float32MultiArray` | KeepLast(10) | `Rrg::publishTimings` |
| `gbplanner/homing_local_goal` | `geometry_msgs/msg/PoseStamped` | KeepLast(10) | `Gbplanner::updateHomingGoal` / `updateGlobalGoal` |

### 1.B Visualization (`Visualization`, all in `gbplanner_rviz.cpp`)

Frame id of every marker is `world_frame_id`, defaulting to `"world"` and
overwritten at runtime by `Rrg::setGlobalFrame()` with the `header.frame_id` of
the incoming `PlannerSrv` request.

| Topic | Type | QoS |
|---|---|---|
| `vis/planning_workspace` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/no_gain_zone` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/planning_graph` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/planning_projected_graph` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/planning_failed` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/shortest_paths` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/robot_state` | `visualization_msgs/msg/MarkerArray` | KeepLast(100) |
| `vis/sensor_fov` | `visualization_msgs/msg/MarkerArray` | KeepLast(100) |
| `vis/best_planning_paths` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/volumetric_gains` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/sampler` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/ray_casting` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/ref_path` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/imp_ref_path` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/planning_global_graph` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/planning_homing_path` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/planning_global_path` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/shortest_path_clustering` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/state_history` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/geofence` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/negative_edges` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/hyperplanes` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/mod_path` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/blind_mod_path` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/alternate_path` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/opening_traversal_path` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/graph_vertices` | `visualization_msgs/msg/MarkerArray` | KeepLast(10) |
| `vis/occupied_pcl` | `sensor_msgs/msg/PointCloud2` | KeepLast(10) |
| `vis/viewpoints` | `geometry_msgs/msg/PoseArray` | KeepLast(10) |

Every one of these is guarded by `get_subscription_count() < 1` — nothing is
serialised until an rviz client subscribes.

### 1.C voxblox (owned by `map_manager`)

`voxblox_ros` builds these names with `<node name>/<topic>`, i.e. they land under
`/gbplanner_node/...`, **not** in the root namespace the way ROS 1 put them.

| Topic | Type |
|---|---|
| `~/surface_pointcloud` | `sensor_msgs/msg/PointCloud2` |
| `~/tsdf_pointcloud` | `sensor_msgs/msg/PointCloud2` |
| `~/occupied_nodes` | `visualization_msgs/msg/MarkerArray` |
| `~/tsdf_slice` | `sensor_msgs/msg/PointCloud2` |
| `~/mesh` | `voxblox_msgs/msg/Mesh` |
| `~/tsdf_map_out` | `voxblox_msgs/msg/Layer` |
| `~/icp_transform` | `geometry_msgs/msg/TransformStamped` (only when `enable_icp`) |
| `~/esdf_pointcloud`, `~/esdf_slice`, `~/traversable`, `~/esdf_map_out` | ESDF server only; this build uses the TSDF server (`#define use_tsdf` in `map_manager_voxblox_impl.h`) |

---

## 2. Subscribed topics

### 2.A Planner (this package)

| Topic | Type | QoS | Handler |
|---|---|---|---|
| `pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | KeepLast(100) | `Gbplanner::poseCallback` |
| `pose_stamped` | `geometry_msgs/msg/PoseStamped` | KeepLast(100) | `Gbplanner::poseStampedCallback` |
| `odometry` | `nav_msgs/msg/Odometry` | KeepLast(100) | `Gbplanner::odometryCallback` — the only state source the sim launch remaps |
| `/robot_status` | `planner_msgs/msg/RobotStatus` | KeepLast(1) | `Gbplanner::robotStatusCallback` (battery time). **Absolute name.** |
| `/traversability_estimation/untraversable_polygon` | `geometry_msgs/msg/PolygonStamped` | KeepLast(100) | `Gbplanner::untraversablePolygonCallback`. **Absolute name.** Can be dropped/recreated at runtime via `gbplanner/enable_untraversable_polygon_subscriber` |
| `local_navigation_goal` | `geometry_msgs/msg/PoseStamped` | KeepLast(100) | `Gbplanner::localNavGoalCallback`; the sim launch remaps `/move_base_simple/goal` onto it |
| `planner_control_interface/stop_request` | `std_msgs/msg/Bool` | KeepLast(5) | `Gbplanner::stopMsgCallback` |
| `planner_control_interface/stop_request` | `std_msgs/msg/Bool` | KeepLast(100) | `Rrg::stopMsgCallback` — a **second** subscription to the same topic, as in ROS 1 |
| `semantic_location` | `planner_semantic_msgs/msg/SemanticPoint` | KeepLast(100) | `Rrg::semanticsCallback` |
| `opening_detections` | `planner_msgs/msg/MultipleOpeningDetections` | KeepLast(100) | `Rrg::openingDetectionCallback` |
| `query_point` | `geometry_msgs/msg/PoseStamped` | KeepLast(1) | `Rrg::queryPtCallback` (debug) |
| `cam_pitch` | `sensor_msgs/msg/JointState` | KeepLast(1) | `Rrg::camPitchCallback` |
| `elevation_map` | `grid_map_msgs/msg/GridMap` | KeepLast(1) | `Rrg::eleMapCallback` — ground robots only |

### 2.B voxblox (owned by `map_manager`)

| Topic | Type | Note |
|---|---|---|
| `~/pointcloud` | `sensor_msgs/msg/PointCloud2` | depth = `pointcloud_queue_size` (5). **The main sensor input.** Under ROS 1 it was `/pointcloud`; the port made it node-private, so the launch pass must remap `/gbplanner_node/pointcloud` |
| `~/freespace_pointcloud` | `sensor_msgs/msg/PointCloud2` | only when `use_freespace_pointcloud: True`. **Note:** this package's own `freespace_pointcloud` publisher uses the *relative* name and therefore does not connect to it any more; it is dead code in both ROS 1 and ROS 2, so nothing breaks, but do not assume the loop exists |
| `~/tsdf_map_in` | `voxblox_msgs/msg/Layer` | |
| `~/transform` | `geometry_msgs/msg/TransformStamped` | only when `use_tf_transforms: False` |

`voxblox_ros` creates these with default `rclcpp::QoS(KeepLast(n))` (reliable);
if the sensor driver publishes best-effort, the launch pass has to reconcile that
— it is a property of `voxblox_ros`, not of this package.

---

## 3. Services offered

### 3.A Behaviour-tree front end (`GbplannerRos`)

| Service | Type | Note |
|---|---|---|
| `gbplanner_ros` | `planner_msgs/srv/PlannerSrv` | **The PCI entry point.** Ticks the behaviour tree once and returns its path. Runs for seconds. |
| `gbplanner_ros_homing` | `planner_msgs/srv/PlannerHoming` | Sets `bt_states_.homing_required`; leaves `path` empty (the body that filled it is commented out in the original) |

### 3.B `Gbplanner`

| Service | Type |
|---|---|
| `gbplanner` | `planner_msgs/srv/PlannerSrv` |
| `gbplanner/global` | `planner_msgs/srv/PlannerGlobal` |
| `gbplanner/homing` | `planner_msgs/srv/PlannerHoming` |
| `gbplanner/set_homing_pos` | `planner_msgs/srv/PlannerSetHomingPos` |
| `gbplanner/search` | `planner_msgs/srv/PlannerSearch` |
| `gbplanner/passing_gate` | `planner_msgs/srv/PlannerRequestPath` |
| `gbplanner/set_global_bound` | `planner_msgs/srv/PlannerSetGlobalBound` |
| `gbplanner/set_dynamic_global_bound` | `planner_msgs/srv/PlannerDynamicGlobalBound` |
| `gbplanner/clear_untraversable_zones` | `std_srvs/srv/Trigger` |
| `gbplanner/load_graph` | `planner_msgs/srv/PlannerStringTrigger` |
| `gbplanner/save_graph` | `planner_msgs/srv/PlannerStringTrigger` |
| `gbplanner/go_to_waypoint` | `planner_msgs/srv/PlannerGoToWaypoint` |
| `gbplanner/get_frontiers` | `planner_msgs/srv/PlannerGetFrontiers` |
| `gbplanner/get_target_costs` | `planner_msgs/srv/PlannerGetTargetCosts` |
| `gbplanner/validate_frontiers` | `planner_msgs/srv/PlannerValidateFrontiers` |
| `gbplanner/enable_untraversable_polygon_subscriber` | `std_srvs/srv/SetBool` |
| `gbplanner/set_planning_trigger_mode` | `planner_msgs/srv/PlannerSetPlanningMode` |
| `gbplanner/get_inspection_path` | `planner_msgs/srv/PlannerSrv` |
| `gbplanner/force_compartment_transition` | `std_srvs/srv/Trigger` |
| `gbplanner/switch_operation_mode` | `std_srvs/srv/SetBool` |

`planner_geofence_service_` exists as a member in ROS 1 but was never advertised
and its callback was never defined; it is not advertised here either.

### 3.C `Rrg`

| Service | Type |
|---|---|
| `gbplanner/reset_timer` | `std_srvs/srv/Trigger` |
| `get_opening_traversal_path` | `std_srvs/srv/Trigger` |
| `approve_opening_traversal` | `planner_msgs/srv/PlannerOpeningApproval` |
| `reset_map` | `std_srvs/srv/Trigger` |
| `query_srv` | `std_srvs/srv/Trigger` |
| `remove_geofence` | `planner_msgs/srv/PlannerSetPlanningMode` |

### 3.D voxblox (owned by `map_manager`)

`~/generate_mesh`, `~/clear_map`, `~/publish_pointclouds`, `~/publish_map`
(`std_srvs/srv/Empty`); `~/save_map`, `~/load_map` (`voxblox_msgs/srv/FilePath`).

---

## 4. Service clients

| Service called | Type | Note |
|---|---|---|
| `land_srv` | `std_srvs/srv/Empty` | Fire-and-forget (`async_send_request` with an empty completion callback). Fired once when the auto-landing time budget expires. Meant to be remapped by the launch file. |
| `planner_control_interface/std_srvs/homing_trigger` | `std_srvs/srv/Trigger` | Client is created but never called (same in ROS 1) |

No actions are offered or consumed.

---

## 5. Parameters

All parameter names use `.` separators. The node declares each one lazily through
`getParamOpt()` (planner_common) with `dynamic_typing` and an *unset* default, so
an absent key leaves the C++ default in place instead of zeroing it. A YAML file
therefore only has to supply the keys it wants to override, and `/**:` works.

`gbplanner_config.yaml` is the file that supplies group 5.1–5.7;
`voxblox_sim_config.yaml` supplies 5.9. Both are loaded onto the same node.

### 5.1 Top-level (read by this package directly)

| Name | Type | Default | Supplied by |
|---|---|---|---|
| `behavior_tree_path` | string | `<share>/gbplanner/config/bt_xml/main_tree.xml` | launch file (`<param>` in ROS 1) — not present in any config yaml |
| `tree_path` | string | `""` | nothing; read into a variable that is only logged |

### 5.2 `RobotParams.*` (`RobotParams::loadParams`) — gbplanner_config.yaml

`type` (string, `kAerialRobot` / `kGroundRobot`), `size` (double[3]),
`size_extension_min` (double[3]), `size_extension` (double[3]),
`center_offset` (double[3]), `relax_ratio` (double), `bound_mode` (string,
`kExtendedBound` / `kRelaxedBound` / `kMinBound` / `kExactBound` / `kNoBound`),
`safety_extension` (double[3]), `footprint` (double[2], ground robots).

### 5.3 `SensorParams.*` and `CameraAnnotationParams.*` and `FreeFrustumParams.*`

`<group>.sensor_list` (string[]), then for each entry `<name>` in that list:

`type` (string `kCamera`/`kLidar`/`kSpherical`), `CameraType` (string
`kFixed`/`kRotating`/`kZoom`/`kRotatingZoom`), `min_range`, `max_range` (double),
`center_offset` (double[3]), `rotations` (double[3], rad),
`fov` (double[2], rad), `resolution` (double[2], rad), `rot_lims` (double[2], rad),
`frontier_percentage_threshold` (double), `sensor_frame`, `frame_id`,
`callback_topic`, `focal_length_topic` (strings), `height`, `width`,
`height_removal`, `width_removal` (int).

`SensorParams` and `CameraAnnotationParams` come from gbplanner_config.yaml.
`FreeFrustumParams` is absent from every shipped config; its `loadParams`
failing is expected and forces `PlanningParams.freespace_cloud_enable = false`.

### 5.4 `BoundedSpaceParams.<Global|Local|LocalSearch|LocalAdaptiveExp>.*` and `NoGainZones.<zone>.*`

`type` (string `kCuboid`/`kSphere`), `min_val`/`max_val` (double[3]),
`min_extension`/`max_extension` (double[3]), `rotations` (double[3]),
`radius`, `radius_extension` (double). All from gbplanner_config.yaml.
The zone names under `NoGainZones` come from `PlanningParams.no_gain_zones_list`.

### 5.5 `RandomSamplerParams.<SamplerForExploration|SamplerForSearching|SamplerForAdaptiveExp>.<X|Y|Z|Heading|Pitch>.*`

`pdf_type` (string `kConst`/`kUniform`/`kNormal`/`kCauchy`/`kNormalUniform`),
`sample_mode` (string `kManual`/`kLocal`/`kGlobal`/`kExploredMap`/`kIgnore`),
`min_val`, `max_val`, `mean_val`, `std_val`, `const_val` (double).
From gbplanner_config.yaml.

### 5.6 `PlanningParams.*` — gbplanner_config.yaml

Strings: `global_frame_id` (default `world`), `type`, `rr_mode`,
`graph_building_mode`.
String arrays: `exp_sensor_list`, `inspection_sensor_list`, `no_gain_zones_list`.
Booleans: `yaw_tangent_correction`, `use_current_state`, `geofence_checking_enable`,
`interpolate_projection_distance`, `augment_free_frustum_en`,
`free_frustum_before_planning`, `use_ray_model_for_volumetric_gain`,
`leafs_only_for_volumetric_gain`, `cluster_vertices_for_gain`,
`nonuniform_ray_cast`, `planning_backward`, `path_safety_enhance_enable`,
`auto_global_planner_enable`, `go_home_if_fully_explored`, `auto_homing_enable`,
`homing_backward`, `auto_landing_enable`, `use_camera_gain`,
`annotate_map_with_camera`, `inspection_planning`, `keep_leaf_yaw_only`,
`enable_opening_traversal`, `only_opening_traversal`,
`auto_opening_path_approval`, `exploration_only`, `basic_inspection_viewpoints`,
`add_only_frontiers_to_global_graph`, `use_flipped_yaw`,
`limit_vertices_to_surface`, `allow_sudden_dir_change`,
`select_closest_frontier`, `freespace_cloud_enable`.
Integers: `max_inspection_vertices`, `inspection_graph_vertices`,
`max_exploration_iterations`, `box_check_method`, `line_check_method`,
`max_opening_attempts`, `local_navigation_max_fail_iters`,
`max_num_low_gain_iters`.
Doubles: `v_max`, `v_homing_max`, `yaw_rate_max`, `edge_length_min`,
`edge_length_max`, `edge_overshoot`, `num_vertices_max`, `num_edges_max`,
`num_loops_cutoff`, `num_loops_max`, `nearest_range`, `nearest_range_min`,
`nearest_range_max`, `max_ground_height`, `robot_height`, `max_inclination`,
`augment_free_voxels_time`, `exp_gain_voxel_size`, `free_voxel_gain`,
`occupied_voxel_gain`, `unknown_voxel_gain`, `path_length_penalty`,
`path_direction_penalty`, `hanging_vertex_penalty`, `clustering_radius`,
`ray_cast_step_size_multiplier`, `traverse_length_max`, `traverse_time_max`,
`path_interpolation_distance`, `relaxed_corridor_multiplier`,
`time_budget_limit`, `time_budget_before_landing`,
`opening_traversal_path_edge_length`, `opening_alignment_z_offset`,
`min_coverage_percentage`, `inspection_xy_spacing`, `inspection_z_spacing`,
`inspection_thr_esdf_dist`, `inspection_target_viewing_range`,
`max_opening_height`, `max_surface_distance`, `min_occ_surface`,
`global_graph_odom_dist`, `global_graph_odom_connect_radius`,
`local_navigation_reaching_radius`, `active_homing_update_radius`.
Nested: `PlanningParams.compartment_dimensions.*` (a `BoundedSpaceParams` group),
`PlanningParams.compartment_centers` (double[], flattened triples).

### 5.7 `AdaptiveObbParams.*` — gbplanner_config.yaml, read by `adaptive_obb`

`type` (string `kPca`/`kMvbb`/`kAabb`), `local_pointcloud_range`,
`bounding_box_size_max`, `distribution_scaling_max`, `voxel_filter_leaf_size`
(double), and for the MVBB variant `max_iter_genetic_algorithm`,
`max_iter_nelder_mead`, `population_size`, `tol_termination`, `tol_iter`.

### 5.8 Groups no shipped config supplies (defaults apply)

* `RobotDynamics.{v_max,v_homing_max,yaw_rate_max}` (double)
* `GeofenceParams.AreaList` (string[]) and `GeofenceParams.<area>.{center,size}` (double[])
* `DarpaGateParams.{enable,load_from_darpa_frame,darpa_frame_offset,world_frame_id,gate_center_frame_id,center,heading,center_search_radius,center_search_step,line_search_length,line_search_range,line_search_step,line_length}`

`DarpaGateParams.enable` defaults to false, which is what disables
`gbplanner/passing_gate`.

### 5.9 voxblox / map_manager parameters — voxblox_sim_config.yaml

Read by `voxblox_ros` on the same node, so they must be loaded onto
`gbplanner_node`: `world_frame`, `sensor_frame`, `use_tf_transforms`,
`use_freespace_pointcloud`, `tsdf_voxels_per_side`, `tsdf_voxel_size`,
`max_ray_length_m`, `min_ray_length_m`, `truncation_distance`,
`voxel_carving_enabled`, `use_sparsity_compensation_factor`,
`sparsity_compensation_factor`, `color_mode`, `verbose`,
`update_mesh_every_n_sec`, `mesh_min_weight`, `slice_level`, `method`,
`integration_order_mode`, `max_consecutive_ray_collisions`, `publish_slices`,
`publish_pointclouds`, `publish_traversable`, `traversability_radius`,
`allow_clear`, `pointcloud_queue_size`, `min_time_between_msgs_sec`,
`enable_icp`, `icp_refine_roll_pitch`, `accumulate_icp_corrections`,
`timestamp_tolerance_sec`, `publish_tsdf_map`, `publish_esdf_map`,
`tsdf_surface_distance_threshold_factor`, `esdf_max_distance_m`,
`clear_sphere_for_planning`, `clear_sphere_radius`, `use_const_weight`,
`use_weight_dropoff`, `max_weight`, `clearing_ray_weight_factor`,
`weight_ray_by_range`, `occupancy_min_distance_voxel_size_factor`.

Plus `occupancy_distance_voxelsize_factor` (double), which `map_manager` reads
itself.

### 5.10 Standard rclcpp parameters

`use_sim_time` (bool). The mission clock (`Rrg::rostime_start_`,
`getTimeElapsed()`) and every `header.stamp` come from `node->now()` and are
therefore sim-time aware. The four planner timers are **wall** timers and are
not.

---

## 6. TF frames

**This package publishes no transforms.** It owns one `tf2_ros::Buffer` +
`tf2_ros::TransformListener`, created in the `Rrg` constructor; the listener
spins its own node (`/transform_listener_impl_<hex>` shows up in `ros2 node
list`) on its own thread, so `lookupTransform(..., timeout)` never waits on this
node's executor.

`ros2 node info /gbplanner_node` does list `/tf` under *Publishers*: that
broadcaster belongs to `voxblox::TsdfServer` (`tf_broadcaster_`, used for the
ICP-corrected frame) and only sends anything when `enable_icp` is true — it is
false in every shipped config.

| Lookup | Target frame | Source frame | Where |
|---|---|---|---|
| dynamic global bound | `PlanningParams.global_frame_id`, overwritten at runtime by `Rrg::setGlobalFrame` | `header.frame_id` of the `PlannerDynamicGlobalBound` request | `Rrg::setGlobalBound` |
| untraversable polygon | `PlanningParams.global_frame_id` | `header.frame_id` of the incoming `PolygonStamped` | `Rrg::addGeofenceAreas`, 0.1 s timeout; skipped entirely when the two frames match |
| DARPA gate | `DarpaGateParams.world_frame_id` (default `world`) | `DarpaGateParams.gate_center_frame_id` (default `darpa`) | `Rrg::searchPathToPassGate`, 1.0 s timeout; only when `DarpaGateParams.enable` |
| voxblox pointcloud | `world_frame` (voxblox param, `world`) | `sensor_frame` if set, else the pointcloud's `header.frame_id` | `voxblox::Transformer` |

Frames only *written* into message headers, never looked up:

* `PlanningParams.global_frame_id` — markers, `/gbplanner_path`,
  `gbplanner/local_target_viz`, `gbplanner/homing_local_goal`, and the
  `PlannerGetFrontiers` / `PlannerGetTargetCosts` response headers.
* `SensorParams.<name>.frame_id` — stamped on `freespace_pointcloud`.

---

## 7. Behaviour tree

`registerTree()` registers 24 node types and loads `behavior_tree_path`, falling
back to `ament_index_cpp::get_package_share_directory("gbplanner") +
"/config/bt_xml/main_tree.xml"`. Registered ids:

`LocalExploration`, `GlobalExploration`, `LocalExpExhaustedCheck`,
`GlobalExpExhaustedCheck`, `Inspection`, `CompartmentTransition`, `Homing`,
`HomingCheck`, `OPENINGPhase1`, `OPENINGPhaseCheck`, `OPENINGPhase2`,
`LocalExpExhaustedReset`, `Idle`, `OPENINGP1FailCheck`, `SetNextCompartment`,
`AllCompartmentsInspectedCheck`, `LocalNavigation`,
`LocalNavigationExhaustedCheck`, `LocalNavigationExhaustedReset`,
`CalculateHomingPath`, `UpdateHomingGoal`, `SwitchToLocalNavigation`,
`CalculateGlobalPath`, `UpdateGlobalGoal`.

The tree is created with `factory_.createTree("MainTree")`, so every
`main_tree.xml` must define `<BehaviorTree ID="MainTree">`. Includes inside the
XML are resolved relative to the including file, so per-scenario trees under
`config/<scenario>/main_tree.xml` reach the shared subtrees with
`../../../bt_xml/...` — that only works from the **installed** share directory,
which has the same layout as the source tree.

---

## 8. Timers

| Period | Callback | Clock |
|---|---|---|
| 0.25 s | `Rrg::timerCallback` — exploring direction, odometry-driven global graph growth, auto-landing check | wall |
| 0.50 s | `Rrg::expandGlobalGraphTimerCallback` — samples into the global graph under a 0.1 s budget | wall |
| 1.00 s | `Rrg::expandGlobalGraphFrontierAdditionTimerCallback` | wall |
| 0.10 s | `Rrg::cameraAnnotationTimerCallback` — no-op unless `PlanningParams.annotate_map_with_camera` | wall |

`Rrg::freePointCloudtimerCallback` exists but no timer is created for it, exactly
as in ROS 1.

---

## 9. Notes for the launch pass

1. **Node name is fixed** to `gbplanner_node` in `main()`. `voxblox_ros` derives
   its topic names from `node->get_name()`, so renaming the node in a launch file
   also moves `~/pointcloud` and friends.
2. **Both YAML files go on this one node.** `gbplanner_config.yaml` and
   `voxblox_sim_config.yaml` are `/**:`-scoped, so `parameters=[cfg1, cfg2]` on a
   single `Node` action is the whole story. `behavior_tree_path` is a plain node
   parameter now (`-p behavior_tree_path:=...` or a third entry in `parameters`),
   not a `<param>` child.
3. **The sensor input moved.** ROS 1 remapped `/pointcloud`; the ROS 2 voxblox
   port makes it node-private, so the remap target is now
   `/gbplanner_node/pointcloud`.
4. **ROS 1 remaps that carry over unchanged:** `odometry`,
   `local_navigation_goal` (from `/move_base_simple/goal`), and — on the PCI side
   — `planner_server` → `gbplanner_ros`, `planner_homing_server` →
   `gbplanner/homing`.
5. **`land_srv`** is a client, not a server; remap it at the gbplanner node.
6. The executable calls `rclcpp::remove_ros_arguments()` before handing the rest
   of `argv` to gflags, so `--ros-args ...` is accepted; any other `--flag` is
   still parsed by gflags and will abort on an unknown name.
