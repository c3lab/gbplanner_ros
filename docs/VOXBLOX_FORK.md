# Why gbplanner uses a fork of voxblox-ros2

`vcstool/image_deps.repos` pins `GabrieleSantangelo/voxblox-ros2` at branch
**`gbplanner`**, not `master`. This file records why, because both differences
below fail *silently* — the planner keeps running and produces plausible-looking
paths through solid rock.

## The mismatch

| | branch |
|---|---|
| ROS 1 gbplanner builds against | `ntnu-arl/voxblox` @ `feature/esdf_colorization` |
| that branch sits on top of | upstream `ethz-asl/voxblox` @ `feature/new_abort_criterion_for_fast_integrator` |
| `voxblox-ros2` was forked from | upstream `ethz-asl/voxblox` @ **`master`** |

So the ROS 2 port is missing the whole upstream integrator-rework branch *plus*
the ntnu commits on top of it. It is not a ROS 1 → ROS 2 API difference; it is a
different integrator.

## What actually breaks

### 1. Clearing rays carve obstacles out of the map

`TsdfIntegratorBase::Config` on the ROS 1 side has three fields that master does
not (`voxblox/include/voxblox/integrator/tsdf_integrator.h:74,77,90`):

```
float clearing_ray_weight_factor = 1.0;
bool  weight_ray_by_range        = false;
bool  use_symmetric_weight_dropoff = false;
```

Every shipped gbplanner config sets the first one:

```
gbplanner/config/uav/gz/cave_exploration/voxblox_sim_config.yaml:47
    clearing_ray_weight_factor: 0.01   # ANYmal: 0.01  Flyab: 0.05
```

Meaning: a free-space (clearing) sample carries 1 % of a surface sample's weight,
so surfaces are ~100× harder to erase. On unmodified `voxblox-ros2` the parameter
does not exist, so it is ignored, clearing rays carry full weight, and obstacles
get carved away. `map_manager`'s `getBoxStatus` / `getPathStatus` then return
`kFree` **through walls** — with no error, no warning, and a map that looks fine
in RViz.

`getVoxelWeight` also differs: the ROS 1 version returns `1.0` unless
`weight_ray_by_range` is set, master always returns `1/z²`.

### 2. Colour blending destroys the camera-annotation flag

`map_manager` does not use `TsdfVoxel::color` as a colour. It uses `color.r` as a
one-bit "this voxel has been seen by the annotation camera" flag:

```
map_manager/src/voxblox/voxblox_common_impl.cpp:740   voxel->color.r = 255;   // write
map_manager/src/voxblox/voxblox_common_impl.cpp:786,797,894,921
                                                      if (voxel->color.r < 255)  // read
```

ntnu commit `bf023f0` comments the colour-blend block out of `updateTsdfVoxel`
precisely so this survives. On `voxblox-ros2` master the block is live
(`voxblox/src/integrator/tsdf_integrator.cc:201-204`), and the incoming colour is
not neutral: for a `PointXYZI` lidar cloud, `convertColor` maps intensity through
a colour map. So every later lidar integration drags an annotated voxel's
`color.r` back below 255, previously-inspected voxels re-read as unknown, and the
inspection / semantic gain never converges.

## What the fork changes

Branch `gbplanner`, one commit on top of `243dc3e`:

| File | Change |
|---|---|
| `voxblox/include/voxblox/integrator/tsdf_integrator.h` | replaced wholesale with the ntnu version |
| `voxblox/src/integrator/tsdf_integrator.cc` | replaced wholesale with the ntnu version |
| `voxblox_ros/include/voxblox_ros/ros_params.h` | declare/get the three weighting params; drop the sparsity pair, which the ntnu integrator does not have |
| `voxblox/include/voxblox/core/voxel.h` | `EsdfVoxel` gains `Color color` (ntnu `719b5e6`) |
| `voxblox/include/voxblox/core/common.h` | `Color()` default alpha 0 → 255 (ntnu `719b5e6`) |
| `voxblox_ros/package.xml` | drop `<depend>voxblox_rviz_plugin</depend>` (ntnu `d1db0e1`) |

The two integrator files contain **no ROS API at all** — verified by grep for
`ros::`, `ROS_`, `rclcpp` and by diffing their include lists, which are identical
between the two checkouts. That is why they drop in unchanged rather than needing
their own port.

The sparsity-compensation pair is dropped rather than kept because it is the
mirror-image trap: `use_sparsity_compensation_factor: True` and
`sparsity_compensation_factor: 100.0` are **dead** in ROS 1 (no such field on that
`Config`) but **live** on master. With `max_weight: 50`, a factor of 100 saturates
near-surface voxels in a single measurement and freezes the TSDF surface.

## Verification

`colcon build` of the branch on `ros:jazzy`: 9 packages, exit 0 (warnings only —
Eigen `-Wmaybe-uninitialized`, protobuf syntax notes).

Behavioural checks worth running once `map_manager` is ported:

1. after `annotateCameraVoxels`, a voxel's `color.r` must still be 255 following
   further lidar integration;
2. `getPathStatus` across a known wall must return `kOccupied`.
