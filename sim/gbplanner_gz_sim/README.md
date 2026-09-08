# gbplanner_gz_sim

Gazebo Harmonic (gz-sim 8) simulation for the gbplanner ROS 2 stack: worlds, the
RMF-Owl model, the `ros_gz_bridge` wiring and one launch file.

It exists to give the planner the four things it cannot run without -- odometry, a
lidar `sensor_msgs/PointCloud2`, TF and `/clock` -- and to fly the path the
planner produces. The ROS 1 stack ran Gazebo Garden through `arl_gazebo_sim`;
Jazzy pairs with Harmonic, so this half was written rather than translated.

## Quick start

```bash
# Self-contained: no external assets.
ros2 launch gbplanner_gz_sim gz_sim.launch.py world:=cave_box

# The DARPA SubT cave. Needs the subt_cave_sim models (see Assets).
ros2 launch gbplanner_gz_sim gz_sim.launch.py \
    world:=darpa_cave_01 x:=40.0 y:=5.0 z:=1.5 \
    resource_path:=/path/to/subt_cave_sim/models
```

`headless:=false` brings up the gz GUI; `rviz:=true` adds an RViz view of the
raw simulation (TF, point cloud, odometry, `/gbplanner_path`).

## What it publishes

With the default `robot_name:=rmf_owl`:

| Topic | Type | Rate | Note |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | 1 kHz | one per physics step; set `use_sim_time` everywhere |
| `/rmf_owl/odometry` | `nav_msgs/Odometry` | 50 Hz | `world` -> `rmf_owl/base_link`, ground truth |
| `/rmf_owl/lidar/points` | `sensor_msgs/PointCloud2` | 10 Hz nominal | `frame_id: rmf_owl/laser_link`, 2048x128. 2048x128 rays per sweep is what the ROS 1 sim asked for and it does not keep up on a laptop GPU; measured 6-8 Hz. Lower it with `lidar_horizontal_samples:=` / `lidar_vertical_samples:=`. |
| `/rmf_owl/imu` | `sensor_msgs/Imu` | 200 Hz | |
| `/tf` | `tf2_msgs/TFMessage` | 50 Hz | `world` -> *world name* -> `rmf_owl` -> links |

and subscribes:

| Topic | Type | Note |
|---|---|---|
| `/gbplanner_path` | `nav_msgs/Path` | what `pci_general` publishes; the follower flies it |
| `/rmf_owl/command/velocity` | `geometry_msgs/Twist` | body frame, into gz |
| `/rmf_owl/enable` | `std_msgs/Bool` | arms the gz velocity controller |

`robot_name` is substituted into the model SDF, so it is simultaneously the gz
entity name, the prefix of every gz topic, the ROS namespace of the bridged
topics and the TF frame prefix. Two robots under one gz server differ only in
that argument.

## Joining it to `gbplanner`

`gz_sim.launch.py` is the include point. Its full argument list is in the
module docstring (`ros2 launch gbplanner_gz_sim gz_sim.launch.py --show-args`).
On the planner side, `gbplanner_node` needs:

```
remap  odometry        ->  /rmf_owl/odometry
remap  ~/pointcloud    ->  /rmf_owl/lidar/points     # node-private in ROS 2
param  use_sim_time    :=  true
```

`voxblox`'s `world_frame: world` and `PlanningParams.global_frame_id: world`
already match what this package publishes; nothing in `gbplanner/config` has to
change.

## Assets

`worlds/darpa_cave_01.sdf` has 69 `<include>` blocks naming 17 distinct
`model://Cave ...` tile and artifact models that are **not** in this repository. They are the `subt_cave_sim` package: 3.9 GB behind
git-lfs. Provide them in whichever way suits the deployment and point
`resource_path:=` (or `GZ_SIM_RESOURCE_PATH`) at the `models` directory:

* **bind mount** -- the ROS 1 checkout already has them:
  `-v /path/to/gbplanner_ros/bootstrap/sim/subt_cave_sim:/opt/subt_cave_sim:ro`,
  then `resource_path:=/opt/subt_cave_sim/models`;
* **vcs import** -- add the `subt_cave_sim` repository to a `.repos` file under
  `vcstool/` and `git lfs pull` it into `bootstrap/`;
* **Fuel** -- every tile is on `fuel.gazebosim.org` under the *OpenRobotics*
  owner. Rewriting the `<uri>`s to `https://fuel.gazebosim.org/1.0/
  OpenRobotics/models/Cave%20Straight%2001` lets gz download and cache them at
  first run (into `~/.gz/fuel`), at the cost of needing the network once.

None of that is required for `world:=cave_box`, which is built from primitives
and is what the package's own verification runs against.

`models/rmf_owl/model.sdf.in` carries the flight dynamics of the original
RMF-Owl verbatim but replaces its 34 MB of DAE meshes with primitives; see the
comment at the top of that file for the rest of the deltas, including the one
that mattered: Harmonic has no `MulticopterPositionControl`.

## Why there is a `gbplanner_gz_control`

The ROS 1 SDF used two gz systems to fly: `MulticopterVelocityControl` (upstream)
and `MulticopterPositionControl` (an NTNU addition). Harmonic ships the first and
not the second, and its source is not available anywhere in this project:

```
$ find / -name "libgz-sim*multicopter*"
.../libgz-sim8-multicopter-control-system.so.8.11.0
.../libgz-sim8-multicopter-motor-model-system.so.8.11.0
```

Rather than fork a gz system, the position loop moved into ROS as
`gbplanner_gz_control/uav_path_follower_node`: the ROS 1 path follower plus a
saturated P controller that closes position onto the velocity command the
surviving gz system already accepts.

## Regenerating `cave_box.sdf`

`worlds/gen_cave_box.py` writes it. Change the geometry there and re-run:

```bash
python3 worlds/gen_cave_box.py
```
