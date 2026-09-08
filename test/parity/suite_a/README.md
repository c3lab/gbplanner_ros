# Suite A — live parity: ROS 1 in Gazebo Classic vs the ROS 2 port in gz Harmonic

Suite B compares the two ports offline, one shared test body compiled twice,
nothing spinning. Suite A does the opposite: it starts each port's *whole*
stack — simulator, planner node, planner control interface — on the same DARPA
cave, parks the robot at the same world pose, lets voxblox integrate for the
same span of simulated time, calls `/gbplanner`, and compares the graph and the
path that come back.

```
./run_suite_a.sh                        # 5 runs a side, 3 planner calls a run
./run_suite_a.sh --runs 3 --calls 2
./run_suite_a.sh --sides ros1           # one side only
./run_suite_a.sh --runs 1 --start-index 4 --sides ros2   # redo a single run
./run_suite_a.sh --compare-only         # re-run the comparator over out/
./check_map_and_config.py               # same cave? same planner parameters?
```

Exit status is non-zero when any observable falls outside the control described
below and is not listed in `EXPECTED_DEVIATIONS`.

Everything runs in throwaway containers (`gbplanner:noetic-3.0.0`,
`gbplanner-ros2:jazzy-3.0.0`); neither image is rebuilt, nothing is installed on
the host, and `/home/santal/git/gbplanner_ros` is mounted read-only in full and
never written.

## Result as of this writing

Five runs a side, three `/gbplanner` calls a run, 15 samples each, 60 s of
simulated map build-up per run, robot parked at (40, 5, 1) in `darpa_cave_01`.

```
metric                        ROS 1 (control)                      ROS 2        z
                   mean +- halfrange [min,max] mean +- halfrange [min,max]
----------------------------------------------------------------------------------
best_gain           414814.199 +-76778.665 [325643.9,479201.3]  437136.842 +-49894.995 [399014.7,498804.7]    0.29
best_path_id            77.533 +- 46.500 [  44.0, 137.0]     133.867 +-110.000 [  13.0, 233.0]    1.21  <--
graph_bbox_volume     2143.891 +-940.315 [1381.1,3261.7]    3245.956 +-789.971 [2114.7,3694.6]    1.17  <--
graph_density            0.066 +-  0.021 [ 0.045, 0.087]      0.074 +-  0.015 [ 0.059, 0.089]    0.42
graph_max_reach         14.662 +-  1.738 [  11.6,  15.1]      15.053 +-  0.149 [  14.9,  15.2]    0.23
graph_n_edges          562.333 +-286.000 [ 401.0, 973.0]    1454.800 +-755.500 [ 869.0,2380.0]    3.12  <--
graph_n_vertices       136.867 +- 26.500 [ 111.0, 164.0]     240.600 +- 61.000 [ 160.0, 282.0]    3.91  <--
log_path_length          6.420 +-  1.660 [   5.2,   8.5]       6.524 +-  1.537 [   5.2,   8.3]    0.06
log_path_size            3.333 +-  1.000 [   3.0,   5.0]       3.200 +-  0.500 [   3.0,   4.0]    0.13
odom_x / y / z          40.000 /  5.000 /  1.000            40.000 /  5.000 /  1.000              0.00
path_end_dist            6.090 +-  1.747 [   4.3,   7.8]       6.165 +-  1.602 [   4.7,   7.9]    0.04
path_length              6.420 +-  1.660 [   5.2,   8.5]       6.524 +-  1.537 [   5.2,   8.3]    0.06
path_size                3.333 +-  1.000 [   3.0,   5.0]       3.200 +-  0.500 [   3.0,   4.0]    0.13
status                   0.000                                  0.000                             0.00
t_build                 64.565 +- 49.166 [  13.3, 111.6]      14.438 +-  5.622 [  10.0,  21.2]    1.00  <--
t_dijkstra               0.296 +-  0.247 [   0.1,   0.5]       0.101 +-  0.049 [   0.1,   0.2]    0.19
t_eval                   6.396 +-  5.952 [   1.2,  13.1]       2.491 +-  0.963 [   1.7,   3.7]    0.39
t_gain                 262.828 +-150.970 [  71.2, 373.1]      81.759 +- 32.485 [  54.6, 119.5]    1.20  <--
t_total                334.085 +-198.225 [  85.8, 482.2]      98.789 +- 38.263 [  66.4, 142.9]    1.19  <--
wall_call_s              0.516 +-  0.496 [   0.1,   1.1]       0.140 +-  0.080 [   0.1,   0.2]    0.76

hausdorff ROS1 vs ROS1 (control)   mean   2.27 m  [0.64, 3.99]
hausdorff ROS2 vs ROS2             mean   2.09 m  [0.42, 4.55]
hausdorff ROS1 vs ROS2             mean   2.16 m  [0.29, 4.69]
endpoint direction ROS1 vs ROS1  mean   16.6 deg [2.4, 33.8]
endpoint direction ROS1 vs ROS2  mean   15.6 deg [0.8, 40.9]

PASS: every observable is either inside ROS 1's own run-to-run spread or listed
      with the measurement that explains it.
```

**The path is the same path.** Its length (6.42 vs 6.52 m), its waypoint count
(3.33 vs 3.20), the straight-line distance it covers (6.09 vs 6.17 m) and the
gain the planner assigned it (4.15e5 vs 4.37e5) all sit at z <= 0.29. The
strongest statement is the geometric one: the Hausdorff distance between a ROS 1
path and a ROS 2 path is **2.16 m**, and between two ROS 1 paths it is **2.27 m**
— the two ports are, if anything, closer to each other than ROS 1 is to itself.
The same holds for where the paths point: 15.6 degrees across the port against
16.6 degrees within ROS 1. Both sides head into the same tunnel.

**The graph is bigger on ROS 2, and the reason is the simulator, not the port.**
The ROS 2 RRG carries 241 vertices to ROS 1's 137. The suite measures why rather
than guessing: the *density* is the same (0.066 vs 0.074 vertices/m^3, z = 0.42),
the *volume* is not (2144 vs 3246 m^3), and the ROS 2 graph touches the 15 m
local sampling bound on all 15 samples (`graph_max_reach` 15.05 +- 0.15) where
ROS 1 sometimes stops at 11.6 m. Neither side is anywhere near
`num_vertices_max: 400`, so nothing is clipped. The RRG samples the same way and
simply finds more known-free space to sample in: in the same 60 s of simulated
time, gz Harmonic's `gpu_lidar` maps more of the cave than Gazebo Classic's
`gpu_ray` Ouster does. And it does not reach the output — every path observable
above is inside the control.

**ROS 2 is about three times faster** (99 ms per call against 334 ms) while
building the larger graph. Toolchain (GCC 9.4/Eigen 3.3.7/PCL 1.10 against GCC
13.3/Eigen 3.4.0/PCL 1.14) and the fact that the ROS 1 lidar is rasterised on
the CPU next to the planner both push that way. `t_dijkstra` and `t_eval`, the
two stages that touch neither the map nor PCL, are inside the control.

**Capture integrity.** `graph_n_vertices` and `log_n_vertices` — the graph read
off `/vis/planning_graph` and the graph the planner logged — agree on 30/30
samples. They are independent readings of the same object.

## Are the two maps really the same? `check_map_and_config.py` says yes

The whole comparison rests on this, so it is measured rather than asserted.

```
World: 69 includes in ROS 1, 68 in ROS 2
  accepted  base_station   ROS 1 only
  accepted  staging_area   pose 0 0 0 ... vs 0 0 0.1 ...  (ROS 2 raises it 0.1 m)
  include order over the shared tiles identical: True
  voxblox max_ray_length_m = 15.0; hover pose (40.0, 5.0, 1.0)
    base_station   origin (-8.0, 0.0, 0.0)   48.3 m from the hover pose -- out of reach
    staging_area   origin (0.0, 0.0, 0.0)    40.3 m from the hover pose -- out of reach

Planner configuration (cave_exploration)
  accepted voxblox_sim_config.yaml: mesh_min_weight = '1e-4' vs 0.0001
  256 shared leaves compared

PASS: same cave, same planner parameters
```

All 67 shared tiles carry the same `model://` URI, the same pose to the last
digit and the same `static` flag, in the same order, and both worlds resolve
those models out of one bind-mounted `subt_cave_sim` checkout — the same meshes,
not two copies. The two entries that differ are both more than 40 m from where
the robot sits, against a 15 m voxblox integration range, so neither can enter
either map; the check computes that distance instead of taking it on trust.

The planner parameters are compared through ROS 1's **own** loader, because
`rosparam` installs `rad()`/`deg()` resolvers before parsing and no plain YAML
reader agrees with it. 256 leaves match; the single exception is the
`mesh_min_weight` one `check_config_parity.py` already documents (ROS 1 loads
`1e-4` as a *string*, voxblox's `float` `param()` call type-mismatches and falls
back to its own `1e-4`, and the ROS 2 file writes that same number as `0.0001`).

## The subnormal quaternion x/y: it is in the ROS 1 original too

Suite B could not reproduce this offline, on either side, even with the heap
deliberately poisoned, and concluded the cause had to be downstream of `Rrg`.
Suite A reads the live service three ways per side. The counts below are over
the campaign above:

```
ros1  rospy probe       :  4 / 50 poses with subnormal x or y
ros1  rosservice call   :  0 subnormal, 30 exactly zero
ros1  C++ roscpp client :  0 subnormal, 18 exactly zero
ros2  rclpy probe       : 48 / 48 poses with subnormal x or y
ros2  ros2 service call : 26 subnormal, 14 exactly zero
ros2  C++ rclcpp client : 16 subnormal,  0 exactly zero
```

Three things follow.

1. **It is not the Python client's printing.** A C++ `rclcpp` client reading the
   same service sees the same subnormals, bit for bit — e.g.
   `x = 0x00003071ad2acc28`, `y = 0x800001097a0360ac`, which are 64-bit
   userspace addresses reinterpreted as doubles. Not one of the 16 C++ readings
   came back as a clean zero. There is nothing to fix in the
   client, because the response really carries those bytes.
2. **It is not the port.** The ROS 1 original does it too. It is rarer there —
   4 poses in 50 against 48 in 48, and it appears in exactly one of the 15 ROS 1
   samples — and it is intermittent inside a single process: in that run the
   first two `/gbplanner` calls returned clean zeros and the third returned
   subnormals, from the same binary against the same map. That is the signature of a read of
   uninitialised heap: whether it looks like zero depends on what was last freed
   into that block. ROS 1's leftovers happen to be small integers
   (`0x11ab`, `0x1fa0`, `0x25c0`), ROS 2's are pointers.
3. **The fields are unwritten, not corrupted.** For every affected pose,
   `z^2 + w^2 == 1.0` exactly. The rotation the response means is a pure-yaw
   quaternion whose `x` and `y` should both be 0; the two components were simply
   never assigned. A consumer reading yaw out of all four components — `tf::getYaw`
   does — is reading two uninitialised doubles.

What Suite A does **not** establish is the write site. The exploration path is
built in `Rrg::getBestPath` by `tf::poseTFToMsg` / `tf2::toMsg`, both of which
write all four components, and with this configuration
(`path_interpolation_distance: -1.0`, `yaw_tangent_correction: false`,
`path_safety_enhance_enable: false`) none of the two-component write sites in
`rrg.cpp` is reachable. So the defect is real, reproducible live on both ports,
and still unlocated in the source.

## The control is the point

Exact equality is unreachable here, and none of the reasons are the port's
fault:

* `Gbplanner::plannerServiceCallback` starts with `rrg_->reset()`, and
  `RandomSampler::reset` re-seeds its `std::mt19937` from `std::random_device`.
  Every single service call therefore draws a different sample sequence.
* `rrg.cpp` calls an unseeded `rand()` in the global-graph expansion timer,
  which — unlike in Suite B — really does fire here, because the node spins.
* That expansion runs against a wall-clock budget, so how much of it happens
  depends on how loaded the machine is.
* `Graph` is `boost::adjacency_list<setS, listS, undirectedS, ...>`, so a vertex
  descriptor is a pointer into a `std::list` and Dijkstra's tie-breaking depends
  on where the allocator put the vertices. Suite B measured this directly:
  `parent_stable_under_heap_shift = 0` on **both** sides — one binary already
  disagrees with itself after unrelated allocations.
* The two simulators are different programs. The ROS 2 tree deleted the `gzc`
  trees because Gazebo Classic is EOL on Jazzy, so the ROS 1 side is Gazebo
  Classic 11 with a `gpu_ray` Ouster and the ROS 2 side is gz Harmonic with a
  `gpu_lidar`. Same cave, same beam counts, same ranges — different renderer,
  different physics engine, different vehicle model.

So a single ROS1-vs-ROS2 pair proves nothing. The suite runs the ROS 1 side five
times over, measures how far ROS 1 gets from **itself**, and only then asks
whether ROS 2 is further away than that:

```
z = |mean_ros2 - mean_ros1| / max(halfrange_ros1, floor)
```

`z <= 1` means the port moved an observable by no more than ROS 1's own
run-to-run spread. `halfrange` is `(max - min) / 2` over the ROS 1 samples. The
floors in `FLOORS` only ever *widen* a control that five samples happened to
make implausibly narrow; each is roughly the quantisation of its own metric (one
waypoint, one RRG edge length, 5 % of a typical vertex count).

The comparator prints both distributions side by side, always. A verdict without
the control next to it is not a result.

## Protocol

Identical on both sides except where the port forces a difference:

| | ROS 1 | ROS 2 |
|---|---|---|
| launch | `roslaunch gbplanner uav_gzc_cave_exploration.launch` | `ros2 launch gbplanner_gz_sim gz_sim.launch.py` + `ros2 launch gbplanner gbplanner.launch.py` |
| simulator | Gazebo Classic 11, `gzserver` | gz Harmonic, `gz sim -s` |
| world | `arl_gazebo_sim/gzc/worlds/darpa_cave_01.world` | `gbplanner_gz_sim/worlds/darpa_cave_01.sdf` |
| tile models | `bootstrap/sim/subt_cave_sim/models` | the same directory, bind-mounted |
| vehicle | rmf_obelix (rotors_simulator) | rmf_owl (gz multicopter) |
| lidar | `gpu_ray`, 512 x 64, +-45 deg, 0.2–100 m | `gpu_lidar`, 512 x 64, +-45 deg, 0.3–50 m |
| planner config | `config/uav/gz/cave_exploration` | `config/uav/gz/cave_exploration` |
| pose | commanded to (40, 5, 1) via `command/trajectory` | commanded to (40, 5, 1) via the follower's path topic |
| map build-up | 60 s of **simulated** time, stationary | the same |
| trigger | `/gbplanner`, `bound_mode = 0`, 3 calls | the same |

Two sensor details that look like differences and are not. The maximum ranges
differ (100 m vs 50 m) but voxblox truncates every ray at `max_ray_length_m:
15.0` on both sides, so the range only matters for surfaces further away than
either map keeps. And both stacks mount the beam ~3.5 cm above the frame the
cloud is published in — ROS 1 puts the `gpu_ray` at `base_link + 0.08` while its
static TF places the cloud frame at `base_link + 0.05`; ROS 2 puts the sensor at
`laser_link + 0.035925` with `laser_link` at `base_link + 0.05` and only
`laser_link` in TF. Both therefore integrate as if the sensor sat at
`base_link + 0.05`, and the two agree to about 6 mm.

### Deliberate deviations from the two stacks' defaults, and why

* **`config_folder:=uav/gz/cave_exploration` on the ROS 1 launch.** The launch's
  own default is `uav/gzc/cave_exploration`, and the ROS 2 YAMLs were generated
  from the `gz` folder, not the `gzc` one. The two ROS 1 folders are not small
  variations of each other: `graph_building_mode` is `kBasic` vs `kBatch`,
  `tsdf_voxel_size` 0.40 vs 0.30 m, `max_ray_length_m` 20 vs 15 m, the lidar gain
  range 15 vs 20 m, `traverse_length_max` 12 vs 5 m, `exploration_only` true vs
  false. Running the launch's default against the ROS 2 config would measure a
  different planner, not a different middleware. `check_map_and_config.py` proves
  the two `uav/gz/cave_exploration` folders resolve to the same numbers.
* **The robot is commanded to a hover pose.** Left alone the ROS 1 UAV falls onto
  the cave floor (z ~ 0, tilted a few degrees by the terrain under it) while the
  ROS 2 one holds about a metre. A 360-degree lidar at those two heights does not
  build the same map, and every graph observable would inherit the difference.
  Both sides are commanded through their own stack's normal interface.
* **`lidar_horizontal_samples:=512 lidar_vertical_samples:=64` on the ROS 2 sim.**
  Its launch defaults to 2048 x 128 and calls them "the ROS 1 values"; the ROS 1
  `rmf_obelix_base.xacro` instantiates the OS0-128 macro with `samples="512"
  lasers="64"`, overriding the macro's own 512 x 128 default. 512 x 64 is what
  ROS 1 actually simulates.
* **`path_topic:=/suite_a/hover_path` on the ROS 2 sim.** `uav_path_follower_node`
  otherwise listens on `/gbplanner_path`, which `Rrg` also publishes on. Nothing
  in the ROS 1 simulator follows that topic, so leaving the follower there would
  let the ROS 2 robot fly off on an output the ROS 1 robot ignores.
* **Three probe calls per run, not more.** With `exploration_only: false` the
  cave config sets `max_exploration_iterations: 5`, and
  `Gbplanner::plannerServiceCallback` counts every successful exploration call;
  on the sixth it switches `planner_mode_` to inspection and stops returning
  exploration paths. Each run spends exactly five: three from the probe plus one
  each for the CLI and C++ clients that answer the quaternion question. Raising
  `--calls` past three needs that counter raised too.
* **RViz is killed on the ROS 1 side.** `uav_gzc_cave_exploration.launch` starts
  it unconditionally — there is no `rviz_en` guard on that node — and it is pure
  cost on a software rasteriser.

## Headless

Neither stack ran headless out of the box, for two different reasons.

**ROS 1 / Gazebo Classic.** The Ouster is a `<sensor type="gpu_ray">`, rendered by
OGRE inside `gzserver`, which needs a GL context, which needs an X display; a
detached container has none. The image already ships Xvfb and Mesa, so
`ros1_side.sh` starts `Xvfb :99` inside the container and points `DISPLAY` at it.
No host X server is touched and no window opens on the user's desktop.
`__GLX_VENDOR_LIBRARY_NAME` has to be *cleared* — docker-compose sets it to
`nvidia` for the desktop case, and against Xvfb, which has no NVIDIA GLX
extension, forcing the NVIDIA vendor library makes every GL context fail.
llvmpipe is enough: `/clock` runs at ~1000 Hz (real-time factor ~1) and the
lidar at its nominal 10 Hz.

**ROS 2 / gz Harmonic.** `gz sim -s` with no display falls back to EGL, and EGL in
this image finds only Mesa's ICD (`/usr/share/glvnd/egl_vendor.d/50_mesa.json`),
tries the DRM devices, fails `eglInitialize` on all of them, reports "OpenGL 3.3
is not supported" eleven times and then segfaults inside Ogre's material parser.
`libEGL_nvidia.so.0` **is** mounted by the container toolkit; only its vendor ICD
json is missing. `ros2_side.sh` writes that one line into the container (never
the image, never the host) and `gz sim --headless-rendering` then picks the GPU:
`/clock` at ~1000 Hz, lidar at 10 Hz, no X server anywhere.

`--headless-rendering` has to be smuggled through `gz_verbosity:=` because
`gz_sim.launch.py` builds its argument string as `-r -v <gz_verbosity> -s <world>`
and exposes no other hook. Adding a proper `headless_rendering` argument to that
launch file is the right fix; it is not this test's to make.

## Observables

Per service call, on both sides, in one JSON schema (`probe_ros1.py` and
`probe_ros2.py` differ only in the three places the middleware forces):

* the `/gbplanner` response — `status`, `planning_bound_mode`, every path pose,
  and every quaternion component as its raw IEEE-754 bit pattern and class;
* the odometry sampled at the moment of the call;
* the local RRG published on `/vis/planning_graph` — vertex positions out of the
  `SPHERE_LIST` marker and edges out of the `LINE_LIST` marker, which is the
  graph itself, not a summary of it;
* the wall-clock duration of the call;
* and, parsed out of the planner's own log, `Formed a graph with [N] vertices and
  [M] edges with [K] loops`, `Best path: with gain [G] and ID [I]`,
  `Best path: size/length/time`, and the four-way `Time statistics` block. Calls
  are separated in the log by `Current Planner Mode`, which
  `Gbplanner::plannerServiceCallback` logs unconditionally exactly once per call
  on both sides.

From the vertex cloud the comparator derives three more, which are what turn
"the ROS 2 graph is bigger" from an observation into an explanation: the
bounding-box volume the graph spans, the vertex density inside it, and the
furthest vertex from the root. A graph that is larger only because the free
space it was sampled in was larger is not a different planner.

One trap worth naming, because it silently deletes half the observables: on the
ROS 1 side `rosconsole` writes INFO to **stdout** and WARN/ERROR to stderr, and
stdout redirected to a file is fully buffered. Every `Formed a graph with [N]
vertices`, `Best path: with gain [G]` and `Time statistics` block then sits in a
4 KB buffer that the SIGTERM at the end of the run discards, leaving a log with
only the WARN lines in it and a comparator that reports those metrics as absent
rather than as wrong. `ros1_side.sh` sets `ROSCONSOLE_STDOUT_LINE_BUFFERED=1`
and runs roslaunch under `stdbuf -oL -eL`. The ROS 2 side does not need this:
its launch file sets `emulate_tty=True`, so the node's stdout is a pty.

Paths are compared as curves, not waypoint by waypoint: the two planners place
waypoints at unrelated arc lengths, so the comparator reports a symmetric
vertex-to-polyline Hausdorff distance and the angle between the two paths'
start-to-end directions — with the ROS1-vs-ROS1 value of each printed next to it.

## Layout

```
run_suite_a.sh            host runner: builds, runs both sides N times, compares
ros1_side.sh              inside gbplanner:noetic-3.0.0  (Xvfb)
ros2_side.sh              inside gbplanner-ros2:jazzy-3.0.0  (EGL headless)
probe_ros1.py             } one JSON schema, one protocol, two middlewares
probe_ros2.py             }
quat_client/              C++ clients for the quaternion question, both sides
compare_suite_a.py        the comparator, its floors and its deviation list
check_map_and_config.py   same cave and same planner parameters, proven
out/                      gitignored: per-run JSON, planner logs, transcripts
```
