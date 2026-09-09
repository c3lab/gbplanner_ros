# gbplanner3 on ROS 2 Jazzy

A port of NTNU-ARL's graph-based exploration planner from ROS 1 Noetic to ROS 2
Jazzy, with a Gazebo Harmonic simulation. The ROS 1 repository it was ported from
(`gbplanner_ros`) is untouched and still works; this is a separate stack, not a
replacement in place.

## Quick start

```bash
git clone git@github.com:GabrieleSantangelo/gbplanner_ros2.git
cd gbplanner_ros2
make build-all          # image, then vcs import, then colcon build
make scenarios          # what there is to run
make run-sim ugv_niosh  # simulator + planner + RViz, one command
```

`make build-all` needs an ssh-agent with a key that can read the three
`*_ros2` repositories (`make bootstrap` clones them over SSH). `make help` lists
every target.

`make run-sim` takes one word and reads it two ways. A **scenario** name from
`make scenarios` starts that whole demo - gz with its world and robot, the
planner, the control interface and RViz, in one container. Anything else is a
**namespace**, which is what `run-sim` meant before scenarios existed and what
multi-robot still uses: it brings up one namespaced planner stack against a
simulator you started yourself. The recipe says out loud which reading it took.

```bash
make run-sim ugv_urban                        # a scenario
make run-sim ugv_niosh ARGS="world:=cave_box" # ... without the external assets
make run-sim robot0                           # a namespace, as before
make stop                                     # containers survive compose down
```

## Layout

```
planner_msgs/ planner_semantic_msgs/    interfaces (names frozen, see below)
kdtree/ planner_common/ map_manager/    planner core
planner_control_interface/ gbplanner/   planner and its control interface
gbplanner_ui/                           RViz2 panel
sim/gbplanner_gz_sim/                   worlds, robot models, launch
sim/gbplanner_gz_control/               path followers (UAV and UGV)
tools/                                  ROS 1 config and .rviz converters
test/parity/                            the parity suites
docker/ docker-compose.yml Makefile     build and run
bootstrap/                              gitignored; vcs import lands here
```

Dependencies are split three ways, and the split is by *what depends on what*,
not by who wrote it:

| Tier | What | Where | Why |
|---|---|---|---|
| A | voxblox fork, mav_msgs, minkindr, eigen_checks, xmlrpcpp | baked into the image at `/opt/deps_ws` | nothing first-party depends on them |
| B | the nine first-party packages, `sim/*` | bind-mounted from the repo | edited daily |
| C | adaptive_obb, pci_general, manhole_detector | bind-mounted from `bootstrap/` | they depend on first-party code, so baking them would force an image rebuild on every `.srv` change |

## Things that are not obvious

**voxblox is a fork, deliberately.** `vcstool/image_deps.repos` pins
`GabrieleSantangelo/voxblox-ros2` at branch `gbplanner`, not `master`. Upstream is
forked from ethz-asl master and is missing the integrator the ROS 1 side actually
runs, which fails silently rather than loudly - obstacles get carved out of the
map and collision checks return free through walls. `docs/VOXBLOX_FORK.md` has the
detail.

**Interface names are frozen.** `planner_msgs` was seeded from the ROS 2
definitions already in production use through the ros1_bridge, so services are
PascalCase (`PlannerSrv`, `PciToWaypoint`) and message constants are UPPER_SNAKE
(`BASIC_EXPLORATION`, not `kBasicExploration`). Existing clients depend on these.

**Configs are generated, not hand-edited.** The ROS 1 YAMLs use `rad()` / `deg()`
expressions, which are a feature of ROS 1's `rosparam` loader and not YAML at all.
`tools/rosparam2ros2.py` expands them with that loader's exact semantics and
promotes the integer literals that `rclcpp` would reject.
`test/parity/check_config_parity.py` proves the result matches, by having the ROS 1
image resolve the originals with its own loader.

**RViz configs likewise.** `tools/rviz1_to_rviz2_gbplanner.py` wraps rviz2's own
official converter and re-injects the two plugins it cannot know about
(`voxblox_rviz_plugin/VoxbloxMesh` and this repo's own panel).

**Gazebo Classic is not ported.** It is EOL with no Jazzy packages, so the `gzc/`
launch and config trees are gone and only the `gz` paths carry over. The
simulation targets Harmonic, which is what Jazzy pairs with.

**The simulation needs two things to use the GPU**, and both are easy to get
wrong: the NVIDIA EGL ICD manifest (the container toolkit mounts the library but
not the manifest; the image writes it) and `headless_rendering:=true` on
`gz_sim.launch.py`. `gz sim -s` alone removes the GUI without enabling offscreen
rendering, and the gpu_lidar then raycasts on the CPU - 6.7 Hz instead of its
nominal 10.

**Killed containers leak shared memory, and the symptom is not shared memory.**
Every service runs with `ipc: host`, so Fast DDS puts its shared-memory segments
in the host's `/dev/shm`. A container that is killed rather than shut down leaves
them behind, and once a few hundred have accumulated, ROS 2 discovery inside a
*new* container stops working: `ros2 topic list` returns nothing, nodes never
find each other's services, and because Gazebo has its own transport it goes on
looking perfectly healthy. It reads as a simulator that failed to start. `make
stop` counts them and `make clean-shm` removes them.

**Multi-robot isolation is namespaces, not a master.** ROS 2 has no roscore, so
`make run-sim robot0` and `make run-sim robot1` share one DDS domain and separate
by namespace. Set `ROS_DOMAIN_ID` only to isolate robots completely.

**What the UGV needed: the robot was seeing itself.** gbplanner's UGV configs
were written for the SMB, and on the MARBLE Husky the planner would not build a
graph at all - 8200 planning calls, one vertex each, the robot stationary
through all of it. Two values fixed it, and only one of them is interesting.

The blocker is that `Rrg::buildGraph` clears its own root box with
`augmentFreeBox`, which calls `clearIfUnknown` and so by construction never
touches an observed-*occupied* voxel. If the root box reads occupied rather
than unknown, every edge out of the root vertex is refused and the graph never
leaves one vertex. It read occupied because the Husky's lidar sees the Husky:
the sensor is 0.436 m above `base_link` at x = 0.424 with a +/- 45 degree
vertical fan, so its downward-rear rays strike the robot's own deck at ranges
from 0.3 m out to 0.98 m, and voxblox's `min_ray_length_m` of 0.5 integrates
them as a solid shell at deck height that follows the robot everywhere.

* `min_ray_length_m` 0.5 -> **1.0** drops every self-hit, the furthest being
  0.98 m. What it costs is the ring of floor between 0.81 m, where the 45
  degree ray first reaches the ground, and 1.0 m.
* `RobotParams.center_offset` z 0.0 -> **0.32** puts the collision box where
  the robot is. `base_link` sits 0.13 m up and the box is 0.4 m tall, so it
  occupies z in [0.25, 0.65] - the Husky's real body, above its 0.13 m of
  ground clearance. Upstream's 0.0 was written for the SMB, whose base frame is
  0.5 m up.

An earlier version of this port used `center_offset` 0.80 instead and left
`min_ray_length_m` alone. That also works - it clears the self-hit shell by
floating over it, and it is how the shell was found - but it models the robot
as a box 0.6 to 1.0 m above the ground, so the planner cannot see anything the
wheels would hit. Fixing the ray length removes the reason the hack existed.

Two other things were tried and are *not* in the configs. Halving voxblox's
`truncation_distance`, on the theory that the floor's TSDF band was smearing
upwards: it does not move the occupied band. And `occupancy_distance_voxelsize_factor`
0.5, to thin the occupied shell so the box could sit lower still at 0.22: the
planner then replanned 502 times in 420 s and covered 13 m, against 19 replans
and 20 m at 0.32. Upstream's 1.0 stays.

How long the robot sits still before the first useful graph is a property of
the world, not of the robot. On `niosh_osrf` there is no warm-up at all - the
first planning call already returns 240-270 vertices. On `cave_box` the Husky
starts half a metre from a wall in a world with nothing else in it, and takes
roughly 1400 root-only calls before there is enough free space to sample
through.

## Parity with the ROS 1 original

Three suites, all runnable, all comparing against the real ROS 1 stack rather than
against expectations.

| Suite | What it proves | Run it |
|---|---|---|
| config | the generated YAMLs resolve to the same values ROS 1's own loader produces - 27 files, 2271 leaves | `test/parity/check_config_parity.py --ros1-config-dir <ros1>/gbplanner/config` |
| B | the computational core computes the same numbers - one test body compiled against both workspaces, 1217 keys, `rrg.cpp` included | `test/parity/run_parity.sh` |
| A | both stacks live on the same cave produce comparable graphs and paths, measured against ROS 1's own run-to-run spread | `test/parity/suite_a/run_suite_a.sh` |
| C | a scenario, launched exactly as `make run-sim` launches it, explores on its own and keeps exploring | `test/parity/suite_c/run_suite_c.sh --scenario ugv_niosh --seconds 420` |

Suite C is not a parity suite - it has no ROS 1 side - but it lives with them
because it answers the question the other three cannot: whether the stack does
anything. Every check is written so that "nothing happened" fails rather than
passes, which is harder than it sounds: a grep for errors over an empty log is
silent, and an empty node list reads as "no crashes". Two of its checks exist
only because an earlier version of it passed a run that had not worked - one
bounds the degenerate warm-up, the other records what velocity the follower was
asking for, so a robot that stops tells you whether the follower gave up or the
robot could not execute what it was given.

Suite A is the one to read the README of before running: it is slow, and its
result only means anything because it measures ROS 1 against itself first. Exact
equality is unreachable - the RNG re-seeds per iteration, graph expansion has a
wall-clock budget, and Dijkstra's tie-breaking depends on allocator addresses, so
ROS 1 does not reproduce itself either.

## Known differences from ROS 1

Documented rather than hidden. Details are in the commit messages that introduced
each.

- Graphs saved by the Noetic planner cannot be loaded here: `ros::serialization`
  and CDR are different wire formats. Only matters if you have ROS 1 graph files
  to reuse.
- `ugv_urban` runs but is barely evidence of anything: the SubT Urban world is
  large enough that gz manages roughly a fifth of real time, so a 420 s run
  buys 3 planning cycles and 11 m of driving. It passes Suite C, and Suite C's
  "planned repeatedly" threshold is 3.
- Concurrent multi-robot flight is not exercised. Namespacing is verified; two
  robots actually moving at once is not.
- The UGV scenarios run the MARBLE Husky where ROS 1 ran the SMB, and the
  Classic UAV scenarios run the RMF-Owl where ROS 1 ran RotorS' rmf_obelix.
  Neither replaced robot is a gz model and neither controller stack was ported.
  The planner configs are upstream's apart from the UGV's
  `RobotParams.center_offset` and `min_ray_length_m`, which the section above
  explains.
- **The Husky gets stuck on the mine floor, and neither the planner nor the
  follower can tell.** Two of the four `ugv_niosh` runs on the final
  configuration ended badly, one of them this way: the robot immobile, the
  follower publishing a constant 0.54 m/s forward, the odometry moving by
  micrometres, the planner still producing paths and the control interface
  still waiting for a path end that never arrives. Nothing logs an error. It is
  physics, not planning - the Husky has 0.13 m of ground clearance and the SubT
  meshes have lips that high - and it happened both before and after the
  `center_offset` change, so a planner that could see low obstacles would not
  obviously have avoided it. Neither the follower nor the control interface has
  any recovery behaviour; adding one is the fix, and it is not in this port.
- The Husky occasionally falls through the tunnel mesh: the other bad run of
  those four ended at z = -602 m, still accelerating. Thin triangle-mesh
  collision at speed under DART. Suite C fails such a run now; before the check
  existed it scored it as 1102 m of exploration and passed.
- `pci_general` throws on shutdown: SIGINT landing while it sleeps between
  planner calls gives `context cannot be slept with because it's invalid` and
  the process aborts. It is a teardown race in third-party code, harmless to a
  run, and Suite C reports it as a note rather than a failure by splitting the
  log at the shutdown signal.
- `darpa_subt_final_circuit` is not ported. The world itself is eight inline
  models and one Classic plugin, an afternoon's work, but its geometry is a
  single 843 MB COLLADA file used as both visual and collision mesh, which is
  not something this simulation can load and be verified against.
- `manhole_detector` builds and runs, but its detection pipeline is untested: it
  needs an organised point cloud, which came from a ROS 1 lidar_simulator branch
  with no counterpart here.
- Several pre-existing bugs are preserved on purpose, because parity came first -
  uninitialised reads in `SensorParamsBase` and `BoundedSpaceParams`,
  `setEuler(0, 0, yaw)` writing roll, and the unstable Dijkstra tie-breaking noted
  above. Suite B documents each.
