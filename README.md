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
make run-sim uav_cave   # simulator + planner + RViz, one command
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
make run-sim uav_niosh                        # a scenario
make run-sim uav_cave ARGS="headless:=true"   # ... with gz's GUI off
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
sim/gbplanner_gz_control/               path follower
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

## Running a scenario

Three scenarios, all of them a UAV, all needing the `subt_cave_sim` model set
(`SUBT_CAVE_SIM` in the Makefile says where it is looked for):

```bash
make run-sim uav_cave   ROS_DOMAIN_ID=42     # DARPA SubT cave
make run-sim uav_niosh  ROS_DOMAIN_ID=42     # NIOSH mine
make run-sim uav_cargo  ROS_DOMAIN_ID=42     # cargo tank, actuated camera
```

`ROS_DOMAIN_ID` is worth setting whenever anyone else is on the network: ROS 2
has no master, so two stacks on the default domain will find each other's nodes.

Nothing plans until it is told to. From the RViz panel press **Initialization**,
then **Start Planner**; or, naming the container the scenario brought up:

```bash
docker exec gbplanner_ros2-uav_cave bash -lc 'source /workspace/gbplanner3_ws/install/setup.bash
ros2 service call /pci_initialization_trigger planner_msgs/srv/PciInitialization
sleep 15
ros2 service call /planner_control_interface/std_srvs/automatic_planning std_srvs/srv/Trigger'
```

Useful arguments: `ARGS="headless:=true"` runs gz without its GUI, `RVIZ=false`
drops RViz, and both matter because the lidar raycasts on the CPU whenever EGL
cannot get a hardware context.

To stop, and to clean up after:

```bash
make stop-sim SCENARIO=uav_cave
make clean-shm        # leaked Fast DDS segments, with nothing running
```

`make scenarios` lists what there is; `make help` lists every target.

## Checking the port still holds

```bash
cd test/parity
./run_parity.sh                    # Suite B: the numbers, against ROS 1's
./suite_a/run_suite_a.sh           # Suite A: whole stacks, same cave
./suite_c/run_suite_c.sh           # Suite C: does it explore, for minutes
```

Suite B is the one that gives a verdict: it exits non-zero on any difference its
comparator cannot account for. The table below says what each is for.

## Parity with the ROS 1 original

Three suites, all runnable, all comparing against the real ROS 1 stack rather than
against expectations.

| Suite | What it proves | Run it |
|---|---|---|
| config | the generated YAMLs resolve to the same values ROS 1's own loader produces - 27 files, 2271 leaves | `test/parity/check_config_parity.py --ros1-config-dir <ros1>/gbplanner/config` |
| B | the computational core computes the same numbers - one test body compiled against both workspaces, 1217 keys, `rrg.cpp` included | `test/parity/run_parity.sh` |
| A | both stacks live on the same cave produce comparable graphs and paths, measured against ROS 1's own run-to-run spread | `test/parity/suite_a/run_suite_a.sh` |
| C | a scenario, launched exactly as `make run-sim` launches it, explores on its own and keeps exploring | `test/parity/suite_c/run_suite_c.sh --scenario uav_cave --seconds 420` |

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
- Concurrent multi-robot flight is not exercised. Namespacing is verified; two
  robots actually moving at once is not.
- The scenarios fly the RMF-Owl where ROS 1 flew RotorS' rmf_obelix: neither
  RotorS nor its controller stack was ported, and rmf_obelix is not a gz model.
  The planner configs are upstream's.
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
