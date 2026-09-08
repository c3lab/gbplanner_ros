# Suite B — deterministic C++ parity tests

One test body, compiled twice: once against the ROS 1 Noetic workspace, once
against the ROS 2 Jazzy one. Both binaries print the same ~1200 `key = value`
observables, and `compare_parity.py` diffs them and explains every difference or
fails.

```
./run_parity.sh              # build the ROS 2 workspace, run both sides, compare
./run_parity.sh --skip-ws-build
./run_parity.sh --seed 999   # any fixed seed; the two sides must still agree
```

Exit status is non-zero on any difference the comparator cannot account for.
Everything runs in containers (`gbplanner:noetic-3.0.0`,
`gbplanner-ros2:jazzy-3.0.0`); nothing is installed on the host and nothing under
`/home/santal/git/gbplanner_ros` is written.

## Result as of this writing

```
keys: 1217 in ros1.txt, 1217 in ros2.txt, 1217 shared
byte-identical:   1208
within tolerance: 1 (map.point_distance)
explained:        8
unexplained:      0
```

ROS 1 side: GCC 9.4, libstdc++ 9, Boost 1.71, Eigen 3.3.7, PCL 1.10.
ROS 2 side: GCC 13.3, libstdc++ 13, Boost 1.83, Eigen 3.4.0, PCL 1.14.

## How one test body runs against both sides

* `unit/parity_body.cpp` is the whole test. It never mentions `ros/` or
  `rclcpp/`.
* `unit/parity_shim_ros1.h` and `unit/parity_shim_ros2.h` expose an identical
  `parity::Fixture`. They are the only files that know which middleware is in
  play, and they carry exactly three differences: how a node is created
  (`ros::init` plus a private `roscore` vs `rclcpp::init`), how a parameter is
  supplied (global param server keyed with `/` vs per-node parameter overrides
  keyed with `.` — the test always writes the dotted canonical form), and the
  `MapManagerVoxblox` / `Rrg` constructor signatures.
* `CMakeLists.txt` builds both configurations with the same flags:
  `-O2 -ffp-contract=off -fno-fast-math -fno-unsafe-math-optimizations
  -march=x86-64 -mtune=generic`, plus `RAY_CAST_METHOD=0 COL_CHECK_METHOD=0
  EDGE_CHECK_METHOD=0`. `-O3` is deliberately avoided: GCC 13 reassociates
  floating-point reductions there that GCC 9 does not.
* The sources under test are **compiled into the test binary**, not linked from
  the installed libraries, so both sides are built from their own repository's
  sources under identical flags: `kdtree.c`, all six `planner_common/src/*.cpp`,
  all three `map_manager/src/**.cpp`, and `gbplanner/src/rrg.cpp` +
  `gbplanner_rviz.cpp`.
* Each side's results are emitted to the file named by `GBP_PARITY_OUT`, not to
  stdout — ROS 1's `rosconsole` writes INFO to stdout and would corrupt the
  channel.
* Each side runs its binary twice and `run_parity.sh` requires the two runs to
  be byte-identical before comparing the sides at all.

## Seed injection

`planner_common/src/random_sampler.cpp` re-seeds its `std::mt19937` from
`std::random_device` at three sites (`RandomSamplerBase::reset`,
`RandomSampler::setPDF`, `RandomSampler::reset`). Neither repository is edited.
Instead `prepare_sources.sh` copies `kdtree/`, `planner_common/`, `map_manager/`
and `gbplanner/` into a scratch tree **inside the container** and applies one
`sed` there:

```
generator_.seed(rd())  ->  generator_.seed(gbp_parity_seed())
```

`gbp_parity_seed()` lives in `unit/gbp_parity_seed.h`, is forced into that one
translation unit with `-include`, and reads `GBP_PARITY_SEED` (default 12345,
falling back to `std::random_device` when unset). The script asserts that
exactly three sites matched before and after, so a stale patch fails the run
rather than silently comparing two entropy-seeded streams. The identical rewrite
is applied to both sides, so the sampler under test differs from the shipped one
in exactly one respect and differs in that respect identically on ROS 1 and
ROS 2.

`rrg.cpp`'s unseeded `rand()` (`rrg.cpp:3221`) is only reached from
`expandGlobalGraphTimerCallback`. Nothing spins in this suite, so no timer ever
fires and that call site is never entered.

## What is covered

| Area | What is exercised | Keys |
|---|---|---|
| `kdtree` | 2000-point tree; 200 nearest-neighbour queries cross-checked against a brute-force scan; range queries at r ∈ {0.5, 2, **4.0**, 8} compared as sets; `kd_res_size` vs iteration count; `kd_clear` + reuse | 24 |
| `planner_common` params | `SensorParamsBase/SensorParams/RobotParams/BoundedSpaceParams/PlanningParams/RandomSampler::loadParams` driven through the real ROS parameter path on both sides. `PlanningParams` is loaded twice — from the cave configuration and from an **empty namespace**, so every "read failed, install the derived default" branch fires | 479 + 322 + 47 + 15 |
| Sensor geometry | `updateFrustumEndpoints` endpoint counts (the 1-ULP canary on the `dv += v_res` loop bound), `getFrustumEndpoints` at 15 poses, `getFrustumEdges` (kCamera only, see Findings), `isInsideFOV` over 3000 points per sensor, `isFrontier` threshold sweep | in the 322 |
| `trajectory.cpp` | `interpolatePath` (both overloads), `getPathLength`, `shortenPath`, `computeDTWDistance`, `computeDistanceBetweenTwoTrajectories`, `compareTwoTrajectories` sweep, `estimateDirectionFromPath`, and `computeDistanceBetweenTrajectoryAndDirection` — the exact call `evaluateGraph` makes | 37 |
| `Graph` / `GraphManager` | Boost Dijkstra on a 72-vertex lattice with deliberately tied costs and on an 80-vertex graph with unique costs; distances, parents, leaf sets, path contiguity, kd-tree nearest/range lookups | 21 |
| `geofence_manager.cpp` | `boost::geometry` over four polygons: `getCoordinateStatus` on 3111 points, `getBoxStatus` with the real 0.4 m footprint, `getPathStatus` on 300 segments | 4 |
| `random_sampler.cpp` | Reference `std::mt19937` sequences plus the four distributions the planner uses; 400 `RandomSampler::generate` draws with a fixed seed; the repeat-after-`reset()` contract | 6 + 6 |
| `map_manager` | A synthetic TSDF built in code (see below): layer statistics, `getVoxelStatus` over 47817 points, `getVoxelDistance`, `getPointDistance`, `getPointGradient`, `getBoxStatus` (600 centres × 2 flags), `getPathStatus` (400 segments × 2 flags), `getRayStatus` (400 rays × 2 flags, with `end_voxel` and `tsdf_dist`), `getScanStatus` for 5 sensors × 6 poses (`gain_log` triples plus the full ordered `voxel_log`), `extractLocalMap`, `getLocalPointcloud` | 192 |
| Response quaternions | `getBestPath`, `getBestPathSimplified`, `getHomingPath` and `Trajectory::interpolatePath`, each after a heap poison, with every component emitted as value, raw bits and IEEE-754 class | in the 36 |
| `rrg.cpp` | **One complete planning iteration in each graph-building mode** over that map: `loadParams`, `setState`, `reset`, `batchGraph`/`buildGraph`, `evaluateGraph`, `getBestPath` — with the resulting local graph, global graph, per-vertex volumetric gains and best path all compared | 36 |

### The synthetic map

There is no bag and no `.vxblx` fixture. `buildSyntheticLayer` ray-casts four
sensor poses against eight axis-aligned boxes — a 12 m corridor with a floor, a
ceiling, two side walls, a back wall, a cross wall with a 1 m doorway, and a
pillar — and feeds the 3698 resulting points to a `voxblox::SimpleTsdfIntegrator`
with every config field spelled out and `integrator_threads = 1`. Multi-threaded
integration accumulates the same weighted averages in a scheduler-dependent
order and is not reproducible even ROS 1 → ROS 1, so one thread is mandatory.

The resulting layer: 30 blocks, 21708 observed voxels, 11884 occupied, 9824
free — identical on both sides. The three probes that state the point directly:

```
map.probe.wall              = "O"   # inside the right wall
map.probe.corridor          = "F"   # middle of the corridor
map.probe.behind_cross_wall = "U"   # the room the sensors never saw
```

### The RRG iteration

`Rrg`'s second constructor takes an externally owned `MapManager`, which is what
lets the test fill the layer and then plan over it. The four timers the
constructor creates never fire because nothing spins, so the iteration runs
synchronously and the wall-clock-budgeted global-graph expansion is never
entered. The first `setState` is treated as the first odometry and wipes the map
(`rrg.cpp:7855-7860`), so the scene is integrated *after* it.

Both sides produce, bit for bit:

```
rrg.batch.local_graph.num_vertices = 382     rrg.basic.local_graph.num_vertices = 11
rrg.batch.local_graph.num_edges    = 6705    rrg.basic.local_graph.num_edges    = 55
rrg.batch.path_size                = 3       rrg.basic.path_size                = 2
```

plus every vertex position, every edge as a sorted position pair with its
weight, every vertex's `(gain, num_unknown, num_free, num_occupied, is_frontier)`
tuple, and every path waypoint including orientation. Vertex **IDs** are never
compared — they are assigned while walking an `unordered_map` — so positions are
compared as a sorted multiset instead.

## What is not covered

* **Anything that needs the node to spin.** No callbacks, no services, no TF, no
  QoS. Suite B constructs objects and calls their methods directly. A ported
  service handler that never reaches `Rrg` would pass this suite.
* **Multiple planning iterations.** `reset()` re-seeds with the same constant, so
  a fixed seed makes every iteration draw the identical sample sequence. That is
  right for a frozen single iteration and wrong for a closed loop; a multi-run
  comparison would need a seed schedule.
* **Global planning**: `getFrontiers`, target costing, homing, the global graph
  beyond the 5 vertices one iteration puts there.
* **The mesh, ESDF and skeleton halves of voxblox.** Only the TSDF path that
  `map_manager` reads is exercised.
* **`geometry_msgs` serialisation.** `saveGraph`/`loadGraph` write ROS 1 wire
  format on one side and CDR on the other; the files are deliberately not
  interchangeable and are not compared.
* **`getFrustumEdges` on kLidar sensors** and `isInsideSpace` at the declared
  bounds of a `kSphere` — see Findings, both read uninitialised memory in the
  original code.
* **The real configuration files.** Parameter *values* here are C++ literals, so
  that this suite measures `loadParams`, not YAML drift. YAML drift is what
  `check_config_parity.py` is for.

## Findings

### 1. Dijkstra's shortest-path *tree* is not reproducible, on either side

`Graph` is `boost::adjacency_list<setS, listS, undirectedS, ...>`. With `listS`
vertex storage a vertex descriptor is a pointer into a `std::list`, and the
`setS` out-edge container is ordered by that pointer — so when several shortest
paths have exactly equal cost, which one Dijkstra returns depends on where the
allocator happened to put the vertices.

The suite measures this instead of assuming it: `solveTiedLattice` builds the
same 72-vertex lattice twice in one process, the second time after 4099 unrelated
allocations. Both sides report

```
graph.tied.distance_stable_under_heap_shift = 1
graph.tied.parent_stable_under_heap_shift   = 0
graph.tied.leaf_ids_stable_under_heap_shift = 0
```

i.e. the same binary already disagrees with itself. `graph.tied.parent` and
`graph.tied.leaf_ids` therefore differ between ROS 1 and ROS 2 as well, and that
difference is not attributable to the port. What the planner actually consumes is
stable and does agree: `graph.tied.distance` is identical, every reconstructed
path is contiguous in the edge set (`broken_path_edges = 0`) and its length equals
the reported distance to 0.0 (`worst_path_length_residual = 0`), and
`graph.unique.parent` — the same test on a graph with no ties — matches exactly.

This matters beyond the test: `findLeafVertices` walks the parent map, and the
RRG uses the leaf set to pick candidate path endpoints. Two runs of the *same*
build can select different leaves.

### 2. Three uninitialised reads in the original code

None is a port divergence — both sides run the same buggy code — but each one
showed up as a difference the first time it was compared, which is how they were
found.

* `SensorParamsBase::edge_points_B` is only assigned on the `kCamera` path
  (`params.cpp:191`). `getFrustumEdges` reads it unconditionally
  (`params.cpp:461`), so calling it on a `kLidar` sensor — which is what all four
  sensors in the cave configuration are — returns the robot position plus
  garbage. On ROS 1 the memory happened to be zero, so the four "edges" came back
  as the position itself; on ROS 2 they were arbitrary.
* `BoundedSpaceParams::min_val` / `max_val` / `rotations` are only assigned on
  the `kCuboid` path (`params.cpp:708-760`). They are harmless for
  `isInsideSpace` on a sphere, which uses only `root_pos` and `radius_total`, but
  any code that reads the bounds of a `kSphere` space reads indeterminate values.
* `PlanningParams::augment_free_frustum_en` (`params.h:299`) is never written by
  `loadParams` and never read anywhere in the tree. Dumping it printed 248 on one
  side and 0 on the other — the raw byte, because the compiler is entitled to
  assume a `bool` holds 0 or 1.

The test now probes only the defined cases and adds a `kCamera` sensor
(`CamPinhole`) so that `getFrustumEdges` and the normal-vector branch of
`isInsideFOV` are still covered.

### 3. Subnormal quaternion x/y in the PlannerSrv response: NOT reproduced here

A live ROS 2 run was reported returning response poses whose `orientation.z` and
`.w` were correct while `.x` and `.y` held subnormal doubles around 1e-310 - the
signature of a read of uninitialised heap memory. Suite B was pointed at that
question directly. The result is a clean negative, on both sides.

`rrg.quat.*` runs every path producer that is reachable without spinning, each
one preceded by `poisonHeap()`, which dirties ~16 MB with the bit pattern
`0x0000ABCDEF012345` - a subnormal double of exactly the reported magnitude - and
frees it again, so a read of uninitialised heap picks that up instead of a fresh
zero page. Every component is emitted as a value, as its raw IEEE-754 bits, and
as a one-character class (`Z` zero, `S` subnormal, `N` normal, `I` infinite,
`A` NaN):

```
                                     ros1            ros2
rrg.quat.best_path.class             ZZZNZZNNZZNN    ZZZNZZNNZZNN
rrg.quat.best_path_simplified.class  ZZZNZZNNZZNN    ZZZNZZNNZZNN
rrg.quat.homing_path.class           ZZZNZZZN        ZZZNZZZN
rrg.quat.interpolated_best_path      ZZNN x 11       ZZNN x 11
rrg.quat.basic_best_path.class       ZZZNZZNN        ZZZNZZNN
```

Every `x` and `y` is exactly `0x0000000000000000`, and the `.bits` strings are
byte-identical between the two sides. `getBestPath` - the call whose result
becomes the `PlannerSrv` response path - `getBestPathSimplified`, `getHomingPath`
and `Trajectory::interpolatePath` all return fully initialised quaternions on
ROS 1 and on ROS 2 alike.

Two supporting facts, both checked rather than assumed:

* An exhaustive scan of every `<expr>.orientation.<component> =` assignment in
  the ROS 2 first-party tree finds **no partial quaternion write anywhere**;
  every group of consecutive assignments covers all four components.
* `rrg.cpp:5179-5180` is not a two-component write. It is the `.z` and `.w`
  half of a four-line block in `getHomingPath` whose `.x` and `.y` are at
  5177-5178, and the ROS 1 block at 5140-5143 is line-for-line identical.

So the divergence does not originate in `Rrg`'s pose construction. What this
suite cannot see, and where the cause must therefore be: the response assembly
in `gbplanner.cpp` / `gbplanner_ros.cpp` (not compiled here - they are the node
wrapper), the CDR serialisation and transport, or the client that read the
response. Reproducing it needs a live service call, which is Suite A territory,
not Suite B's.

### 4. `getPointDistance` differs by up to 5e-7 relative on 6 of 250 probes

Trilinear interpolation over voxels stored as `float`; float eps is 1.2e-7, so
this is two or three ULPs and is what Eigen 3.3.7 vs 3.4.0 and GCC 9 vs 13 buy
you. `getVoxelDistance` (no interpolation) is byte-identical, and every decision
built on top of the interpolated value — `getVoxelStatus`, `getBoxStatus`,
`getPathStatus`, `getRayStatus`, and therefore the whole RRG iteration — still
agrees exactly. The comparator allows 1e-6 relative here and nothing else.

## Layout

```
run_parity.sh            host-side runner: builds, runs both sides, compares
prepare_sources.sh       copies the packages into a scratch tree and seeds the sampler
build_and_run_ros1.sh    inside gbplanner:noetic-3.0.0 (starts its own roscore)
build_and_run_ros2.sh    inside gbplanner-ros2:jazzy-3.0.0
CMakeLists.txt           one project, -DGBP_PARITY_SIDE=ros1|ros2
compare_parity.py        the comparator, its tolerances and its deviation list
check_config_parity.py   pre-existing YAML/config checker, unrelated to Suite B
unit/parity_body.cpp     the shared test body
unit/parity_shim_ros1.h  } the only files that know which middleware is in play
unit/parity_shim_ros2.h  }
unit/parity_emit.h       the key=value result channel
unit/gbp_parity_seed.h   the seed shim forced into random_sampler.cpp
out/                     gitignored: ros1.txt, ros2.txt and the build logs
```

Tolerances and accepted deviations live in `TOLERANCES` and
`EXPECTED_DEVIATIONS` at the top of `compare_parity.py`, each with the reason it
exists. The default is bit equality; anything looser is opt-in and named.
