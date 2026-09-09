#!/usr/bin/env bash
# Suite C - does the ported stack actually explore, and does it keep exploring?
#
# Suites A and B answer "are the numbers the same as ROS 1's". Neither answers
# "does the thing work end to end for minutes at a time", and that is a separate
# question: a planner can produce a correct first path and then stall, leak, or
# stop replanning. This runs the whole stack autonomously and asserts on what
# came out.
#
# It runs the scenario launch file itself - the same one `make run-sim <name>`
# starts - rather than assembling the nodes by hand. A scenario that passes here
# is a scenario a user can run, which is the only claim worth making.
#
# Thresholds are deliberately loose. They are here to catch a stack that stopped
# working, not to pin behaviour - Suite A is what pins behaviour, against ROS 1's
# own run-to-run spread. A tight threshold here would fail on simulator jitter
# and teach everyone to ignore it.
#
#   ./run_suite_c.sh                                    # uav_cave, its own world, 180 s
#   ./run_suite_c.sh --scenario ugv_niosh --seconds 300
#   ./run_suite_c.sh --scenario uav_cave --world darpa_cave_01
#
# Exit status is non-zero if any check fails. Nothing is installed on the host;
# everything runs in a throwaway container.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
SUBT="${SUBT_CAVE_SIM:-/home/santal/git/gbplanner_ros/bootstrap/sim/subt_cave_sim}"

SCENARIO=uav_cave
SECONDS_TO_RUN=180
WORLD=""
EXTRA_ARGS=""
SETTLE=60

# scenario -> launch file, and the odometry topic to measure movement on. That
# topic has to name the robot the scenario actually spawns: "the robot moved" is
# the one check a planner cannot satisfy by planning without driving anything.
scenario_launch() {
  case "$1" in
    uav_cave)  echo "uav_gz_cave_exploration.launch.py" ;;
    uav_niosh) echo "uav_gz_niosh_exploration.launch.py" ;;
    uav_cargo) echo "uav_gz_cargo_inspection.launch.py" ;;
    ugv_niosh) echo "ugv_gz_niosh_exploration.launch.py" ;;
    ugv_urban) echo "ugv_gz_urban_exploration.launch.py" ;;
    anymal_niosh) echo "anymal_gz_niosh_exploration.launch.py" ;;
    go2) echo "go2_gz_exploration.launch.py" ;;
    *) return 1 ;;
  esac
}
scenario_odom() {
  case "$1" in
    uav_*) echo "/rmf_owl/odometry" ;;
    ugv_*) echo "/marble_husky/odometry" ;;
    anymal_*) echo "/anymal/odometry" ;;
    go2) echo "/odom" ;;
    *) return 1 ;;
  esac
}

while [ $# -gt 0 ]; do
  case "$1" in
    --scenario) SCENARIO="$2"; shift 2 ;;
    --seconds)  SECONDS_TO_RUN="$2"; shift 2 ;;
    --world)    WORLD="$2"; shift 2 ;;
    --settle)   SETTLE="$2"; shift 2 ;;
    --args)     EXTRA_ARGS="$EXTRA_ARGS $2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

LAUNCH_FILE="$(scenario_launch "$SCENARIO")" || { echo "unknown scenario: $SCENARIO" >&2; exit 2; }
ODOM_TOPIC="$(scenario_odom "$SCENARIO")"
[ -n "$WORLD" ] && EXTRA_ARGS="$EXTRA_ARGS world:=$WORLD"
# Spawn height goes with the world, not with the scenario: a robot spawned
# above the height it rests at falls, and for the anymal that corrupts the
# seed elevation_mapping puts under its feet. cave_box's floor is at zero,
# every other world's is not.
if [ "$SCENARIO" = "anymal_niosh" ] && [ "$WORLD" = "cave_box" ]; then
  EXTRA_ARGS="$EXTRA_ARGS z:=0.62"
fi

# Every world except cave_box pulls its meshes from subt_cave_sim, which is
# 3.9 GB and deliberately not vendored here.
RESOURCE_MOUNT=()
if [ "$WORLD" != "cave_box" ]; then
  [ -d "$SUBT" ] || { echo "scenario $SCENARIO needs $SUBT, which is missing" >&2; exit 2; }
  RESOURCE_MOUNT=(-v "$SUBT:/opt/subt_cave_sim:ro")
fi

OUT="$HERE/out/$SCENARIO"
mkdir -p "$OUT"; rm -f "$OUT"/*.log "$OUT"/*.json "$OUT"/*.txt

cat > "$OUT/inside.sh" <<'INNER'
# No `set -u`: ROS's setup.bash reads variables it has not defined yet
# (AMENT_TRACE_SETUP_FILES among them) and aborts under it.
set -o pipefail
source /opt/ros/jazzy/setup.bash
source /opt/deps_ws/install/setup.bash
source /workspace/gbplanner3_ws/install/setup.bash

LAUNCH_FILE="$1"; RUN_S="$2"; ODOM="$3"; SETTLE="$4"; shift 4

# One launch, exactly as `make run-sim <scenario>` runs it, minus the GUIs.
ros2 launch gbplanner "$LAUNCH_FILE" rviz:=false headless:=true "$@" \
    > /out/launch.log 2>&1 &
LAUNCH_PID=$!

# The settle time is the world's mesh load plus voxblox's first integrations,
# and it is per world rather than per stack: cave_box is up in seconds, the
# NIOSH tunnel is one 223 MB DAE.
sleep "$SETTLE"

# Sample the robot's pose so coverage can be measured from how far it travelled,
# which needs no map introspection and cannot be faked by a planner that plans
# without moving.
( for i in $(seq 1 400); do
    timeout 3 ros2 topic echo --once --field pose.pose.position "$ODOM" 2>/dev/null \
      | tr '\n' ' ' | sed 's/$/\n/'
    sleep 2
  done ) > /out/poses.txt 2>&1 &
POSE_PID=$!

# The velocity the follower is asking for, sampled the same way. A robot that
# stops moving is two different faults with the same odometry: the follower gave
# up commanding, or it is still commanding and the robot cannot execute it. One
# is control logic, the other is geometry or physics, and the pose trace alone
# cannot tell them apart. An empty line means no command was published within
# the timeout.
CMD="${ODOM%/odometry}/command/velocity"
( for i in $(seq 1 400); do
    timeout 3 ros2 topic echo --once "$CMD" 2>/dev/null \
      | tr '\n' ' ' | sed 's/$/\n/'
    sleep 2
  done ) > /out/cmd_vel.txt 2>&1 &
CMD_PID=$!

# The initialisation step, which is what the RViz panel's "Initialization"
# button calls. It exists because nothing sees the ground underneath a robot:
# driving forward leaves the robot standing on a patch it observed a moment
# earlier, on the way in. That matters for a ground robot, whose every sample
# is projected onto observed terrain - without it the planner refuses the
# robot's own position and never leaves the root vertex.
#
# Harmless where it is not configured: with init_motion_enable false the call
# returns immediately, and the aerial scenarios take off instead. Failure is
# not fatal here, only reported, because a scenario that does not offer the
# service is a scenario that does not need it.
echo "triggering initialisation motion"
timeout 60 ros2 service call /pci_initialization_trigger \
    planner_msgs/srv/PciInitialization > /out/init.log 2>&1 \
    || echo "initialisation service not available or refused; continuing"
# Let the manoeuvre finish and the map fill in behind the robot before the
# planner is asked for anything.
sleep 20

echo "triggering autonomous exploration"
timeout 60 ros2 service call /planner_control_interface/std_srvs/automatic_planning \
    std_srvs/srv/Trigger > /out/trigger.log 2>&1

sleep "$RUN_S"

kill $POSE_PID $CMD_PID 2>/dev/null
# Aliveness from the graph rather than from a PID: `ros2 launch` respawns
# nothing, so a node that died is a node that has left the graph, and an empty
# node list fails the check instead of passing it silently.
{ echo "launch alive at end: $(kill -0 $LAUNCH_PID 2>/dev/null && echo yes || echo no)"
  ros2 node list 2>/dev/null | sed 's/^/node: /'
} > /out/alive.txt

pkill -f gbplanner_node; pkill -f pci_general; pkill -f path_follower
pkill -f "gz sim"; pkill -f ruby; pkill -f parameter_bridge
sleep 2

# ros2 launch prefixes every line with the process it came from, so one log can
# be split back into the three the checker wants without running the nodes
# separately.
#
# Everything from the first shutdown signal onwards goes into its own file. A
# node that dies while being torn down and a node that dies mid-run are not the
# same finding, and folding them together means either ignoring real crashes or
# failing every run on a teardown race. The checker asserts on the run and
# reports the teardown.
SHUTDOWN_LINE=$(grep -an 'signal_handler(SIGINT/SIGTERM)' /out/launch.log | head -1 | cut -d: -f1)
if [ -n "$SHUTDOWN_LINE" ]; then
  head -n $((SHUTDOWN_LINE - 1)) /out/launch.log > /out/run.log
  tail -n +"$SHUTDOWN_LINE" /out/launch.log > /out/teardown.log
else
  cp /out/launch.log /out/run.log
  : > /out/teardown.log
fi

grep -a 'gbplanner_node' /out/run.log > /out/planner.log
grep -a 'pci_general_ros_node' /out/run.log > /out/pci.log
grep -aE 'ruby|gz_sim|spawn_|parameter_bridge|path_follower' /out/run.log > /out/sim.log
true
INNER

echo "Suite C: scenario=$SCENARIO, launch=$LAUNCH_FILE, args='$EXTRA_ARGS', ${SECONDS_TO_RUN}s after ${SETTLE}s settle"
ROOT_DIR="$REPO" IMAGE_NAME=gbplanner-ros2 ROS_DISTRO=jazzy GBPLANNER3_VERSION=3.0.0 \
NAMESPACE=suitec REBUILD_PKG=gbplanner LAUNCH_FILE=x.py RVIZ=false \
docker compose -f "$REPO/docker-compose.yml" run --rm --no-deps \
  "${RESOURCE_MOUNT[@]}" -v "$OUT:/out" dev \
  bash /out/inside.sh "$LAUNCH_FILE" "$SECONDS_TO_RUN" "$ODOM_TOPIC" "$SETTLE" $EXTRA_ARGS \
  > "$OUT/container.log" 2>&1

python3 "$HERE/check_suite_c.py" --out "$OUT" --seconds "$SECONDS_TO_RUN" --scenario "$SCENARIO"
