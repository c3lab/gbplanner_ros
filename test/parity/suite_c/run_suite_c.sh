#!/usr/bin/env bash
# Suite C - does the ported stack actually explore, and does it keep exploring?
#
# Suites A and B answer "are the numbers the same as ROS 1's". Neither answers
# "does the thing work end to end for minutes at a time", and that is a separate
# question: a planner can produce a correct first path and then stall, leak, or
# stop replanning. This runs the whole stack autonomously and asserts on what
# came out.
#
# Thresholds are deliberately loose. They are here to catch a stack that stopped
# working, not to pin behaviour - Suite A is what pins behaviour, against ROS 1's
# own run-to-run spread. A tight threshold here would fail on simulator jitter
# and teach everyone to ignore it.
#
#   ./run_suite_c.sh                 # default world, 180 s of exploration
#   ./run_suite_c.sh --seconds 300 --world darpa_cave_01
#
# Exit status is non-zero if any check fails. Nothing is installed on the host;
# everything runs in a throwaway container.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
OUT="$HERE/out"
SECONDS_TO_RUN=180
WORLD=cave_box
RESOURCE_MOUNT=()

while [ $# -gt 0 ]; do
  case "$1" in
    --seconds) SECONDS_TO_RUN="$2"; shift 2 ;;
    --world)   WORLD="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

# darpa_cave_01 pulls its tiles from the ROS 1 checkout of subt_cave_sim, which
# is 3.9 GB and deliberately not vendored here.
if [ "$WORLD" = "darpa_cave_01" ]; then
  SUBT="/home/santal/git/gbplanner_ros/bootstrap/sim/subt_cave_sim"
  [ -d "$SUBT" ] || { echo "world $WORLD needs $SUBT, which is missing" >&2; exit 2; }
  RESOURCE_MOUNT=(-v "$SUBT:/opt/subt_cave_sim:ro")
fi

mkdir -p "$OUT"; rm -f "$OUT"/*.log "$OUT"/*.json

cat > "$OUT/inside.sh" <<'INNER'
# No `set -u`: ROS's setup.bash reads variables it has not defined yet
# (AMENT_TRACE_SETUP_FILES among them) and aborts under it.
set -o pipefail
source /opt/ros/jazzy/setup.bash
source /opt/deps_ws/install/setup.bash
source /workspace/gbplanner3_ws/install/setup.bash

WORLD="$1"; RUN_S="$2"
RES=""; [ -d /opt/subt_cave_sim ] && RES="resource_path:=/opt/subt_cave_sim/models"
SPAWN=""; [ "$WORLD" = "darpa_cave_01" ] && SPAWN="x:=40.0 y:=5.0 z:=1.5"

ros2 launch gbplanner_gz_sim gz_sim.launch.py headless:=true rviz:=false \
    headless_rendering:=true world:="$WORLD" $RES $SPAWN > /out/sim.log 2>&1 &
sleep 45

C=/workspace/gbplanner3_ws/install/share/gbplanner/config/uav/gz/cave_exploration
ros2 run gbplanner gbplanner_node --ros-args -p use_sim_time:=true \
  --params-file $C/gbplanner_config.yaml --params-file $C/voxblox_sim_config.yaml \
  -r odometry:=/rmf_owl/odometry -r /gbplanner_node/pointcloud:=/rmf_owl/lidar/points \
  > /out/planner.log 2>&1 &
PLANNER_PID=$!
sleep 10

ros2 run pci_general pci_general_ros_node --ros-args -p use_sim_time:=true \
  --params-file $C/planner_control_interface_sim_config.yaml \
  -r planner_server:=gbplanner_ros -r planner_homing_server:=gbplanner/homing \
  -r odometry:=/rmf_owl/odometry -r command/trajectory:=/rmf_owl/command/trajectory \
  > /out/pci.log 2>&1 &
PCI_PID=$!
sleep 15

# Sample the robot's pose so coverage can be measured from how far it travelled,
# which needs no map introspection and cannot be faked by a planner that plans
# without moving.
( for i in $(seq 1 200); do
    timeout 3 ros2 topic echo --once --field pose.pose.position /rmf_owl/odometry 2>/dev/null \
      | tr '\n' ' ' | sed 's/$/\n/'
    sleep 2
  done ) > /out/poses.txt 2>&1 &
POSE_PID=$!

echo "triggering autonomous exploration"
timeout 30 ros2 service call /planner_control_interface/std_srvs/automatic_planning \
    std_srvs/srv/Trigger > /out/trigger.log 2>&1

sleep "$RUN_S"

kill $POSE_PID 2>/dev/null
echo "planner alive at end: $(kill -0 $PLANNER_PID 2>/dev/null && echo yes || echo no)" > /out/alive.txt
echo "pci alive at end: $(kill -0 $PCI_PID 2>/dev/null && echo yes || echo no)" >> /out/alive.txt
pkill -f gbplanner_node; pkill -f pci_general; pkill -f "gz sim"; pkill -f ruby
true
INNER

echo "Suite C: world=$WORLD, $SECONDS_TO_RUN s of autonomous exploration"
ROOT_DIR="$REPO" IMAGE_NAME=gbplanner-ros2 ROS_DISTRO=jazzy GBPLANNER3_VERSION=3.0.0 \
NAMESPACE=robot0 REBUILD_PKG=gbplanner LAUNCH_FILE=x.py RVIZ=false \
docker compose -f "$REPO/docker-compose.yml" run --rm --no-deps \
  "${RESOURCE_MOUNT[@]}" -v "$OUT:/out" dev bash /out/inside.sh "$WORLD" "$SECONDS_TO_RUN" \
  > "$OUT/container.log" 2>&1

python3 "$HERE/check_suite_c.py" --out "$OUT" --seconds "$SECONDS_TO_RUN"
