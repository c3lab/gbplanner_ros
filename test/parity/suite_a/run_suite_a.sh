#!/usr/bin/env bash
# Suite A runner. Brings up the ROS 1 stack in its own simulator and the ROS 2
# port in its own, on the same DARPA cave, samples both, and compares them with
# the ROS1-vs-ROS1 spread as the tolerance.
#
#   ./run_suite_a.sh                      # 5 runs a side, 3 planner calls a run
#   ./run_suite_a.sh --runs 3 --calls 2
#   ./run_suite_a.sh --runs 1 --start-index 4 --sides ros2   # redo one run
#   ./run_suite_a.sh --skip-ws-build
#   ./run_suite_a.sh --compare-only       # re-run the comparator over out/
#
# Everything happens in throwaway containers. /home/santal/git/gbplanner_ros is
# mounted read-only in full and is never written; neither existing image is
# rebuilt or retagged. Runs are strictly sequential: both containers use
# --network host, and two roscores or two DDS graphs on one host would see each
# other's topics.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_ROOT="$(cd "$HERE/../../.." && pwd)"
ROS1_ROOT="${GBP_ROS1_ROOT:-/home/santal/git/gbplanner_ros}"
ROS1_IMAGE="${GBP_ROS1_IMAGE:-gbplanner:noetic-3.0.0}"
ROS2_IMAGE="${GBP_ROS2_IMAGE:-gbplanner-ros2:jazzy-3.0.0}"
OUT_DIR="$HERE/out"

RUNS=5
# Index the first run is numbered with. Redoing a single run in place, without
# touching the rest of a campaign, is --runs 1 --start-index N.
START_INDEX=0
CALLS=3
MAP_SECONDS=60
HOVER="40.0 5.0 1.0"
# The planner configuration both sides load. The ROS 2 YAMLs were generated from
# the ROS 1 uav/gz/cave_exploration ones, so pointing the ROS 1 launch at that
# folder makes the two sides run the same planner parameters while still using
# the launch the user reported working. uav/gzc/cave_exploration is the launch's
# own default and differs substantially (kBasic instead of kBatch graph
# building, 0.40 m voxels instead of 0.30, 15 m lidar gain range instead of 20);
# a run with it measures a different planner, not a different middleware.
CONFIG_FOLDER="uav/gz/cave_exploration"
SKIP_WS_BUILD=0
COMPARE_ONLY=0
SIDES="ros1 ros2"

while [ $# -gt 0 ]; do
  case "$1" in
    --runs) RUNS="$2"; shift 2 ;;
    --start-index) START_INDEX="$2"; shift 2 ;;
    --calls) CALLS="$2"; shift 2 ;;
    --map-seconds) MAP_SECONDS="$2"; shift 2 ;;
    --config-folder) CONFIG_FOLDER="$2"; shift 2 ;;
    --sides) SIDES="$2"; shift 2 ;;
    --skip-ws-build) SKIP_WS_BUILD=1; shift ;;
    --compare-only) COMPARE_ONLY=1; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

mkdir -p "$OUT_DIR"

# ---------------------------------------------------------------------------
# Mount sets, mirroring each repository's own docker-compose x-common-volumes.
# The ROS 1 set is the compose list verbatim with :ro appended everywhere.
# ---------------------------------------------------------------------------
ros1_mounts() {
  local R="$ROS1_ROOT" p
  echo "-v;$R/bootstrap/gazebo_garden_ws:/workspace/gazebo_garden_ws:ro"
  echo "-v;$R/bootstrap/ros_gz_bridge_ws:/workspace/ros_gz_bridge_ws:ro"
  for p in BehaviorTree.CPP adaptive_obb_ros gbplanner_ros manhole_detector_ros pci_general; do
    echo "-v;$R/bootstrap/exploration/$p:/workspace/gbplanner3_ws/src/exploration/$p:ro"
  done
  echo "-v;$R/bootstrap/mapping/voxblox:/workspace/gbplanner3_ws/src/mapping/voxblox:ro"
  for p in catkin_boost_python_buildtool catkin_simple eigen_catkin eigen_checks \
           gflags_catkin glog_catkin kindr mav_comm minkindr minkindr_ros \
           multi_dof_joint_trajectory_rviz_plugins numpy_eigen protobuf_catkin; do
    echo "-v;$R/bootstrap/misc/$p:/workspace/gbplanner3_ws/src/misc/$p:ro"
  done
  for p in arl_gazebo_sim_ros lidar_simulator rotors_simulator smb_simulator subt_cave_sim; do
    echo "-v;$R/bootstrap/sim/$p:/workspace/gbplanner3_ws/src/sim/$p:ro"
  done
  for p in gbplanner gbplanner_ui kdtree map_manager planner_common \
           planner_control_interface planner_gazebo_sim planner_msgs \
           planner_semantic_msgs ugv_simulator vcstool; do
    echo "-v;$R/$p:/workspace/gbplanner3_ws/src/exploration/$p:ro"
  done
  echo "-v;$R/bootstrap/gbplanner3_ws/build:/workspace/gbplanner3_ws/build:ro"
  echo "-v;$R/bootstrap/gbplanner3_ws/logs:/workspace/gbplanner3_ws/log:ro"
  echo "-v;$R/bootstrap/gbplanner3_ws/install:/workspace/gbplanner3_ws/install:ro"
  echo "-v;$R/bootstrap/gbplanner3_ws/devel:/workspace/gbplanner3_ws/devel:ro"
  echo "-v;$R/bootstrap/install:/workspace/install:ro"
}

ros2_mounts() {
  local R="$ROS2_ROOT" p
  for p in planner_msgs planner_semantic_msgs kdtree planner_common map_manager \
           planner_control_interface gbplanner gbplanner_ui; do
    echo "-v;$R/$p:/workspace/gbplanner3_ws/src/exploration/$p"
  done
  echo "-v;$R/sim/gbplanner_gz_sim:/workspace/gbplanner3_ws/src/sim/gbplanner_gz_sim"
  echo "-v;$R/sim/gbplanner_gz_control:/workspace/gbplanner3_ws/src/sim/gbplanner_gz_control"
  echo "-v;$R/bootstrap/exploration:/workspace/gbplanner3_ws/src/exploration/third_party"
  echo "-v;$R/tools:/workspace/tools"
  echo "-v;$R/bootstrap/gbplanner3_ws/build:/workspace/gbplanner3_ws/build"
  echo "-v;$R/bootstrap/gbplanner3_ws/install:/workspace/gbplanner3_ws/install"
  echo "-v;$R/bootstrap/gbplanner3_ws/log:/workspace/gbplanner3_ws/log"
  echo "-v;$R/bootstrap/.cache/gz:/root/.gz"
  # The cave tiles are ~3.9 GB of git-lfs models and live only in the ROS 1
  # repository. Read-only, and the same bytes both sides render.
  echo "-v;$ROS1_ROOT/bootstrap/sim/subt_cave_sim:/opt/subt_cave_sim:ro"
}

read_mounts() {  # turn the ;-separated lines above into an argv array
  local -n dest="$1"; shift
  dest=()
  while IFS=';' read -r flag value; do
    [ -n "$flag" ] && dest+=("$flag" "$value")
  done < <("$@")
}

if [ "$COMPARE_ONLY" -eq 0 ] && [ "$SKIP_WS_BUILD" -eq 0 ]; then
  echo "== building the ROS 2 workspace (planner + gz sim packages) =="
  mkdir -p "$ROS2_ROOT/bootstrap/gbplanner3_ws"/{build,install,log} \
           "$ROS2_ROOT/bootstrap/.cache/gz"
  read_mounts m2 ros2_mounts
  docker run --rm "${m2[@]}" --entrypoint bash "$ROS2_IMAGE" -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /opt/deps_ws/install/setup.bash
    cd /workspace/gbplanner3_ws
    colcon build --merge-install --symlink-install \
      --packages-up-to gbplanner gbplanner_gz_sim gbplanner_gz_control \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev' \
    >"$OUT_DIR/ws_build.log" 2>&1 || { tail -40 "$OUT_DIR/ws_build.log"; exit 1; }
fi

run_ros1() {
  local index="$1"
  read_mounts m1 ros1_mounts
  docker rm -f "suite_a_ros1_$index" >/dev/null 2>&1 || true
  docker run --rm --name "suite_a_ros1_$index" \
    --network host --ipc host --privileged --gpus all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e RUN_INDEX="$index" -e CALLS="$CALLS" -e MAP_SECONDS="$MAP_SECONDS" \
    -e HOVER="$HOVER" -e CONFIG_FOLDER="$CONFIG_FOLDER" \
    "${m1[@]}" -v "$HERE:/parity" -v "$OUT_DIR:/out" \
    "$ROS1_IMAGE" bash -lc /parity/ros1_side.sh
}

run_ros2() {
  local index="$1"
  read_mounts m2 ros2_mounts
  docker rm -f "suite_a_ros2_$index" >/dev/null 2>&1 || true
  docker run --rm --name "suite_a_ros2_$index" \
    --network host --ipc host --privileged --gpus all \
    -e NVIDIA_DRIVER_CAPABILITIES=all -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RUN_INDEX="$index" -e CALLS="$CALLS" -e MAP_SECONDS="$MAP_SECONDS" \
    -e HOVER="$HOVER" \
    "${m2[@]}" -v "$HERE:/parity" -v "$OUT_DIR:/out" \
    --entrypoint bash "$ROS2_IMAGE" -lc /parity/ros2_side.sh
}

if [ "$COMPARE_ONLY" -eq 0 ]; then
  for side in $SIDES; do
    for i in $(seq "$START_INDEX" $((START_INDEX + RUNS - 1))); do
      echo "== $side run $i =="
      if [ "$side" = "ros1" ]; then run_ros1 "$i"; else run_ros2 "$i"; fi
    done
  done
fi

python3 "$HERE/compare_suite_a.py" "$OUT_DIR"
