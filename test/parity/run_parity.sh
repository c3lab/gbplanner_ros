#!/usr/bin/env bash
# Suite B runner. Builds and runs one shared test body against both workspaces,
# in containers, and diffs the two result files. Exits non-zero on any
# difference the comparator cannot account for.
#
# Nothing is installed on the host and nothing under the ROS 1 repository is
# written: its packages and its prebuilt workspace are bind-mounted read-only,
# and the copy that gets patched lives inside the throwaway container.
#
#   ./run_parity.sh [--seed N] [--skip-ws-build] [--verbose]
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_ROOT="$(cd "$HERE/../.." && pwd)"
ROS1_ROOT="${GBP_ROS1_ROOT:-/home/santal/git/gbplanner_ros}"
ROS2_IMAGE="${GBP_ROS2_IMAGE:-gbplanner-ros2:jazzy-3.0.0}"
ROS1_IMAGE="${GBP_ROS1_IMAGE:-gbplanner:noetic-3.0.0}"
OUT_DIR="$HERE/out"
SEED=12345
SKIP_WS_BUILD=0
VERBOSE=""

while [ $# -gt 0 ]; do
  case "$1" in
    --seed) SEED="$2"; shift 2 ;;
    --skip-ws-build) SKIP_WS_BUILD=1; shift ;;
    --verbose) VERBOSE="--verbose"; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

mkdir -p "$OUT_DIR"

# ---------------------------------------------------------------------------
# Mount sets. The in-container workspace paths are identical on both sides on
# purpose: prepare_sources.sh then takes the same argument for both.
# ---------------------------------------------------------------------------
ros2_mounts=()
for pkg in planner_msgs planner_semantic_msgs kdtree planner_common map_manager \
           planner_control_interface gbplanner gbplanner_ui; do
  ros2_mounts+=(-v "$ROS2_ROOT/$pkg:/workspace/gbplanner3_ws/src/exploration/$pkg")
done
ros2_mounts+=(
  -v "$ROS2_ROOT/bootstrap/exploration:/workspace/gbplanner3_ws/src/exploration/third_party"
  -v "$ROS2_ROOT/sim/gbplanner_gz_sim:/workspace/gbplanner3_ws/src/sim/gbplanner_gz_sim"
  -v "$ROS2_ROOT/sim/gbplanner_gz_control:/workspace/gbplanner3_ws/src/sim/gbplanner_gz_control"
  -v "$ROS2_ROOT/bootstrap/gbplanner3_ws/build:/workspace/gbplanner3_ws/build"
  -v "$ROS2_ROOT/bootstrap/gbplanner3_ws/install:/workspace/gbplanner3_ws/install"
  -v "$ROS2_ROOT/bootstrap/gbplanner3_ws/log:/workspace/gbplanner3_ws/log"
  -v "$ROS2_ROOT/test:/workspace/test"
  -v "$OUT_DIR:/parity/out"
)

ros1_mounts=()
for pkg in kdtree planner_common map_manager gbplanner planner_msgs planner_semantic_msgs; do
  ros1_mounts+=(-v "$ROS1_ROOT/$pkg:/workspace/gbplanner3_ws/src/exploration/$pkg:ro")
done
ros1_mounts+=(
  -v "$ROS1_ROOT/bootstrap/gbplanner3_ws/install:/workspace/gbplanner3_ws/install:ro"
  -v "$ROS1_ROOT/bootstrap/gbplanner3_ws/devel:/workspace/gbplanner3_ws/devel:ro"
  -v "$ROS2_ROOT/test:/workspace/test:ro"
  -v "$OUT_DIR:/parity/out"
)

# ---------------------------------------------------------------------------
# The ROS 2 side needs planner_msgs and planner_semantic_msgs installed before
# the parity binary can link; the ROS 1 side is already installed inside the
# image's bind-mounted workspace and is never rebuilt.
# ---------------------------------------------------------------------------
if [ "$SKIP_WS_BUILD" -eq 0 ]; then
  echo "== building the ROS 2 workspace up to gbplanner =="
  mkdir -p "$ROS2_ROOT/bootstrap/gbplanner3_ws"/{build,install,log}
  docker run --rm "${ros2_mounts[@]}" --entrypoint bash "$ROS2_IMAGE" -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /opt/deps_ws/install/setup.bash
    cd /workspace/gbplanner3_ws
    colcon build --merge-install --symlink-install \
      --packages-up-to gbplanner \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev' >"$OUT_DIR/ws_build.log" 2>&1 \
    || { echo "workspace build failed, see $OUT_DIR/ws_build.log" >&2; exit 1; }
fi

echo "== ROS 1 side =="
docker run --rm "${ros1_mounts[@]}" -e "GBP_PARITY_SEED=$SEED" \
  --entrypoint bash "$ROS1_IMAGE" \
  -lc '/workspace/test/parity/build_and_run_ros1.sh /parity/out/ros1.txt' \
  >"$OUT_DIR/ros1.log" 2>&1 \
  || { echo "ROS 1 side failed, see $OUT_DIR/ros1.log" >&2; tail -30 "$OUT_DIR/ros1.log" >&2; exit 1; }
tail -1 "$OUT_DIR/ros1.log"

echo "== ROS 2 side =="
docker run --rm "${ros2_mounts[@]}" -e "GBP_PARITY_SEED=$SEED" \
  --entrypoint bash "$ROS2_IMAGE" \
  -lc '/workspace/test/parity/build_and_run_ros2.sh /parity/out/ros2.txt' \
  >"$OUT_DIR/ros2.log" 2>&1 \
  || { echo "ROS 2 side failed, see $OUT_DIR/ros2.log" >&2; tail -30 "$OUT_DIR/ros2.log" >&2; exit 1; }
tail -1 "$OUT_DIR/ros2.log"

status=0

echo
echo "== self-determinism =="
for side in ros1 ros2; do
  if cmp -s "$OUT_DIR/$side.txt" "$OUT_DIR/$side.txt.repeat"; then
    echo "$side: two runs of the same binary are byte-identical"
  else
    echo "$side: THE SAME BINARY DID NOT REPRODUCE ITSELF" >&2
    diff <(cut -d' ' -f1 "$OUT_DIR/$side.txt") \
         <(cut -d' ' -f1 "$OUT_DIR/$side.txt.repeat") | head -20 >&2
    status=1
  fi
done

echo
echo "== ROS 1 vs ROS 2 =="
python3 "$HERE/compare_parity.py" "$OUT_DIR/ros1.txt" "$OUT_DIR/ros2.txt" $VERBOSE || status=1

exit $status
