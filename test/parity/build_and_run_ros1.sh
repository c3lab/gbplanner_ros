#!/usr/bin/env bash
# ROS 1 (Noetic) side. Runs inside gbplanner:noetic-3.0.0, which is never
# rebuilt: the workspace it was built against is bind-mounted read-only and the
# packages under test are copied out of it before anything is patched.
#
#   $1 output file for the emitted key=value results
set -eo pipefail

OUT="${1:-/parity/out/ros1.txt}"
SEED="${GBP_PARITY_SEED:-12345}"
TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRATCH=/tmp/gbp_parity

source /opt/ros/noetic/setup.bash
source /workspace/gbplanner3_ws/install/setup.bash

"$TEST_DIR/prepare_sources.sh" \
  /workspace/gbplanner3_ws/src/exploration "$SCRATCH/src"

cmake -S "$TEST_DIR" -B "$SCRATCH/build" \
  -DGBP_PARITY_SIDE=ros1 \
  -DGBP_PARITY_SRC="$SCRATCH/src" >/dev/null
cmake --build "$SCRATCH/build" -j"$(nproc)" >/dev/null

# MapManagerVoxblox holds a voxblox TsdfServer by value, which is built from a
# ros::NodeHandle - so this side genuinely needs a master. It is a private one
# inside this container, never touched by anything else.
roscore -p 11399 >/tmp/roscore.log 2>&1 &
ROSCORE_PID=$!
export ROS_MASTER_URI=http://127.0.0.1:11399
for _ in $(seq 1 60); do
  rosparam list >/dev/null 2>&1 && break
  sleep 0.5
done
if ! rosparam list >/dev/null 2>&1; then
  echo "ros1: roscore did not come up" >&2
  kill "$ROSCORE_PID" 2>/dev/null || true
  exit 1
fi
trap 'kill "$ROSCORE_PID" 2>/dev/null || true' EXIT

mkdir -p "$(dirname "$OUT")"
GBP_PARITY_SEED="$SEED" GBP_PARITY_OUT="$OUT" "$SCRATCH/build/gbp_parity"
# Second run of the identical binary. run_parity.sh requires the two to be
# byte-identical: a side that cannot reproduce itself cannot be compared to the
# other one, and saying so is cheaper than debugging a phantom divergence.
GBP_PARITY_SEED="$SEED" GBP_PARITY_OUT="$OUT.repeat" "$SCRATCH/build/gbp_parity"
echo "ros1: $(wc -l <"$OUT") result lines -> $OUT"
