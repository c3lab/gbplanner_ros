#!/usr/bin/env bash
# ROS 2 (Jazzy) side. Runs inside gbplanner-ros2:jazzy-3.0.0.
#   $1 output file for the emitted key=value results
set -euo pipefail

OUT="${1:-/parity/out/ros2.txt}"
SEED="${GBP_PARITY_SEED:-12345}"
TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRATCH=/tmp/gbp_parity

# The ROS setup scripts reference unset variables, so -u is lifted around them.
set +u
source /opt/ros/jazzy/setup.bash
source /opt/deps_ws/install/setup.bash
source /workspace/gbplanner3_ws/install/setup.bash
set -u

"$TEST_DIR/prepare_sources.sh" \
  /workspace/gbplanner3_ws/src/exploration "$SCRATCH/src"

cmake -S "$TEST_DIR" -B "$SCRATCH/build" \
  -DGBP_PARITY_SIDE=ros2 \
  -DGBP_PARITY_SRC="$SCRATCH/src" >/dev/null
cmake --build "$SCRATCH/build" -j"$(nproc)" >/dev/null

mkdir -p "$(dirname "$OUT")"
GBP_PARITY_SEED="$SEED" GBP_PARITY_OUT="$OUT" "$SCRATCH/build/gbp_parity"
# Second run of the identical binary. run_parity.sh requires the two to be
# byte-identical: a side that cannot reproduce itself cannot be compared to the
# other one, and saying so is cheaper than debugging a phantom divergence.
GBP_PARITY_SEED="$SEED" GBP_PARITY_OUT="$OUT.repeat" "$SCRATCH/build/gbp_parity"
echo "ros2: $(wc -l <"$OUT") result lines -> $OUT"
