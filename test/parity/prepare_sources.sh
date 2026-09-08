#!/usr/bin/env bash
# Copy the three packages under test into a scratch tree and apply the one
# behavioural patch the suite needs. Runs inside the container, so the ROS 1
# repository is only ever read.
#
#   $1  source workspace root holding kdtree/ planner_common/ map_manager/ gbplanner/
#   $2  destination scratch directory
set -euo pipefail

SRC_ROOT="$1"
DEST="$2"

rm -rf "$DEST"
mkdir -p "$DEST"
for pkg in kdtree planner_common map_manager gbplanner; do
  cp -a "$SRC_ROOT/$pkg" "$DEST/$pkg"
done
chmod -R u+w "$DEST"

# Seed injection, see unit/gbp_parity_seed.h. There are exactly three re-seed
# sites (RandomSamplerBase::reset, RandomSampler::setPDF, RandomSampler::reset);
# if that count ever changes the patch is stale and the run must fail rather
# than silently compare two entropy-seeded streams.
RS="$DEST/planner_common/src/random_sampler.cpp"
BEFORE=$(grep -c 'generator_\.seed(rd())' "$RS" || true)
if [ "$BEFORE" -ne 3 ]; then
  echo "prepare_sources: expected 3 re-seed sites in random_sampler.cpp, found $BEFORE" >&2
  exit 1
fi
sed -i 's/generator_\.seed(rd())/generator_.seed(gbp_parity_seed())/g' "$RS"
AFTER=$(grep -c 'generator_\.seed(gbp_parity_seed())' "$RS")
if [ "$AFTER" -ne 3 ]; then
  echo "prepare_sources: seed patch did not apply" >&2
  exit 1
fi

echo "prepare_sources: $SRC_ROOT -> $DEST (3 re-seed sites patched)"
