#!/usr/bin/env bash
# Clone the Tier C sources (third-party packages that depend on gbplanner's own
# packages) into bootstrap/. No build here: this step needs the network,
# while `make build-ws` must stay runnable offline.
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive
export GIT_SSH_COMMAND="ssh -o StrictHostKeyChecking=accept-new"

# vcstool consults known_hosts directly before spawning git, so populate it here
# rather than relying on the ssh client's own prompt (these containers are --rm,
# so nothing persists between runs).
mkdir -p ~/.ssh
ssh-keyscan github.com >> ~/.ssh/known_hosts 2>/dev/null

REPOS_FILE=${REPOS_FILE:-/workspace/src/vcstool/packages.repos}
ASSETS_FILE=${ASSETS_FILE:-/workspace/src/vcstool/assets.repos}
for f in "$REPOS_FILE" "$ASSETS_FILE"; do
  test -f "$f" || { echo "repos file not found: $f" >&2; exit 1; }
done

mkdir -p /workspace/bootstrap/exploration
mkdir -p /workspace/bootstrap/sim
mkdir -p /workspace/bootstrap/gbplanner3_ws/{build,install,log}
mkdir -p /workspace/bootstrap/.cache/gz

cd /workspace/bootstrap
# Re-running over an existing checkout fetches instead of wiping it.
vcs import --recursive < "$REPOS_FILE"

# The world models every scenario needs. `git lfs pull` is what turns the
# checkout from LFS pointer files into meshes; on a re-run it only fetches
# objects that are missing.
vcs import < "$ASSETS_FILE"
git -C /workspace/bootstrap/sim/subt_cave_sim lfs pull

echo
echo "Tier C sources in bootstrap/:"
vcs status --nested . || true
