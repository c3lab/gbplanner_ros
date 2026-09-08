#!/usr/bin/env bash
# Clone the Tier C sources (third-party packages that depend on gbplanner's own
# packages) into bootstrap/. No build here: this step needs the host ssh-agent,
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
test -f "$REPOS_FILE" || { echo "repos file not found: $REPOS_FILE" >&2; exit 1; }

mkdir -p /workspace/bootstrap/exploration
mkdir -p /workspace/bootstrap/gbplanner3_ws/{build,install,log}
mkdir -p /workspace/bootstrap/.cache/gz

cd /workspace/bootstrap
# Re-running over an existing checkout fetches instead of wiping it.
vcs import --recursive < "$REPOS_FILE"

echo
echo "Tier C sources in bootstrap/:"
vcs status --nested . || true
