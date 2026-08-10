
# #!/usr/bin/env bash
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive
export APT_LISTCHANGES_FRONTEND=none

# Gazebo Garden collection, pinned to commits rather than branch heads. The
# upstream collection file this used to curl at run time
# (ntnu-arl/gz-sim, fix/position_control, collection-garden.yaml) pins every
# repository to a branch, and Garden branches still move: gz-common5 dropped
# its assimp < 5.2 compatibility path, so on Focal (assimp 5.0.1) the build now
# dies on "assimp/GltfMaterial.h: No such file or directory". See the header of
# the lock file for how to bump it.
COLLECTION_FILE=${COLLECTION_FILE:-/workspace/src/docker/collection-garden.lock.yaml}

mkdir -p gazebo_garden_ws/src
cd gazebo_garden_ws/src
test -f "$COLLECTION_FILE" || {
  echo "collection file not found: $COLLECTION_FILE" >&2
  exit 1
}
vcs import < "$COLLECTION_FILE"

# curl -fsSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list
# apt-get update

cd /workspace/gazebo_garden_ws/src
# apt-get update && apt-get -y --no-install-recommends install $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | grep -v 'keyboard-configuration' | tr '\n' ' ')
# gz-sim used to be wiped and re-cloned here because the upstream collection
# file could point it anywhere; the lock file pins the ntnu-arl fork directly,
# so vcs import above already left it at the right commit.

cd /workspace/gazebo_garden_ws
colcon graph
colcon build --cmake-args -DBUILD_TESTING=OFF -Wno-dev --merge-install

# colcon setup scripts may reference COLCON_TRACE; with `set -u` this can fail if unset.
export COLCON_TRACE=${COLCON_TRACE:-}
set +u
source /workspace/gazebo_garden_ws/install/setup.bash
set -u

cd /workspace
mkdir -p ros_gz_bridge_ws/src
cd ros_gz_bridge_ws/src

# Same reasoning as the collection lock: garden_noetic is a moving branch, so
# pin the commit the known-good workspace was built from. Re-running bootstrap
# over an existing checkout fetches instead of wiping it.
ROS_GZ_COMMIT=${ROS_GZ_COMMIT:-d5fc8e925294c0b8f31b357e7d1ffa7f3f1bdfee}
if [ -d ros_gz/.git ]; then
  git -C ros_gz fetch origin garden_noetic
else
  git clone https://github.com/ntnu-arl/ros_gz.git -b garden_noetic ros_gz
fi
git -C ros_gz checkout --detach "$ROS_GZ_COMMIT"

cd /workspace/ros_gz_bridge_ws
catkin config --install --cmake-args -Wno-dev
catkin build

set +u
source /workspace/ros_gz_bridge_ws/install/setup.bash
set -u

cd /workspace/gbplanner3_ws
vcs import < ./src/exploration/vcstool/packages.repos
cd src/sim/subt_cave_sim
git lfs pull

set +u
source /opt/ros/noetic/setup.bash
source /workspace/gazebo_garden_ws/install/local_setup.bash
source /workspace/ros_gz_bridge_ws/install/local_setup.bash

cd /workspace/gbplanner3_ws
catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
catkin build

cat >/workspace/install/setup_all.bash <<'EOF'
#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
source /workspace/gazebo_garden_ws/install/setup.bash
source /workspace/ros_gz_bridge_ws/install/setup.bash
source /workspace/gbplanner3_ws/devel/setup.bash
EOF
chmod +x /workspace/install/setup_all.bash
echo "Created unified setup script: /workspace/install/setup_all.bash"

set -u