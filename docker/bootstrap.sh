
# #!/usr/bin/env bash
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive
export APT_LISTCHANGES_FRONTEND=none

mkdir -p gazebo_garden_ws/src
cd gazebo_garden_ws/src
curl -fsSL -o collection-garden.yaml https://raw.githubusercontent.com/ntnu-arl/gz-sim/refs/heads/fix/position_control/collection-garden.yaml
vcs import < collection-garden.yaml

# curl -fsSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list
# apt-get update

cd /workspace/gazebo_garden_ws/src
# apt-get update && apt-get -y --no-install-recommends install $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | grep -v 'keyboard-configuration' | tr '\n' ' ')
rm -rf /workspace/gazebo_garden_ws/src/gz-sim
cd /workspace/gazebo_garden_ws/src
git clone https://github.com/ntnu-arl/gz-sim.git -b dev/multicopter_control


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

rm -rf /workspace/ros_gz_bridge_ws/src/ros_gz
git clone https://github.com/ntnu-arl/ros_gz.git -b garden_noetic
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