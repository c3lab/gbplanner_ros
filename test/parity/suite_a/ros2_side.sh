#!/usr/bin/env bash
# One ROS 2 sample run. Executes INSIDE gbplanner-ros2:jazzy-3.0.0.
#
# Headless on this side is a different problem from ROS 1's. gz Harmonic renders
# its gpu_lidar through ogre2, which wants OpenGL 3.3; `gz sim -s` with no
# display falls back to EGL, and EGL in this image finds only Mesa's ICD, tries
# the DRM devices, fails eglInitialize on all of them and then segfaults inside
# Ogre's material parser. The NVIDIA EGL vendor ICD is missing purely because
# the container toolkit does not install it, while libEGL_nvidia.so.0 IS
# mounted. Writing the one-line ICD json into the CONTAINER (never the image,
# never the host) makes --headless-rendering pick the GPU and the whole thing
# runs at RTF ~1 with no X server at all.
#
# --headless-rendering has to be smuggled through gz_verbosity because
# gz_sim.launch.py builds its gz argument string as "-r -v <gz_verbosity> -s
# <world>" and exposes no other hook. Editing the launch file to add a proper
# argument is the right fix; it is not this test's to make.
set -eo pipefail
export HOME=/root

RUN_INDEX="${RUN_INDEX:-0}"
CALLS="${CALLS:-3}"
MAP_SECONDS="${MAP_SECONDS:-60}"
HOVER="${HOVER:-40.0 5.0 1.0}"
SPAWN_Z="${SPAWN_Z:-1.0}"
LIDAR_H="${LIDAR_H:-512}"
LIDAR_V="${LIDAR_V:-64}"
OUT=/out

mkdir -p /usr/share/glvnd/egl_vendor.d
cat > /usr/share/glvnd/egl_vendor.d/10_nvidia.json <<'JSON'
{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}
JSON

source /opt/ros/jazzy/setup.bash
source /opt/deps_ws/install/setup.bash
source /workspace/gbplanner3_ws/install/setup.bash

# path_topic moves the vehicle's path follower off /gbplanner_path and onto a
# topic only the probe writes. The ROS 1 simulator has nothing subscribed to
# /gbplanner_path, so leaving the follower there would let the ROS 2 robot fly
# off on a planner output the ROS 1 robot ignores.
ros2 launch gbplanner_gz_sim gz_sim.launch.py \
    headless:=true rviz:=false world:=darpa_cave_01 \
    resource_path:=/opt/subt_cave_sim/models \
    x:=40.0 y:=5.0 z:="$SPAWN_Z" \
    lidar_horizontal_samples:="$LIDAR_H" lidar_vertical_samples:="$LIDAR_V" \
    path_topic:=/suite_a/hover_path \
    gz_verbosity:="1 --headless-rendering" \
    >"$OUT/ros2_sim_run${RUN_INDEX}.log" 2>&1 &
SIM_PID=$!

for _ in $(seq 1 120); do
  timeout 5 ros2 topic echo --once /rmf_owl/lidar/points >/dev/null 2>&1 && break
  sleep 2
done

ros2 launch gbplanner gbplanner.launch.py \
    rviz:=false use_sim_time:=true \
    odometry_topic:=/rmf_owl/odometry \
    pointcloud_topic:=/rmf_owl/lidar/points \
    command_trajectory_topic:=/rmf_owl/command/trajectory \
    >"$OUT/ros2_run${RUN_INDEX}.log" 2>&1 &
PLANNER_PID=$!

# set -e is deliberate here: a probe that cannot reach the hover pose or the
# service has produced no sample, and the runner must stop rather than let the
# comparator average over a short campaign.
python3 /parity/probe_ros2.py --out "$OUT/ros2_run${RUN_INDEX}.json" \
    --run "$RUN_INDEX" --calls "$CALLS" --map-seconds "$MAP_SECONDS" \
    --command-topic /suite_a/hover_path --hover $HOVER

# The quaternion question needs the same response read three ways: the rclpy
# client above, the stock CLI client, and a C++ client.
ros2 service call /gbplanner planner_msgs/srv/PlannerSrv \
    "{header: {frame_id: world}, bound_mode: 0}" \
    >"$OUT/ros2_run${RUN_INDEX}_servicecall.txt" 2>&1 || true

if [ ! -x /parity/build/quat_client/quat_client ]; then
  cmake -S /parity/quat_client -B /parity/build/quat_client \
      -DCMAKE_BUILD_TYPE=Release >"$OUT/quat_client_build.log" 2>&1
  cmake --build /parity/build/quat_client -j4 >>"$OUT/quat_client_build.log" 2>&1
fi
/parity/build/quat_client/quat_client \
    >"$OUT/ros2_run${RUN_INDEX}_cppclient.txt" 2>&1 || true

kill "$PLANNER_PID" 2>/dev/null || true
kill "$SIM_PID" 2>/dev/null || true
sleep 5
pkill -f 'gz sim' || true
pkill -f ruby || true
exit 0
