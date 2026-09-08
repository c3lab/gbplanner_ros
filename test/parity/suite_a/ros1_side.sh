#!/usr/bin/env bash
# One ROS 1 sample run. Executes INSIDE gbplanner:noetic-3.0.0.
#
# Headless is the whole problem on this side: uav_gzc_cave_exploration.launch is
# Gazebo Classic, its Ouster is a <sensor type="gpu_ray">, and a GPU ray sensor
# is rendered by OGRE inside gzserver -- which needs a GL context, which needs
# an X display. A detached container has none. The answer here is Xvfb inside
# the container plus Mesa's llvmpipe, both already in the image: no host X
# server is touched, no window is opened on the user's desktop, and the
# simulation still runs at a real-time factor of ~1.
#
# __GLX_VENDOR_LIBRARY_NAME is explicitly cleared. docker-compose sets it to
# "nvidia" for the desktop case; against Xvfb, which has no NVIDIA GLX
# extension, forcing the NVIDIA vendor library makes every GL context fail.
#
# The launch file starts RViz unconditionally (there is no rviz_en guard on that
# node), so it is killed as soon as it registers rather than left burning CPU on
# a software rasteriser.
set -eo pipefail
export HOME=/root

CONFIG_FOLDER="${CONFIG_FOLDER:-uav/gz/cave_exploration}"
RUN_INDEX="${RUN_INDEX:-0}"
CALLS="${CALLS:-3}"
MAP_SECONDS="${MAP_SECONDS:-60}"
HOVER="${HOVER:-40.0 5.0 1.0}"
OUT=/out

Xvfb :99 -screen 0 1280x1024x24 +extension GLX +extension RANDR +render -noreset \
    >"$OUT/xvfb_run${RUN_INDEX}.log" 2>&1 &
sleep 3
export DISPLAY=:99
unset __GLX_VENDOR_LIBRARY_NAME __NV_PRIME_RENDER_OFFLOAD

source /workspace/install/setup_all.bash
export ROS_MASTER_URI=http://localhost:11311

# rosconsole writes INFO to stdout and WARN/ERROR to stderr. Redirected to a
# file, stdout is fully buffered, so every "Formed a graph with [N] vertices",
# "Best path: with gain [G]" and "Time statistics" block sits in a 4 KB buffer
# that SIGTERM at the end of the run throws away - the log then contains only
# the WARN lines and the graph observables are simply absent. Both of these are
# needed: the env var makes rosconsole flush each record, stdbuf sets the mode
# on the stream itself, and roslaunch's children inherit both.
export ROSCONSOLE_STDOUT_LINE_BUFFERED=1

stdbuf -oL -eL roslaunch gbplanner uav_gzc_cave_exploration.launch \
    gazebo_gui_en:=false config_folder:="$CONFIG_FOLDER" \
    >"$OUT/ros1_run${RUN_INDEX}.log" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 120); do
  rostopic list >/dev/null 2>&1 && break
  sleep 2
done
for _ in $(seq 1 60); do
  rosnode list 2>/dev/null | grep -q gbplanner_ui && { rosnode kill /gbplanner_ui >/dev/null 2>&1; break; }
  sleep 1
done

# set -e is deliberate here: a probe that cannot reach the hover pose or the
# service has produced no sample, and the runner must stop rather than let the
# comparator average over a short campaign.
python3 /parity/probe_ros1.py --out "$OUT/ros1_run${RUN_INDEX}.json" \
    --run "$RUN_INDEX" --calls "$CALLS" --map-seconds "$MAP_SECONDS" --hover $HOVER

# Three readings of the same service for the quaternion question: rospy (above),
# the stock command-line client, and a C++ client. If they disagree the bug is in
# a client; if they agree the response really carries what they show.
rosservice call /gbplanner "{header: {frame_id: world}, bound_mode: 0}" \
    >"$OUT/ros1_run${RUN_INDEX}_rosservice.txt" 2>&1 || true

if [ ! -x /parity/build/quat_client_ros1 ]; then
  mkdir -p /parity/build
  g++ -std=c++14 -O2 -o /parity/build/quat_client_ros1 \
      /parity/quat_client/quat_client_ros1.cpp \
      -I/opt/ros/noetic/include -I/workspace/gbplanner3_ws/devel/include \
      -L/opt/ros/noetic/lib -lroscpp -lrostime -lrosconsole \
      -lroscpp_serialization -lboost_system \
      >"$OUT/quat_client_ros1_build.log" 2>&1 || true
fi
if [ -x /parity/build/quat_client_ros1 ]; then
  /parity/build/quat_client_ros1 \
      >"$OUT/ros1_run${RUN_INDEX}_cppclient.txt" 2>&1 || true
fi

kill "$LAUNCH_PID" 2>/dev/null || true
sleep 5
pkill -f gzserver || true
pkill -f roslaunch || true
pkill -f rosmaster || true
exit 0
