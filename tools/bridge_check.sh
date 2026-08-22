#!/usr/bin/env bash
# Bridge health check for the ANYmal deployment.
#
# Answers one question: does everything D-GHOST needs actually exist on both
# sides of the ros1_bridge? It reads the graph only - it never calls a bridged
# service, because parameter_bridge turns a failed ROS 1 call into an exception
# that kills the bridge container (ros1_bridge factory.hpp, forward_2_to_1).
#
#   ./tools/bridge_check.sh                       # NAMESPACE=robot0 ROBOT_NAME=anymal
#   NAMESPACE=ANYmal_1 ./tools/bridge_check.sh
#
# Run it on the robot, with all three stacks up (run-robot, run-robot-bridge,
# and the D-GHOST agent). Exit status is the number of failed checks.
set -uo pipefail

NAMESPACE="${NAMESPACE:-robot0}"
ROBOT_NAME="${ROBOT_NAME:-anymal}"
ROS1_CONTAINER="${ROS1_CONTAINER:-gbplanner-robot-${NAMESPACE}}"
ROS2_CONTAINER="${ROS2_CONTAINER:-d_ghost_container}"
BRIDGE_CONTAINER="${BRIDGE_CONTAINER:-ros1-bridge-robot-${NAMESPACE}}"

FAILED=0
ok()   { printf '  \033[32mOK\033[0m    %s\n' "$1"; }
bad()  { printf '  \033[31mFAIL\033[0m  %s\n' "$1"; FAILED=$((FAILED+1)); }
head_() { printf '\n\033[1m%s\033[0m\n' "$1"; }

ros1() { docker exec "$ROS1_CONTAINER" bash -ci "source /workspace/install/setup_all.bash >/dev/null 2>&1; $*" 2>/dev/null; }
ros2() { docker exec "$ROS2_CONTAINER" bash -c "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; source /ros_ws/install/setup.bash >/dev/null 2>&1; export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}; $*" 2>/dev/null; }

head_ "Containers"
for c in "$ROS1_CONTAINER" "$BRIDGE_CONTAINER" "$ROS2_CONTAINER"; do
    if grep -qx "$c" <<<"$(docker ps --format '{{.Names}}')"; then ok "$c up"; else bad "$c NOT running"; fi
done

head_ "ROS 1 graph (namespace /${ROBOT_NAME})"
R1_TOPICS="$(ros1 rostopic list)"
R1_SERVICES="$(ros1 rosservice list)"
for t in "/${ROBOT_NAME}/move_base_simple/goal" "/${ROBOT_NAME}/gbplanner_path" "/msf_core/odometry" "/${NAMESPACE}/tf" "/${NAMESPACE}/tf_static"; do
    grep -qx -- "$t" <<<"$R1_TOPICS" && ok "topic $t" || bad "topic $t MISSING"
done
for s in gbplanner/get_frontiers gbplanner/get_target_costs gbplanner/validate_frontiers \
         gbplanner/switch_operation_mode planner_control_interface/std_srvs/automatic_planning \
         planner_control_interface/std_srvs/stop; do
    grep -qx -- "/${ROBOT_NAME}/${s}" <<<"$R1_SERVICES" && ok "service /${ROBOT_NAME}/${s}" || bad "service /${ROBOT_NAME}/${s} MISSING"
done

# gbplanner_node must be ON the goal topic, not merely the PCI: the PCI only
# arms a waypoint, the local-navigation branch is what drives.
GOAL_INFO="$(ros1 "rostopic info /${ROBOT_NAME}/move_base_simple/goal")"
if grep -q gbplanner_node <<<"$GOAL_INFO"; then
    ok "gbplanner_node subscribes the goal topic"
else
    bad "gbplanner_node NOT subscribed to the goal topic (local_navigation_goal remap missing)"
fi

head_ "ROS 1 TF prefixing"
R1_NODES="$(ros1 "rosnode list")"
if grep -q tf_prefixer <<<"$R1_NODES"; then
    ok "tf_prefixer running"
else
    bad "tf_prefixer NOT running (image lacks ros-noetic-tf-remapper-cpp?)"
fi
# Captured first, not piped: `timeout` exits 124 after killing rostopic hz, and
# under `set -o pipefail` that failure would sink the whole pipeline even though
# the rate line was printed.
TF_HZ="$(ros1 "timeout 5 rostopic hz /${NAMESPACE}/tf" || true)"
if grep -q "average rate" <<<"$TF_HZ"; then
    ok "/${NAMESPACE}/tf carries data"
else
    bad "/${NAMESPACE}/tf silent - the ROS 2 fleet TF tree will be empty"
fi

# tf_remapper_cpp has no wildcards: a frame that is live on /tf but absent from
# /tf_prefixer/mappings is forwarded UNRENAMED and collides in the fleet graph.
# The launch file says to re-check this by hand after any stack change; this is
# that check, automated. The grep catches child_frame_id too - the remapper
# rewrites both ends, so an unmapped child leaks just as an unmapped parent does.
LIVE_FRAMES="$(ros1 "timeout 8 rostopic echo -n 5 /tf; timeout 6 rostopic echo -n 8 /tf_static" \
    | grep -oE 'frame_id: "[^"]+"' | sed 's/.*: "//; s/"$//' | sort -u)"
MAPPED_FRAMES="$(ros1 "rosparam get /tf_prefixer/mappings" \
    | sed -n 's/^[[:space:]]*old:[[:space:]]*//p' | tr -d '"' | sort -u)"
if [ -z "$LIVE_FRAMES" ]; then
    bad "could not read any frame off /tf - is the ROS 1 stack publishing TF at all?"
else
    UNMAPPED="$(comm -23 <(printf '%s\n' "$LIVE_FRAMES") <(printf '%s\n' "$MAPPED_FRAMES") | tr '\n' ' ')"
    if [ -z "${UNMAPPED// }" ]; then
        ok "all $(printf '%s\n' "$LIVE_FRAMES" | wc -l) live TF frames are in the prefixer mapping table"
    else
        bad "live on /tf but NOT in /tf_prefixer/mappings, these cross unrenamed: $UNMAPPED"
    fi
fi

head_ "ROS 2 graph (namespace /${NAMESPACE})"
R2_TOPICS="$(ros2 'timeout 20 ros2 topic list')"
R2_SERVICES="$(ros2 'timeout 20 ros2 service list')"
for t in "/${NAMESPACE}/move_base_simple/goal" "/${NAMESPACE}/odometry" "/${NAMESPACE}/gbplanner_path" "/${NAMESPACE}/tf"; do
    grep -qx -- "$t" <<<"$R2_TOPICS" && ok "topic $t" || bad "topic $t NOT bridged"
done
for s in gbplanner/get_frontiers gbplanner/get_target_costs gbplanner/validate_frontiers \
         gbplanner/switch_operation_mode planner_control_interface/std_srvs/automatic_planning \
         planner_control_interface/std_srvs/stop; do
    grep -qx -- "/${NAMESPACE}/${s}" <<<"$R2_SERVICES" && ok "service /${NAMESPACE}/${s}" || bad "service /${NAMESPACE}/${s} NOT bridged"
done

# The prefixed frames have to reach the global ROS 2 /tf, or every D-GHOST
# frame conversion that is not an identity fails.
ROS2_TF="$(ros2 "timeout 12 ros2 topic echo --once /tf")"
if grep -q "${NAMESPACE}/" <<<"$ROS2_TF"; then
    ok "ROS 2 /tf carries ${NAMESPACE}/-prefixed frames"
else
    bad "ROS 2 /tf has no ${NAMESPACE}/ frames"
fi

head_ "Config agreement"
GOAL_FRAME="$(ros2 "timeout 10 ros2 param get /${NAMESPACE}/task_execution_node goal_frame_id" | sed 's/.*: //')"
case "$GOAL_FRAME" in
    world) ok "task_execution_node goal_frame_id = world" ;;
    "")    bad "task_execution_node not running, or goal_frame_id unreadable" ;;
    *)     bad "goal_frame_id = '$GOAL_FRAME' - the ROS 1 tf tree only knows bare frames, setGoal will throw" ;;
esac
ODOM="$(ros2 "timeout 10 ros2 param get /${NAMESPACE}/task_execution_node odom_topic" | sed 's/.*: //')"
case "$ODOM" in
    odometry) ok "task_execution_node odom_topic = odometry" ;;
    "")       bad "odom_topic unreadable" ;;
    *)        bad "odom_topic = '$ODOM' - the bridge publishes /${NAMESPACE}/odometry" ;;
esac

# Opt-in, because a bridged service call whose ROS 1 server is down makes
# parameter_bridge throw and takes the bridge container with it. Only run this
# once the ROS 1 checks above have passed.
if [ "${PROBE:-0}" = "1" ]; then
    head_ "Live probe (PROBE=1)"
    if [ "$FAILED" -ne 0 ]; then
        bad "refusing to probe: fix the failures above first, a call into a dead ROS 1 server kills the bridge"
    else
        FR="$(ros2 "timeout 60 ros2 service call /${NAMESPACE}/gbplanner/get_frontiers planner_msgs/srv/PlannerGetFrontiers '{force_update: true, skip_gain_refresh: false}'" || true)"
        case "$FR" in
            *"success=True"*) ok  "get_frontiers answered success=True" ;;
            *"success=False"*) bad "get_frontiers answered success=False - the planner has no graph yet (no map? no elevation layer? never planned?)" ;;
            *) bad "get_frontiers did not answer" ;;
        esac
        # Same numbers a goal would carry, in the frame the ROS 1 side speaks.
        #
        # The listener starts FIRST and writes to a file. `ros2 topic pub -1` is
        # not latched and the bridge's ROS 1 publisher does not latch either, so
        # a subscriber that attaches after the publish waits for a message that
        # has already gone - for ever, since rostopic echo has no timeout of its
        # own and the docker exec outlives Ctrl-C.
        docker exec "$ROS1_CONTAINER" bash -ci \
            "source /workspace/install/setup_all.bash >/dev/null 2>&1; \
             timeout 20 rostopic echo -n1 /${ROBOT_NAME}/move_base_simple/goal \
             > /tmp/bridge_check_goal.txt 2>&1" >/dev/null 2>&1 &
        ECHO_PID=$!
        sleep 3
        ros2 "timeout 15 ros2 topic pub -1 /${NAMESPACE}/move_base_simple/goal geometry_msgs/msg/PoseStamped '{header: {frame_id: world}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'" >/dev/null
        wait "$ECHO_PID" 2>/dev/null
        GOT="$(ros1 "cat /tmp/bridge_check_goal.txt" 2>/dev/null || true)"
        if grep -q "frame_id" <<<"$GOT"; then
            ok "a ROS 2 goal arrives on the ROS 1 side"
        else
            bad "a ROS 2 goal does NOT reach ROS 1"
        fi
    fi
fi

printf '\n'
if [ "$FAILED" -eq 0 ]; then printf '\033[32mall checks passed\033[0m\n'; else printf '\033[31m%d check(s) failed\033[0m\n' "$FAILED"; fi
exit "$FAILED"
