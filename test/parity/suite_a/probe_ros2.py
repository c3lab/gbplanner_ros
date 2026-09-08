#!/usr/bin/env python3
"""Suite A probe, ROS 2 side. Records the same JSON schema as probe_ros1.py.

The differences from the ROS 1 probe are exactly the three the port forces:

  * the planner service is planner_msgs/srv/PlannerSrv rather than
    planner_msgs/planner_srv,
  * the hover setpoint is a nav_msgs/Path on the path topic that
    uav_path_follower_node station-keeps on, because the gz vehicle has a
    velocity interface where the ROS 1 rotors vehicle had a
    MultiDOFJointTrajectory position interface, and
  * the clock is read from a node with use_sim_time set rather than from a
    global parameter.

Everything measured, and every key written out, is identical.
"""

import argparse
import json
import struct
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from planner_msgs.srv import PlannerSrv
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import MarkerArray


def bits(value):
    return "%016x" % struct.unpack("<Q", struct.pack("<d", float(value)))[0]


def fp_class(value):
    value = float(value)
    if value != value:
        return "A"
    if value in (float("inf"), float("-inf")):
        return "I"
    if value == 0.0:
        return "Z"
    if abs(value) < 2.2250738585072014e-308:
        return "S"
    return "N"


class Probe(Node):
    def __init__(self, args):
        # use_sim_time has to be an override rather than a later set_parameters
        # call: a node that starts on system time and switches afterwards
        # reports settle intervals measured across two different clocks.
        super().__init__(
            "suite_a_probe",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.args = args
        self.odom = None
        self.graph_msg = None
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry, args.odom_topic, self._odom_cb, sensor_qos)
        self.create_subscription(MarkerArray, "/vis/planning_graph",
                                 self._graph_cb, 2)
        self.path_pub = self.create_publisher(Path, args.command_topic, 1)
        self.client = self.create_client(PlannerSrv, "/gbplanner")

    def _odom_cb(self, msg):
        self.odom = msg

    def _graph_cb(self, msg):
        self.graph_msg = msg

    def now_sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for(self, seconds_wall):
        deadline = time.time() + seconds_wall
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    # -- setup -------------------------------------------------------------
    def wait_for_inputs(self, timeout):
        deadline = time.time() + timeout
        while self.odom is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.odom is None:
            raise SystemExit("no odometry on %s after %ds" %
                             (self.args.odom_topic, timeout))
        if not self.client.wait_for_service(timeout_sec=timeout):
            raise SystemExit("/gbplanner did not appear")

    def hover_command(self):
        msg = Path()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.args.hover
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
        return msg

    def fly_to_hover(self, timeout_sim):
        target = self.args.hover
        start = self.now_sim()
        inside_since = None
        while rclpy.ok():
            self.path_pub.publish(self.hover_command())
            self.spin_for(0.1)
            position = self.odom.pose.pose.position
            err = max(abs(position.x - target[0]), abs(position.y - target[1]),
                      abs(position.z - target[2]))
            now = self.now_sim()
            if err < self.args.hover_tol:
                if inside_since is None:
                    inside_since = now
                elif now - inside_since >= self.args.hover_hold:
                    return now - start
            else:
                inside_since = None
            if now - start > timeout_sim:
                raise SystemExit("hover not reached in %ds sim (err %.2f m)" %
                                 (timeout_sim, err))
        raise SystemExit("interrupted")

    def integrate_map(self, seconds_sim):
        start = self.now_sim()
        while self.now_sim() - start < seconds_sim:
            self.path_pub.publish(self.hover_command())
            self.spin_for(0.1)

    # -- one sample --------------------------------------------------------
    def sample(self, index):
        self.graph_msg = None
        odom = self.odom
        request = PlannerSrv.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = "world"
        request.bound_mode = 0
        wall = time.time()
        future = self.client.call_async(request)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        response = future.result()
        wall = time.time() - wall

        deadline = time.time() + 5.0
        while self.graph_msg is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        stamp = odom.header.stamp
        return {
            "call": index,
            "sim_time": self.now_sim(),
            "wall_call_s": wall,
            "odom": {
                "pos": [odom.pose.pose.position.x, odom.pose.pose.position.y,
                        odom.pose.pose.position.z],
                "quat": [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                         odom.pose.pose.orientation.z, odom.pose.pose.orientation.w],
                "stamp": stamp.sec + stamp.nanosec * 1e-9,
            },
            "status": int(response.status),
            "planning_bound_mode": int(response.planning_bound_mode),
            "path": [[p.position.x, p.position.y, p.position.z,
                      p.orientation.x, p.orientation.y, p.orientation.z,
                      p.orientation.w] for p in response.path],
            "path_quat_bits": [[bits(p.orientation.x), bits(p.orientation.y),
                                bits(p.orientation.z), bits(p.orientation.w)]
                               for p in response.path],
            "path_quat_class": ["".join([fp_class(p.orientation.x),
                                         fp_class(p.orientation.y),
                                         fp_class(p.orientation.z),
                                         fp_class(p.orientation.w)])
                                for p in response.path],
            "graph": self._graph_summary(),
        }

    def _graph_summary(self):
        if self.graph_msg is None:
            return None
        edges, vertices = [], []
        for marker in self.graph_msg.markers:
            if marker.ns == "edges":
                pts = marker.points
                edges = [[pts[i].x, pts[i].y, pts[i].z,
                          pts[i + 1].x, pts[i + 1].y, pts[i + 1].z]
                         for i in range(0, len(pts) - 1, 2)]
            elif marker.ns == "vertices":
                # One SPHERE_LIST marker holding every vertex, not one marker
                # per vertex: counting markers here would always report 1.
                vertices = [[p.x, p.y, p.z] for p in marker.points]
        return {"n_vertices": len(vertices), "n_edges": len(edges),
                "vertices": vertices, "edges": edges}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--calls", type=int, default=3)
    parser.add_argument("--odom-topic", default="/rmf_owl/odometry")
    parser.add_argument("--command-topic", default="/gbplanner_path")
    parser.add_argument("--hover", type=float, nargs=3, default=[40.0, 5.0, 1.0])
    parser.add_argument("--hover-tol", type=float, default=0.25)
    parser.add_argument("--hover-hold", type=float, default=3.0)
    parser.add_argument("--hover-timeout", type=float, default=120.0)
    parser.add_argument("--map-seconds", type=float, default=60.0)
    parser.add_argument("--run", type=int, default=0)
    args = parser.parse_args()

    rclpy.init()
    probe = Probe(args)
    probe.wait_for_inputs(timeout=180)
    hover_secs = probe.fly_to_hover(args.hover_timeout)
    probe.integrate_map(args.map_seconds)

    samples = [probe.sample(i) for i in range(args.calls)]
    result = {
        "side": "ros2",
        "run": args.run,
        "hover_target": args.hover,
        "hover_reach_sim_s": hover_secs,
        "map_seconds": args.map_seconds,
        "samples": samples,
    }
    with open(args.out, "w") as handle:
        json.dump(result, handle, indent=1, sort_keys=True)
    print("suite_a: wrote %s (%d samples)" % (args.out, len(samples)))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
