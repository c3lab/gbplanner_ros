#!/usr/bin/env python3
"""Suite A probe, ROS 1 side. Drives one live gbplanner stack and records what
it produced.

Runs inside gbplanner:noetic-3.0.0 next to a live
`roslaunch gbplanner uav_gzc_cave_exploration.launch`. Its ROS 2 twin,
probe_ros2.py, records the identical JSON schema so compare_suite_a.py never has
to know which middleware produced a sample.

Three things here are deliberate rather than incidental:

  * The robot is commanded to a fixed hover pose before anything is measured.
    Left alone the ROS 1 UAV settles on the cave floor (z ~ 0, tilted by the
    terrain) while the ROS 2 one holds ~1 m; the map a 360-degree lidar builds
    from those two vantage points is not the same map, and every graph
    observable would inherit that difference.
  * The map is then integrated for a fixed span of SIMULATED time, not wall
    time. Wall time makes the sample depend on how loaded the host is.
  * /vis/planning_graph is subscribed before the first service call. Both ports
    guard that publisher with a subscriber count, so an unsubscribed probe
    silently gets no graph at all.
"""

import argparse
import json
import struct
import time

import rospy
from geometry_msgs.msg import Transform, Twist
from nav_msgs.msg import Odometry
from planner_msgs.srv import planner_srv, planner_srvRequest
from trajectory_msgs.msg import (MultiDOFJointTrajectory,
                                 MultiDOFJointTrajectoryPoint)
from visualization_msgs.msg import MarkerArray


def bits(value):
    """IEEE-754 bit pattern of a double, so a subnormal cannot hide behind %f."""
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


class Probe(object):
    def __init__(self, args):
        self.args = args
        self.odom = None
        self.graph_msg = None
        rospy.Subscriber(args.odom_topic, Odometry, self._odom_cb, queue_size=10)
        rospy.Subscriber("/vis/planning_graph", MarkerArray, self._graph_cb,
                         queue_size=2)
        self.traj_pub = rospy.Publisher(args.command_topic,
                                        MultiDOFJointTrajectory, queue_size=1)

    def _odom_cb(self, msg):
        self.odom = msg

    def _graph_cb(self, msg):
        self.graph_msg = msg

    # -- setup -------------------------------------------------------------
    def wait_for_inputs(self, timeout):
        deadline = time.time() + timeout
        while self.odom is None and time.time() < deadline:
            time.sleep(0.2)
        if self.odom is None:
            raise SystemExit("no odometry on %s after %ds" %
                             (self.args.odom_topic, timeout))
        rospy.wait_for_service("/gbplanner", timeout=timeout)
        self.planner = rospy.ServiceProxy("/gbplanner", planner_srv)

    def hover_command(self):
        msg = MultiDOFJointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.joint_names = ["base_link"]
        point = MultiDOFJointTrajectoryPoint()
        tf = Transform()
        tf.translation.x, tf.translation.y, tf.translation.z = self.args.hover
        tf.rotation.w = 1.0
        point.transforms.append(tf)
        point.velocities.append(Twist())
        point.accelerations.append(Twist())
        point.time_from_start = rospy.Duration(0.0)
        msg.points.append(point)
        return msg

    def fly_to_hover(self, timeout_sim):
        """Hold the setpoint until the robot has been inside the tolerance for
        hold_sim seconds of simulated time."""
        target = self.args.hover
        start = rospy.Time.now()
        inside_since = None
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.traj_pub.publish(self.hover_command())
            pose = self.odom.pose.pose.position
            err = max(abs(pose.x - target[0]), abs(pose.y - target[1]),
                      abs(pose.z - target[2]))
            now = rospy.Time.now()
            if err < self.args.hover_tol:
                if inside_since is None:
                    inside_since = now
                elif (now - inside_since).to_sec() >= self.args.hover_hold:
                    return (now - start).to_sec()
            else:
                inside_since = None
            if (now - start).to_sec() > timeout_sim:
                raise SystemExit("hover not reached in %ds sim (err %.2f m)" %
                                 (timeout_sim, err))
            rate.sleep()
        raise SystemExit("interrupted")

    def integrate_map(self, seconds_sim):
        """Keep publishing the setpoint while voxblox fills in, so the robot
        does not drift out of the pose the samples are taken at."""
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - start).to_sec() < seconds_sim:
            self.traj_pub.publish(self.hover_command())
            rate.sleep()

    # -- one sample --------------------------------------------------------
    def sample(self, index):
        self.graph_msg = None
        odom = self.odom
        request = planner_srvRequest()
        request.header.stamp = rospy.Time.now()
        request.header.frame_id = "world"
        request.bound_mode = 0
        wall = time.time()
        response = self.planner(request)
        wall = time.time() - wall

        # The graph markers are published from inside the call, so by the time
        # the response is back the message is usually already in; give the
        # callback thread a bounded grace period rather than assuming it.
        deadline = time.time() + 5.0
        while self.graph_msg is None and time.time() < deadline:
            time.sleep(0.05)

        return {
            "call": index,
            "sim_time": rospy.Time.now().to_sec(),
            "wall_call_s": wall,
            "odom": {
                "pos": [odom.pose.pose.position.x, odom.pose.pose.position.y,
                        odom.pose.pose.position.z],
                "quat": [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                         odom.pose.pose.orientation.z, odom.pose.pose.orientation.w],
                "stamp": odom.header.stamp.to_sec(),
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
    parser.add_argument("--odom-topic",
                        default="/rmf_obelix/ground_truth/odometry_throttled")
    parser.add_argument("--command-topic", default="/rmf_obelix/command/trajectory")
    parser.add_argument("--hover", type=float, nargs=3, default=[40.0, 5.0, 1.0])
    parser.add_argument("--hover-tol", type=float, default=0.25)
    parser.add_argument("--hover-hold", type=float, default=3.0)
    parser.add_argument("--hover-timeout", type=float, default=120.0)
    parser.add_argument("--map-seconds", type=float, default=60.0)
    parser.add_argument("--run", type=int, default=0)
    args = parser.parse_args()

    rospy.init_node("suite_a_probe", anonymous=True)
    probe = Probe(args)
    probe.wait_for_inputs(timeout=180)
    hover_secs = probe.fly_to_hover(args.hover_timeout)
    probe.integrate_map(args.map_seconds)

    samples = [probe.sample(i) for i in range(args.calls)]
    result = {
        "side": "ros1",
        "run": args.run,
        "hover_target": args.hover,
        "hover_reach_sim_s": hover_secs,
        "map_seconds": args.map_seconds,
        "samples": samples,
    }
    with open(args.out, "w") as handle:
        json.dump(result, handle, indent=1, sort_keys=True)
    print("suite_a: wrote %s (%d samples)" % (args.out, len(samples)))


if __name__ == "__main__":
    main()
