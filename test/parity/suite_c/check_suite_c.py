#!/usr/bin/env python3
"""Assert on what a Suite C run produced.

Every check is written so that "nothing happened" fails rather than passes. That
sounds obvious and is the thing this session got wrong repeatedly: a grep for
errors over an empty log is silent, a node list is empty when discovery has not
converged, and both read as success. So each check names the evidence it needs
and fails when the evidence is absent.

Thresholds are loose on purpose. Suite A pins behaviour against ROS 1's own
run-to-run spread; this one only distinguishes a working stack from a stalled
one, and a threshold tight enough to trip on simulator jitter would just get
ignored.
"""

import argparse
import json
import math
import re
import sys
from pathlib import Path

# A planning iteration that produced a graph. rrg.cpp logs one of these per call.
GRAPH_RE = re.compile(r"Formed a graph with \[(\d+)\] vertices and \[(\d+)\] edges")
GAIN_RE = re.compile(r"Best path: with gain \[([0-9.]+)\]")
TOTAL_RE = re.compile(r"Total\s+:\s+([0-9.]+)\s+\(ms\)")
# The evidence that the three-second wait in runGlobalPlanner does something.
ODOM_MOVE_RE = re.compile(r"odometry moved ([0-9.]+) m during the wait")
POSE_RE = re.compile(r"x:\s*(-?[0-9.eE+-]+)\s+y:\s*(-?[0-9.eE+-]+)\s+z:\s*(-?[0-9.eE+-]+)")

# Anything here in a node log is a hard failure regardless of the other numbers.
FATAL_PATTERNS = [
    ("Resource deadlock avoided", "a thread locked a mutex it already held"),
    ("terminate called", "unhandled exception"),
    ("Segmentation fault", "segfault"),
    ("what():", "exception escaped to the top level"),
]


class Result:
    def __init__(self):
        self.failures = []
        self.notes = []

    def check(self, ok, name, detail):
        mark = "PASS" if ok else "FAIL"
        print(f"  [{mark}] {name}: {detail}")
        if not ok:
            self.failures.append(f"{name}: {detail}")

    def note(self, text):
        print(f"  [note] {text}")
        self.notes.append(text)


def read(path):
    return path.read_text(errors="replace") if path.exists() else ""


def travelled(poses_text):
    """Total path length from the sampled odometry, in metres."""
    points = [(float(x), float(y), float(z)) for x, y, z in POSE_RE.findall(poses_text)]
    total = 0.0
    for a, b in zip(points, points[1:]):
        total += math.dist(a, b)
    return total, points


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--seconds", type=float, required=True)
    parser.add_argument("--scenario", default="uav_cave")
    args = parser.parse_args()

    planner = read(args.out / "planner.log")
    pci = read(args.out / "pci.log")
    sim = read(args.out / "sim.log")
    poses = read(args.out / "poses.txt")
    alive = read(args.out / "alive.txt")
    teardown = read(args.out / "teardown.log")

    print(f"Suite C - autonomous exploration: {args.scenario}")
    print(f"  run length: {args.seconds:.0f} s of wall clock\n")

    result = Result()

    # 0. The logs have to exist at all. Without this, every later check would
    #    "pass" on an empty string.
    result.check(bool(planner.strip()), "planner produced output",
                 f"{len(planner.splitlines())} lines")
    result.check(bool(pci.strip()), "pci produced output",
                 f"{len(pci.splitlines())} lines")
    result.check(bool(sim.strip()), "simulation produced output",
                 f"{len(sim.splitlines())} lines")
    # The crash checks below run on the log up to the shutdown signal. If there
    # is no teardown section the split did not happen, and they cover shutdown
    # too - the safe direction, but worth saying out loud.
    if not teardown.strip():
        result.note("no teardown section: the run log was not split, so the "
                    "crash checks below cover shutdown as well")
    if result.failures:
        print("\nnothing ran; the remaining checks would be meaningless")
        return 1

    # 1. Nothing crashed.
    for text, who in ((planner, "planner"), (pci, "pci")):
        for pattern, meaning in FATAL_PATTERNS:
            hits = text.count(pattern)
            result.check(hits == 0, f"{who} free of '{pattern}'",
                         "none" if hits == 0 else f"{hits}x - {meaning}")

    # Crashes while the launch was being torn down. Reported rather than
    # asserted on: they say nothing about whether the stack works, and
    # pci_general has a known one - rclcpp throws "context cannot be slept with
    # because it's invalid" if SIGINT lands while it is sleeping between planner
    # calls.
    for pattern, meaning in FATAL_PATTERNS:
        if pattern in teardown:
            result.note(f"during shutdown only: '{pattern}' ({meaning})")

    # Both halves of the stack have to still be in the ROS graph. An empty node
    # list fails this, which is the point: "no errors found" over a stack that
    # died two minutes ago is the failure mode this suite exists to catch.
    nodes = [line[len("node: "):] for line in alive.splitlines()
             if line.startswith("node: ")]
    survivors = [n for n in ("gbplanner_node", "pci_general_ros_node")
                 if any(n in node for node in nodes)]
    result.check(
        "launch alive at end: yes" in alive and len(survivors) == 2,
        "nodes still alive at the end",
        f"{len(nodes)} nodes in the graph, of which {survivors or 'neither of the two wanted'}")

    # 2. It planned, repeatedly. One graph is a first path; the regression this
    #    guards against is a stack that plans once and then stops.
    graphs = [(int(v), int(e)) for v, e in GRAPH_RE.findall(planner)]
    first_real = None
    result.check(len(graphs) >= 3, "planned repeatedly",
                 f"{len(graphs)} graphs built")
    if graphs:
        verts = [v for v, _ in graphs]
        # A degenerate graph right after "reset the map" is expected: the robot has
        # not moved, so voxblox has almost nothing integrated and there is nowhere
        # to sample. What would be a real failure is degenerating again later, so
        # the warm-up prefix is allowed any length and everything after the first
        # real graph must stay real.
        first_real = next((i for i, v in enumerate(verts) if v > 10), None)
        if first_real is None:
            result.check(False, "the planner ever built a real graph",
                         f"largest was {max(verts)} vertices over {len(verts)} calls")
        else:
            # The warm-up prefix is allowed any length, which on its own lets a
            # run pass while being degenerate for nearly all of it: ugv_niosh
            # reported 1403 graphs of which 1392 were the root vertex alone, the
            # first real one arriving 27 s in and eleven calls before the run
            # ended. Nothing *after* the warm-up was wrong, so every check below
            # passed on a run that had barely started working. So bound the
            # warm-up itself. A stack that spends most of its planning calls
            # unable to build a graph is not working yet, whatever it does in
            # the calls that are left.
            result.check(
                first_real < len(verts) / 2,
                "warm-up was a minority of the run",
                f"first real graph at call {first_real + 1} of {len(verts)}; "
                f"{100.0 * first_real / len(verts):.0f}% of calls produced "
                f"nothing but the root vertex")

            # What this guards is a stack that stops planning, so it asks two
            # things of the graphs after the warm-up: that they are not mostly
            # degenerate, and that the run does not end degenerate. A single
            # one-vertex graph in the middle is not that - it is one call landing
            # while the robot sits in a pocket the map has not covered yet, and
            # both the UAV and the UGV produce one occasionally in a healthy run.
            # Failing on it would make this check noise, which is worse than not
            # having it. "Nothing happened" still fails, one check above: a run
            # with no real graph at all never reaches here.
            after = verts[first_real:]
            degenerate_after = [v for v in after if v <= 1]
            tail = after[-3:]
            ended_degenerate = len(after) >= 3 and all(v <= 1 for v in tail)
            mostly_degenerate = len(degenerate_after) > len(after) / 4
            result.check(
                not (ended_degenerate or mostly_degenerate),
                "planning did not degenerate after warm-up",
                f"warm-up {first_real} call(s), then {len(after)} graphs "
                f"[{min(after)}..{max(after)}], mean {sum(after)/len(after):.0f}, "
                f"{len(degenerate_after)} degenerate, last three {tail}")

    gains = [float(g) for g in GAIN_RE.findall(planner)]
    result.check(len(gains) >= 1 and all(g > 0 for g in gains),
                 "every best path has positive gain",
                 f"{len(gains)} paths, min gain {min(gains):.0f}" if gains else "none")

    # 3. It kept up. Planning slower than the replan period means the robot is
    #    flying blind between paths.
    totals = [float(t) for t in TOTAL_RE.findall(planner)]
    if totals:
        worst = max(totals)
        mean = sum(totals) / len(totals)
        result.check(worst < 5000.0, "planning latency stayed sane",
                     f"mean {mean:.0f} ms, worst {worst:.0f} ms over {len(totals)} calls")
    else:
        result.check(False, "planning latency measured", "no Time statistics blocks found")

    # 4. The robot actually moved. This is the check that cannot be satisfied by
    #    a planner that produces paths nobody flies.
    distance, points = travelled(poses)
    result.check(len(points) >= 10, "odometry was sampled",
                 f"{len(points)} samples")
    result.check(distance > 5.0, "robot travelled",
                 f"{distance:.1f} m over {len(points)} samples")
    if points:
        span = max(math.dist(points[0], p) for p in points)
        result.check(span > 3.0, "robot left its starting area",
                     f"furthest point {span:.1f} m from start")

        # ... and stayed in the world. Distance alone is satisfied by a robot
        # that fell through the floor: one ugv_niosh run scored 1102 m
        # travelled and 979 m from start while ending at z = -602 m, having
        # dropped through a gap in the tunnel's triangle mesh and accelerated
        # under gravity for the rest of the run. Every other check passed. Two
        # bounds catch it, and both are deliberately far outside anything a
        # ground robot or a multicopter does in these worlds.
        z0 = points[0][2]
        worst_z = max(points, key=lambda p: abs(p[2] - z0))[2]
        result.check(abs(worst_z - z0) < 50.0, "robot stayed in the world",
                     f"z went from {z0:.2f} m to {worst_z:.2f} m")
        jumps = [math.dist(a, b) for a, b in zip(points, points[1:])]
        biggest = max(jumps) if jumps else 0.0
        result.check(biggest < 25.0, "odometry is continuous",
                     f"largest step between two samples {biggest:.1f} m")

    # Why it stopped, when it stopped. Reported rather than asserted on: the
    # assertion is the distance above, and this only says which half of the
    # stack to look at. A follower that has stopped publishing is control logic;
    # one still commanding a robot that does not move is geometry or physics.
    cmd = read(args.out / "cmd_vel.txt")
    if cmd:
        samples = cmd.splitlines()
        commanding = [ln for ln in samples
                      if re.search(r"x:\s*(-?[0-9.eE+-]+)", ln)
                      and abs(float(re.search(r"x:\s*(-?[0-9.eE+-]+)", ln).group(1))) > 1e-3]
        result.note(f"follower published a non-zero forward velocity in "
                    f"{len(commanding)}/{len(samples)} samples"
                    + ("" if commanding else
                       " - it stopped commanding, so look at the follower and "
                       "the control interface, not at the simulation"))

    # 5. The regression fix, observed rather than argued: the three-second wait
    #    in runGlobalPlanner exists so a fresher pose arrives. Under the single
    #    threaded executor nothing could deliver one and this was always 0.
    moves = [float(m) for m in ODOM_MOVE_RE.findall(planner)]
    if moves:
        moved = [m for m in moves if m > 0.01]
        result.check(bool(moved), "odometry advanced during the global wait",
                     f"{len(moved)}/{len(moves)} waits saw movement, max {max(moves):.3f} m")
    else:
        result.note("runGlobalPlanner was never reached, so the wait could not be "
                    "observed. Not a failure - it needs a repositioning cycle - but "
                    "this run is not evidence for that fix either.")

    summary = {
        "scenario": args.scenario,
        "graphs": len(graphs),
        "warmup_calls": first_real if graphs and first_real is not None else None,
        "vertices_mean": (sum(v for v, _ in graphs) / len(graphs)) if graphs else 0,
        "gains": len(gains),
        "planning_ms_mean": (sum(totals) / len(totals)) if totals else None,
        "planning_ms_worst": max(totals) if totals else None,
        "distance_m": distance,
        "odometry_waits_observed": len(moves),
        "failures": result.failures,
    }
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2))

    print()
    if result.failures:
        print(f"FAIL - {len(result.failures)} check(s) failed:")
        for f in result.failures:
            print(f"  - {f}")
        return 1
    print("PASS - the stack explored autonomously and nothing crashed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
