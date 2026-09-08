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
    args = parser.parse_args()

    planner = read(args.out / "planner.log")
    pci = read(args.out / "pci.log")
    sim = read(args.out / "sim.log")
    poses = read(args.out / "poses.txt")
    alive = read(args.out / "alive.txt")

    print("Suite C - autonomous exploration")
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
    if result.failures:
        print("\nnothing ran; the remaining checks would be meaningless")
        return 1

    # 1. Nothing crashed.
    for text, who in ((planner, "planner"), (pci, "pci")):
        for pattern, meaning in FATAL_PATTERNS:
            hits = text.count(pattern)
            result.check(hits == 0, f"{who} free of '{pattern}'",
                         "none" if hits == 0 else f"{hits}x - {meaning}")

    result.check("alive at end: yes" in alive.replace("planner ", "").replace("pci ", ""),
                 "nodes still alive at the end", alive.strip().replace("\n", "; ") or "unknown")

    # 2. It planned, repeatedly. One graph is a first path; the regression this
    #    guards against is a stack that plans once and then stops.
    graphs = [(int(v), int(e)) for v, e in GRAPH_RE.findall(planner)]
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
            after = verts[first_real:]
            degenerate_after = [v for v in after if v <= 1]
            result.check(
                not degenerate_after, "no graph degenerated after warm-up",
                f"warm-up {first_real} call(s), then {len(after)} graphs "
                f"[{min(after)}..{max(after)}], mean {sum(after)/len(after):.0f}")

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
        "graphs": len(graphs),
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
