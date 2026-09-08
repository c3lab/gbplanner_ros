#!/usr/bin/env python3
"""Compare the live ROS 1 and ROS 2 gbplanner runs collected by run_suite_a.sh.

The point of this file is the control. Exact equality is unreachable here and
that is not the port's fault: Rrg::reset() re-seeds std::mt19937 from
std::random_device on every service call, rrg.cpp calls an unseeded rand(),
Graph is boost::adjacency_list<setS, listS, ...> so Dijkstra's tie-breaking
depends on where the allocator put the vertices, and global graph expansion runs
against a wall-clock budget. Suite B measured the tie-breaking one directly:
parent_stable_under_heap_shift is 0 on BOTH sides, i.e. one binary already
disagrees with itself after unrelated allocations.

So every observable is judged against how much ROS 1 disagrees with ROS 1. For
each metric this computes the ROS 1 half-range over its own runs and asks
whether the ROS 1 -> ROS 2 shift of the mean exceeds it:

    z = |mean_ros2 - mean_ros1| / max(halfrange_ros1, floor)

z <= 1 means the two ports differ by no more than ROS 1 differs from itself,
which is the only "similar" that means anything for a randomised planner. The
floors exist so a metric whose ROS 1 spread happened to be zero across five runs
does not divide by zero and fail on the last bit of a float.

Exit status is non-zero when any metric exceeds its control and is not listed in
EXPECTED_DEVIATIONS with a reason.

Usage: compare_suite_a.py <out-dir>
"""

import glob
import json
import math
import os
import re
import sys
import textwrap

# ---------------------------------------------------------------------------
# Floors on the control width, in the metric's own units. A floor only ever
# widens a tolerance that measurement noise made implausibly narrow; it never
# narrows one.
# ---------------------------------------------------------------------------
FLOORS = {
    # Vertex and edge counts of the local RRG. Measured ROS 1 half-range: 26.5
    # vertices and 286 edges over 15 samples, so these floors sit just under the
    # observed control and only ever guard against a degenerate zero-width one.
    "graph_n_vertices": 20.0,
    "graph_n_edges": 200.0,
    "log_n_vertices": 20.0,
    "log_n_edges": 200.0,
    # Volumetric gain of the chosen path, ~4e5 with a ROS 1 half-range of ~7.7e4.
    # 5000 is about 1 % of the mean.
    "best_gain": 5000.0,
    # One waypoint and one RRG edge length: the quantisation of each metric.
    # ROS 1's own half-range is 1.0 waypoint and 1.66 m.
    "path_size": 1.0,
    "path_length": 1.0,
    "path_end_dist": 1.0,
    # Milliseconds. Each floor is the same order as its own metric, not larger:
    # ROS 1 half-ranges are 198 (total), 49 (build), 151 (gain), 0.25 (Dijkstra)
    # and 6.0 (evaluate). A floor much above the metric would make its test
    # vacuous rather than tolerant.
    "t_total": 100.0,
    "t_build": 50.0,
    "t_gain": 100.0,
    "t_dijkstra": 1.0,
    "t_eval": 10.0,
    # Seconds of wall clock for the whole service call; ROS 1 half-range 0.50 s.
    "wall_call_s": 0.3,
    # Metres. Both sides station-keep to the millimetre, so the ROS 1 spread is
    # exactly zero; a third of a 0.3 m voxel is the smallest difference that
    # could move a single map cell.
    "odom_x": 0.10, "odom_y": 0.10, "odom_z": 0.10,
    # Derived from the vertex cloud. The density floor is 0.01 vertices/m^3,
    # about 15 % of the ~0.07 both sides sample at.
    "graph_bbox_volume": 200.0,
    "graph_density": 0.01,
    "graph_max_reach": 1.0,
}

# ---------------------------------------------------------------------------
# Differences that are known, understood and deliberate. Each entry is
# (metric name, explanation). Anything not listed fails the run.
# ---------------------------------------------------------------------------
EXPECTED_DEVIATIONS = [
    (r"^(graph_bbox_volume|(graph|log)_n_(vertices|edges))$",
     "the ROS 2 RRG carries ~1.7x the vertices and ~2.4x the edges, over a "
     "1.5x larger bounding volume. Measured "
     "cause, not assumed: the sampling DENSITY is the same on both sides "
     "(graph_density, ~0.07 vertices/m^3, z well inside the control), while the "
     "volume the graph spans is not (graph_bbox_volume, 2144 vs 3154 m^3) and "
     "the ROS 2 graph reaches the 15 m local sampling bound on every one of 15 "
     "samples (graph_max_reach 15.06 +- 0.12) where ROS 1 sometimes stops at "
     "11.6 m. Neither side comes near num_vertices_max = 400, so nothing is "
     "clipped. The RRG therefore samples the same way and simply finds more "
     "known-free space: after the same 60 s of simulated time the gz Harmonic "
     "gpu_lidar has mapped more of the cave than the Gazebo Classic gpu_ray "
     "Ouster. That is a difference between two simulators, which the port did "
     "not choose - and it does not propagate to the output: best_gain, "
     "path_size, path_length, path_end_dist and the path Hausdorff distance are "
     "all inside the ROS1-vs-ROS1 control."),
    (r"^t_(build|gain|total)$",
     "ROS 2 is FASTER, not slower: 107 ms total against 334 ms, while building "
     "a 1.7x larger graph. Two effects push the same way and neither is a port "
     "regression. (1) Toolchain: GCC 9.4/Eigen 3.3.7/PCL 1.10 against GCC "
     "13.3/Eigen 3.4.0/PCL 1.14 - the axes Suite B records as meta keys. (2) "
     "The ROS 1 simulator is rendering its gpu_ray lidar on the CPU through "
     "Xvfb and llvmpipe, on the same cores as the planner, while gz renders on "
     "the GPU. t_dijkstra and t_eval, the two stages that touch neither the map "
     "nor PCL, are inside the control."),
    (r"^best_path_id$",
     "the vertex id of the chosen path. Ids are handed out in sampling order "
     "and the sampler is re-seeded from std::random_device on every call, so "
     "this number carries no geometry - Suite B does not compare vertex ids "
     "either, for the same reason. What the id stands for is compared instead: "
     "path_size, path_length, path_end_dist and the Hausdorff distance below."),
]

PATTERNS = {
    "call_start": re.compile(r"Current Planner Mode: (\d+)"),
    "graph": re.compile(
        r"Formed a graph with \[(\d+)\] vertices and \[(\d+)\] edges"
        r"(?: with \[(\d+)\] loops)?"),
    "gain": re.compile(r"Best path: with gain \[([-\d.eE+]+)\] and ID \[(-?\d+)\]"),
    "bestpath": re.compile(
        r"Best path:\s+size = (\d+), length = ([-\d.eE+]+), time = ([-\d.eE+]+)"),
    "timing": re.compile(
        r"Time statistics[^\n]*\n[^\n]*Build graph\s*:\s*([-\d.eE+]+)[^\n]*\n"
        r"[^\n]*Compute gain\s*:\s*([-\d.eE+]+)[^\n]*\n"
        r"[^\n]*Dijkstra\s*:\s*([-\d.eE+]+)[^\n]*\n"
        r"[^\n]*Evaluate graph\s*:\s*([-\d.eE+]+)[^\n]*\n"
        r"[^\n]*Total\s*:\s*([-\d.eE+]+)"),
}


# --------------------------------------------------------------------------
# Loading
# --------------------------------------------------------------------------
def parse_log(path):
    """Split a planner log into one record per /gbplanner call.

    'Current Planner Mode' is logged unconditionally at the top of
    Gbplanner::plannerServiceCallback, once per call, on both sides, which makes
    it the only reliable delimiter -- everything else in the block is emitted a
    variable number of times.
    """
    if not os.path.exists(path):
        return []
    text = open(path, "r", errors="replace").read()
    starts = [m.start() for m in PATTERNS["call_start"].finditer(text)]
    records = []
    for i, start in enumerate(starts):
        end = starts[i + 1] if i + 1 < len(starts) else len(text)
        block = text[start:end]
        record = {}
        graphs = PATTERNS["graph"].findall(block)
        if graphs:
            # The last "Formed a graph" in a call is the local RRG the best path
            # is chosen from; earlier ones belong to the global graph update.
            vertices, edges, loops = graphs[-1]
            record["log_n_vertices"] = float(vertices)
            record["log_n_edges"] = float(edges)
            if loops:
                record["log_n_loops"] = float(loops)
        gains = PATTERNS["gain"].findall(block)
        if gains:
            record["best_gain"] = float(gains[-1][0])
            record["best_path_id"] = float(gains[-1][1])
        best = PATTERNS["bestpath"].findall(block)
        if best:
            record["log_path_size"] = float(best[-1][0])
            record["log_path_length"] = float(best[-1][1])
            record["log_path_time"] = float(best[-1][2])
        timing = PATTERNS["timing"].findall(block)
        if timing:
            names = ["t_build", "t_gain", "t_dijkstra", "t_eval", "t_total"]
            for name, value in zip(names, timing[-1]):
                record[name] = float(value)
        records.append(record)
    return records


def polyline(path):
    return [(p[0], p[1], p[2]) for p in path]


def path_length(points):
    return sum(math.dist(points[i], points[i + 1]) for i in range(len(points) - 1))


def metrics_of(sample, log_record):
    out = dict(log_record)
    graph = sample.get("graph")
    if graph:
        out["graph_n_vertices"] = float(graph["n_vertices"])
        out["graph_n_edges"] = float(graph["n_edges"])
    if graph and graph["vertices"]:
        # Where the RRG could go, separated from how densely it filled it. A
        # graph that is bigger only because the free space it was sampled in was
        # bigger is not a different planner.
        vertices = graph["vertices"]
        spans = [max(v[i] for v in vertices) - min(v[i] for v in vertices)
                 for i in range(3)]
        out["graph_bbox_volume"] = spans[0] * spans[1] * spans[2]
        out["graph_density"] = len(vertices) / out["graph_bbox_volume"]
        root = sample["odom"]["pos"]
        out["graph_max_reach"] = max(math.dist(v, root) for v in vertices)
    points = polyline(sample["path"])
    out["path_size"] = float(len(points))
    out["path_length"] = path_length(points) if len(points) > 1 else 0.0
    out["path_end_dist"] = math.dist(points[0], points[-1]) if len(points) > 1 else 0.0
    out["wall_call_s"] = sample["wall_call_s"]
    out["status"] = float(sample["status"])
    out["odom_x"], out["odom_y"], out["odom_z"] = sample["odom"]["pos"]
    return out


def load_side(out_dir, side):
    """Return [(run_index, call_index, metrics, sample)] for one side."""
    rows = []
    for json_path in sorted(glob.glob(os.path.join(out_dir, "%s_run*.json" % side))):
        run = json.load(open(json_path))
        log_path = json_path[:-5] + ".log"
        log_records = parse_log(log_path)
        for sample in run["samples"]:
            index = sample["call"]
            record = log_records[index] if index < len(log_records) else {}
            rows.append((run["run"], index, metrics_of(sample, record), sample))
    return rows


# --------------------------------------------------------------------------
# Statistics
# --------------------------------------------------------------------------
def stats(values):
    n = len(values)
    mean = sum(values) / n
    if n > 1:
        sd = math.sqrt(sum((v - mean) ** 2 for v in values) / (n - 1))
    else:
        sd = 0.0
    return {"n": n, "mean": mean, "sd": sd, "min": min(values), "max": max(values),
            "halfrange": (max(values) - min(values)) / 2.0}


def direction(points):
    if len(points) < 2:
        return None
    dx = [points[-1][i] - points[0][i] for i in range(3)]
    norm = math.sqrt(sum(c * c for c in dx))
    return [c / norm for c in dx] if norm > 1e-9 else None


def point_to_segment(p, a, b):
    ab = [b[i] - a[i] for i in range(3)]
    denominator = sum(c * c for c in ab)
    if denominator < 1e-12:
        return math.dist(p, a)
    t = sum((p[i] - a[i]) * ab[i] for i in range(3)) / denominator
    t = max(0.0, min(1.0, t))
    return math.dist(p, [a[i] + t * ab[i] for i in range(3)])


def directed_hausdorff(first, second):
    """Max over vertices of `first` of the distance to the polyline `second`.

    Vertex-to-polyline, not vertex-to-vertex: the two planners place waypoints
    at unrelated arc lengths, so a vertex-to-vertex distance would measure
    sampling density rather than where the path goes.
    """
    worst = 0.0
    for p in first:
        if len(second) == 1:
            best = math.dist(p, second[0])
        else:
            best = min(point_to_segment(p, second[i], second[i + 1])
                       for i in range(len(second) - 1))
        worst = max(worst, best)
    return worst


def hausdorff(first, second):
    return max(directed_hausdorff(first, second), directed_hausdorff(second, first))


def angle_between(u, v):
    dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(u, v))))
    return math.degrees(math.acos(dot))


# --------------------------------------------------------------------------
# Report
# --------------------------------------------------------------------------
def summarise_external(out_dir, side, suffix):
    """Read the CLI / C++ client transcripts and say what THEY saw for x and y."""
    paths = []
    for pattern in suffix if isinstance(suffix, tuple) else (suffix,):
        paths += glob.glob(os.path.join(out_dir, "%s_run*%s" % (side, pattern)))
    paths.sort()
    if not paths:
        return None
    subnormal = zero = other = 0
    samples = []
    for path in paths:
        text = open(path, "r", errors="replace").read()
        for cls in re.findall(r"orientation\.class = (\S+)", text):
            if "S" in cls[:2]:
                subnormal += 1
            elif cls[:2] == "ZZ":
                zero += 1
            else:
                other += 1
        for bits in re.findall(r"orientation\.bits = (\S+) (\S+)", text):
            samples.extend(bits)
        # The CLI clients print values, not bit patterns: rclpy's repr on ROS 2
        # and a YAML block on ROS 1.
        pairs = re.findall(r"Quaternion\(x=([-\d.e+]+), y=([-\d.e+]+)", text)
        pairs += re.findall(r"orientation:\s*\n\s*x: (\S+)\s*\n\s*y: (\S+)", text)
        for value in pairs:
            for component in value:
                magnitude = abs(float(component))
                if magnitude == 0.0:
                    zero += 1
                elif magnitude < 2.2250738585072014e-308:
                    subnormal += 1
                else:
                    other += 1
                samples.append(component)
    if subnormal + zero + other == 0:
        return "no quaternion lines found in %d transcript(s)" % len(paths)
    return ("%d subnormal, %d exactly zero, %d normal x/y over %d transcript(s); "
            "e.g. %s" % (subnormal, zero, other, len(paths), samples[:3]))


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")

    ros1 = load_side(out_dir, "ros1")
    ros2 = load_side(out_dir, "ros2")
    if not ros1 or not ros2:
        print("suite A: need both sides; found %d ROS 1 and %d ROS 2 samples"
              % (len(ros1), len(ros2)))
        return 2

    runs1 = len({r[0] for r in ros1})
    runs2 = len({r[0] for r in ros2})
    print("Suite A -- live ROS 1 vs ROS 2 gbplanner on darpa_cave_01")
    print("=" * 78)
    print("samples: ROS 1 %d over %d runs, ROS 2 %d over %d runs"
          % (len(ros1), runs1, len(ros2), runs2))
    if runs1 < 5:
        print("WARNING: fewer than 5 ROS 1 runs; the control is under-sampled")
    print()

    # Internal consistency: the graph read off /vis/planning_graph must be the
    # same graph the planner logged. If these ever disagree the capture is
    # wrong and nothing below means anything.
    for label, rows in (("ros1", ros1), ("ros2", ros2)):
        mismatch = [(r, c) for r, c, m, _ in rows
                    if "log_n_vertices" in m and "graph_n_vertices" in m
                    and (m["log_n_vertices"] != m["graph_n_vertices"]
                         or m["log_n_edges"] != m["graph_n_edges"])]
        print("%s: marker graph == logged graph on %d/%d samples%s"
              % (label, len(rows) - len(mismatch), len(rows),
                 "" if not mismatch else "  MISMATCH at %s" % mismatch))
    print()

    names = sorted({k for _, _, m, _ in ros1 + ros2 for k in m})
    failures = []

    print("%-18s %26s %26s %8s" % ("metric", "ROS 1 (control)", "ROS 2", "z"))
    print("%-18s %26s %26s %8s"
          % ("", "mean +- halfrange [min,max]", "mean +- halfrange [min,max]", ""))
    print("-" * 82)
    for name in names:
        values1 = [m[name] for _, _, m, _ in ros1 if name in m]
        values2 = [m[name] for _, _, m, _ in ros2 if name in m]
        if not values1 or not values2:
            print("%-18s %26s %26s %8s"
                  % (name,
                     "n=%d" % len(values1) if values1 else "absent",
                     "n=%d" % len(values2) if values2 else "absent", "-"))
            continue
        s1, s2 = stats(values1), stats(values2)
        floor = FLOORS.get(name, 0.0)
        width = max(s1["halfrange"], floor)
        z = abs(s2["mean"] - s1["mean"]) / width if width > 0 else \
            (0.0 if s2["mean"] == s1["mean"] else float("inf"))
        flag = "" if z <= 1.0 else "  <-- outside control"
        print("%-18s %11.3f +-%7.3f [%6.1f,%6.1f] %11.3f +-%7.3f [%6.1f,%6.1f] %7.2f%s"
              % (name, s1["mean"], s1["halfrange"], s1["min"], s1["max"],
                 s2["mean"], s2["halfrange"], s2["min"], s2["max"], z, flag))
        if z > 1.0:
            failures.append((name, s1, s2, z))

    # ---- spatial relationship between the two path families ---------------
    print()
    print("Path geometry")
    print("-" * 82)
    paths1 = [polyline(s["path"]) for _, _, _, s in ros1 if len(s["path"]) > 1]
    paths2 = [polyline(s["path"]) for _, _, _, s in ros2 if len(s["path"]) > 1]
    if paths1 and paths2:
        within1 = [hausdorff(a, b) for i, a in enumerate(paths1)
                   for b in paths1[i + 1:]]
        within2 = [hausdorff(a, b) for i, a in enumerate(paths2)
                   for b in paths2[i + 1:]]
        across = [hausdorff(a, b) for a in paths1 for b in paths2]
        for label, values in (("ROS1 vs ROS1 (control)", within1),
                              ("ROS2 vs ROS2", within2),
                              ("ROS1 vs ROS2", across)):
            if values:
                s = stats(values)
                print("hausdorff %-24s mean %6.2f m  [%.2f, %.2f]"
                      % (label, s["mean"], s["min"], s["max"]))
        directions1 = [d for d in (direction(p) for p in paths1) if d]
        directions2 = [d for d in (direction(p) for p in paths2) if d]
        if directions1 and directions2:
            within = [angle_between(a, b) for i, a in enumerate(directions1)
                      for b in directions1[i + 1:]]
            across_dir = [angle_between(a, b) for a in directions1 for b in directions2]
            if within:
                s = stats(within)
                print("endpoint direction ROS1 vs ROS1  mean %6.1f deg [%.1f, %.1f]"
                      % (s["mean"], s["min"], s["max"]))
            s = stats(across_dir)
            print("endpoint direction ROS1 vs ROS2  mean %6.1f deg [%.1f, %.1f]"
                  % (s["mean"], s["min"], s["max"]))
            worst_within = max(within) if within else 0.0
            if across_dir and max(across_dir) > max(worst_within, 45.0):
                failures.append(("path_direction", {"mean": worst_within},
                                 {"mean": max(across_dir)},
                                 max(across_dir) / max(worst_within, 45.0)))

    # ---- quaternions ------------------------------------------------------
    print()
    print("Response quaternions")
    print("-" * 82)
    # Three readings of the same service per side -- the language client used by
    # the probe, the stock CLI client, and a C++ client. If they disagree the
    # defect is in a client; if they agree it is in the response.
    for label, rows in (("ros1", ros1), ("ros2", ros2)):
        classes = {}
        for _, _, _, sample in rows:
            for cls in sample.get("path_quat_class", []):
                classes[cls] = classes.get(cls, 0) + 1
        total = sum(classes.values())
        dirty = sum(n for cls, n in classes.items() if "S" in cls[:2])
        xy_bits = sorted({b for _, _, _, s in rows
                          for quad in s.get("path_quat_bits", []) for b in quad[:2]})
        print("%s probe client  : %d/%d poses with subnormal x or y   classes %s"
              % (label, dirty, total, classes))
        print("%s probe client  : distinct x/y bit patterns %s"
              % (label, xy_bits[:4] + (["..(%d total).." % len(xy_bits)]
                                       if len(xy_bits) > 4 else [])))
        # The two CLI transcripts are named differently because the two
        # command-line clients are: rosservice call and ros2 service call.
        for kind, suffix in (("cli client    ", ("_servicecall.txt",
                                                 "_rosservice.txt")),
                             ("C++ client    ", ("_cppclient.txt",))):
            found = summarise_external(out_dir, label, suffix)
            if found is not None:
                print("%s %s: %s" % (label, kind, found))

    # ---- verdict ----------------------------------------------------------
    print()
    # Group by explanation so one deviation is not printed five times over.
    unexplained = []
    grouped = {}
    for name, _, _, z in failures:
        why = next((w for pattern, w in EXPECTED_DEVIATIONS
                    if re.match(pattern, name)), None)
        if why:
            grouped.setdefault(why, []).append("%s (z=%.2f)" % (name, z))
        else:
            unexplained.append((name, z))
    for why, metrics in grouped.items():
        print("EXPLAINED %s" % ", ".join(sorted(metrics)))
        for line in textwrap.wrap(why, 76):
            print("    %s" % line)
        print()
    if unexplained:
        print("FAIL: %d metric(s) outside the ROS1-vs-ROS1 control and unexplained:"
              % len(unexplained))
        for name, z in unexplained:
            print("  %-18s z=%.2f" % (name, z))
        return 1
    print("PASS: every observable is either inside ROS 1's own run-to-run spread "
          "or listed above with the measurement that explains it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
