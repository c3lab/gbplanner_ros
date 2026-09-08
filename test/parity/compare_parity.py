#!/usr/bin/env python3
"""Compare the two result files produced by the Suite B parity binaries.

Both sides emit `key = value` lines; this walks the union of the keys and
reports, for every one that disagrees, what the two sides said and by how much.
It exits non-zero unless every difference is covered by an entry in
EXPECTED_DEVIATIONS, so an unexplained divergence can never be mistaken for a
pass.

Usage: compare_parity.py <ros1.txt> <ros2.txt> [--verbose]
"""

import re
import sys

# ---------------------------------------------------------------------------
# Tolerances.
#
# The default is strict: floating-point results must round-trip identically
# unless a rule below says otherwise, and every relaxation carries the reason it
# exists. Patterns are matched in order; the first match wins.
# ---------------------------------------------------------------------------
TOLERANCES = [
    # getPointDistance is a trilinear interpolation over voxels stored as
    # float. Two or three ULPs of float (eps = 1.2e-7) is the floor for that
    # computation; the observed spread is 5e-7 relative on 6 of 250 probes, and
    # every query built on top of it - getVoxelStatus, getBoxStatus,
    # getPathStatus, getRayStatus - still agrees exactly, so nothing downstream
    # of this number is sensitive to it.
    (r"^map\.(point_distance|point_gradient)$", 1e-6),
    # Sums over tens of thousands of float-precision TSDF voxels. The summation
    # order is fixed by the sorted walk, so the only slack needed is for the
    # last bits of a long accumulation of floats widened to double.
    (r"^map\.(distance_sum|abs_distance_sum|weight_sum)$", 1e-12),
    (r"^map\.local\.(occupied|free)_sum$", 1e-12),
    (r"^map\.local_cloud(_z)?\.checksum$", 1e-12),
    (r"^map\.scan\..*\.voxel_log_centre_sum$", 1e-12),
    # Frustum endpoint coordinate sums: ~400 terms of trigonometric output.
    (r"^sensor\..*\.pose\[\d+\]\.sum$", 1e-13),
    # Everything else, including every individual coordinate and distance.
    (r".*", 0.0),
]

# ---------------------------------------------------------------------------
# Differences that are known, understood and deliberate. Anything not listed
# here fails the run. Each entry is (key regex, explanation).
# ---------------------------------------------------------------------------
EXPECTED_DEVIATIONS = [
    (r"^meta\.side$", "the side label - the only key that must differ"),
    (r"^meta\.(compiler|libstdcxx|boost_version|eigen_version|pcl_version)$",
     "recorded on purpose: these are the toolchain axes the port moves along "
     "(GCC 9.4/13.3, libstdc++ 9/13, Boost 1.71/1.83, Eigen 3.3.7/3.4.0, "
     "PCL 1.10/1.14). They are evidence, not observables."),
    (r"^graph\.tied\.(parent|leaf_ids|leaf_count)$",
     "Dijkstra tie-breaking on a graph where many shortest paths have exactly "
     "equal cost. Graph is boost::adjacency_list<setS, listS, ...>, so a vertex "
     "descriptor is a pointer into a std::list and the out-edge set is ordered "
     "by that pointer - which of the tied paths wins depends on where the "
     "allocator put the vertices. graph.tied.parent_stable_under_heap_shift "
     "reports 0 on BOTH sides, i.e. the same binary already disagrees with "
     "itself when unrelated allocations shift the heap. The numbers the planner "
     "actually consumes are stable and do agree: graph.tied.distance is "
     "identical, broken_path_edges is 0, worst_path_length_residual is 0, and "
     "graph.unique.parent (a graph with no ties) matches exactly."),
]


def parse(path):
    values = {}
    line_re = re.compile(r"^([A-Za-z0-9_.\[\]]+) = (.*)$")
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for lineno, raw in enumerate(handle, 1):
            raw = raw.rstrip("\n")
            if not raw:
                continue
            match = line_re.match(raw)
            if not match:
                raise SystemExit(
                    "%s:%d: not a result line: %r" % (path, lineno, raw[:120]))
            key, value = match.group(1), match.group(2)
            if key in values:
                raise SystemExit("%s:%d: duplicate key %s" % (path, lineno, key))
            values[key] = value
    return values


def tolerance_for(key):
    for pattern, tol in TOLERANCES:
        if re.match(pattern, key):
            return tol
    return 0.0


def explanation_for(key):
    for pattern, why in EXPECTED_DEVIATIONS:
        if re.match(pattern, key):
            return why
    return None


def classify(value):
    if value.startswith('"'):
        return "str"
    if value.startswith("["):
        return "arr"
    return "num"


def num_diff(a, b, tol):
    """Return None when equal within tol, else a human-readable delta."""
    if a == b:
        return None
    try:
        fa, fb = float(a), float(b)
    except ValueError:
        return "%s vs %s" % (a, b)
    if fa != fa and fb != fb:  # both NaN
        return None
    if fa == fb:
        return None
    delta = abs(fa - fb)
    scale = max(abs(fa), abs(fb))
    rel = delta / scale if scale else delta
    if tol and rel <= tol:
        return None
    return "%r vs %r (abs %.3g, rel %.3g)" % (fa, fb, delta, rel)


def arr_diff(a, b, tol):
    ta = a.strip("[]").split()
    tb = b.strip("[]").split()
    if len(ta) != len(tb):
        return "length %d vs %d" % (len(ta), len(tb))
    worst = None
    first = None
    count = 0
    for i, (x, y) in enumerate(zip(ta, tb)):
        d = num_diff(x, y, tol)
        if d is None:
            continue
        count += 1
        if first is None:
            first = (i, d)
        try:
            rel = abs(float(x) - float(y)) / max(abs(float(x)), abs(float(y)), 1e-300)
        except ValueError:
            rel = float("inf")
        if worst is None or rel > worst[1]:
            worst = ((i, d), rel)
    if not count:
        return None
    return "%d/%d elements differ; first at [%d]: %s; worst at [%d]: %s" % (
        count, len(ta), first[0], first[1], worst[0][0], worst[0][1])


def str_diff(a, b):
    if a == b:
        return None
    sa, sb = a.strip('"'), b.strip('"')
    if len(sa) != len(sb):
        return "length %d vs %d" % (len(sa), len(sb))
    positions = [i for i, (x, y) in enumerate(zip(sa, sb)) if x != y]
    head = ", ".join(
        "[%d] %r vs %r" % (i, sa[i], sb[i]) for i in positions[:8])
    return "%d/%d positions differ: %s%s" % (
        len(positions), len(sa), head, " ..." if len(positions) > 8 else "")


def main(argv):
    if len(argv) < 3:
        raise SystemExit(__doc__)
    path_a, path_b = argv[1], argv[2]
    verbose = "--verbose" in argv[3:]

    a = parse(path_a)
    b = parse(path_b)

    only_a = sorted(set(a) - set(b))
    only_b = sorted(set(b) - set(a))
    shared = sorted(set(a) & set(b))

    unexplained = []
    explained = []
    byte_identical = 0
    within_tolerance = []

    for key in only_a:
        unexplained.append((key, "present only in %s" % path_a))
    for key in only_b:
        unexplained.append((key, "present only in %s" % path_b))

    for key in shared:
        va, vb = a[key], b[key]
        if va == vb:
            byte_identical += 1
            continue
        kind = classify(va)
        if classify(vb) != kind:
            detail = "value kind %s vs %s" % (kind, classify(vb))
        elif kind == "str":
            detail = str_diff(va, vb)
        elif kind == "arr":
            detail = arr_diff(va, vb, tolerance_for(key))
        else:
            detail = num_diff(va, vb, tolerance_for(key))
        if detail is None:
            within_tolerance.append((key, tolerance_for(key)))
            continue
        why = explanation_for(key)
        if why:
            explained.append((key, detail, why))
        else:
            unexplained.append((key, detail))

    print("keys: %d in %s, %d in %s, %d shared" %
          (len(a), path_a, len(b), path_b, len(shared)))
    print("byte-identical:   %d" % byte_identical)
    print("within tolerance: %d%s" %
          (len(within_tolerance),
           (" (" + ", ".join(k for k, _ in within_tolerance) + ")")
           if within_tolerance else ""))
    print("explained:        %d" % len(explained))
    print("unexplained:      %d" % len(unexplained))

    if explained:
        print("\nexplained differences (%d):" % len(explained))
        for key, detail, why in explained:
            print("  %s\n      %s\n      reason: %s" % (key, detail, why))

    if unexplained:
        print("\nUNEXPLAINED DIFFERENCES (%d):" % len(unexplained))
        for key, detail in unexplained:
            print("  %s\n      %s" % (key, detail))
        print("\nPARITY FAILED")
        return 1

    if verbose:
        for key in shared:
            print("  ok %s" % key)
    print("\nPARITY OK - every shared key agrees within tolerance")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
