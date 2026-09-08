#!/usr/bin/env python3
"""Convert a ROS 1 gbplanner rosparam YAML into a ROS 2 parameter YAML.

Three things break when a ROS 1 config is handed to rcl_yaml_param_parser:

1. `rad(...)` / `deg(...)` scalars. These are not YAML — they are implicit
   resolvers that ROS 1's `rosparam` module registers on the YAML loader
   (`/opt/ros/noetic/lib/python3/dist-packages/rosparam/__init__.py:642-646`).
   `rad(EXPR)` evaluates EXPR as Python with `pi` bound to `math.pi` and applies
   NO unit conversion (:104-115); `deg(X)` multiplies by pi/180 (:117-129).
   ROS 2 has no equivalent, so the values are expanded here, at conversion time.

2. Integer literals bound to C++ `double` parameters. ROS 1's XmlRpc silently
   promoted them; `rclcpp` throws InvalidParameterTypeException instead. Only
   keys listed in DOUBLE_KEYS are promoted — blanket promotion would corrupt the
   genuinely integral ones (voxels_per_side, queue sizes, iteration counts).

3. Scientific notation without a decimal point (`1e-4`). rcl_yaml_param_parser
   does not accept it; it has to be written out.

The expanded value carries the original expression in a trailing comment so a
reviewer can diff the ROS 2 config against the ROS 1 one by eye.

Usage:
    tools/rosparam2ros2.py IN.yaml OUT.yaml [--node-name '/**']
"""

import argparse
import math
import re
import sys

# Keys whose C++ binding is a double/float but which are written as bare
# integers in at least one shipped config. Derived from the ROS 1 sources:
# planner_common/src/params.cpp, map_manager/src/voxblox/voxblox_common_impl.cpp
# and voxblox's own ros_params.h.
DOUBLE_KEYS = {
    # planner_common / gbplanner
    "max_range",
    "num_vertices_max",
    "num_edges_max",
    "num_loops_cutoff",
    "num_loops_max",
    "augment_free_voxels_time",
    "time_budget_limit",
    # voxblox
    "max_weight",
    "update_mesh_every_n_sec",
    "mesh_min_weight",
    "min_time_between_msgs_sec",
    "timestamp_tolerance_sec",
    "slice_level",
    "traversability_radius",
    "clear_sphere_radius",
    "esdf_max_distance_m",
    "max_ray_length_m",
    "min_ray_length_m",
    "truncation_distance",
    "tsdf_voxel_size",
    "clearing_ray_weight_factor",
    "occupancy_distance_voxelsize_factor",
    "tsdf_surface_distance_threshold_factor",
    "occupancy_min_distance_voxel_size_factor",
}

# Keys that exist in the ROS 1 YAMLs but are read by nobody on either side.
# Left in place they would become undeclared-parameter warnings at best.
DEAD_KEYS = {
    "publish_tsdf_info",            # no such param in ROS 1 or ROS 2 voxblox
    "use_symmetric_weight_drop_off",  # typo; the real key is ..._dropoff
}

ANGLE_RE = re.compile(r"\b(rad|deg)\(([^()]*(?:\([^()]*\)[^()]*)*)\)")


def eval_angle(kind: str, expr: str) -> float:
    """Replicate rosparam's two angle constructors exactly."""
    if kind == "rad":
        # construct_angle_radians: textual pi -> math.pi, then eval. No conversion.
        return float(eval(expr.replace("pi", "math.pi"), {"math": math, "__builtins__": {}}))
    # construct_angle_degrees: plain float, then degrees -> radians.
    return float(expr) * math.pi / 180.0


def fmt(value: float) -> str:
    """Shortest representation that round-trips, and always looks like a float."""
    text = repr(value)
    if "e" in text or "E" in text:
        text = f"{value:.17g}"
        if "e" in text or "E" in text:
            # rcl_yaml_param_parser rejects exponent notation.
            text = f"{value:.20f}".rstrip("0")
            if text.endswith("."):
                text += "0"
    if "." not in text and "inf" not in text and "nan" not in text:
        text += ".0"
    return text


def convert_line(line: str) -> tuple[str, bool]:
    """Return (converted_line, dropped)."""
    body, _, comment = line.partition("#")
    stripped = body.strip()
    if not stripped:
        return line, False

    key = stripped.split(":", 1)[0].strip().lstrip("- ")
    if key in DEAD_KEYS:
        return f"# dropped (read by nobody in ROS 1 or ROS 2): {line.rstrip()}\n", True

    original = body
    angles = ANGLE_RE.findall(body)
    if angles:
        body = ANGLE_RE.sub(lambda m: fmt(eval_angle(m.group(1), m.group(2))), body)

    if key in DOUBLE_KEYS:
        # Promote a bare integer scalar (not a list, not already a float).
        body = re.sub(
            r"(:\s*)([+-]?\d+)(\s*)$",
            lambda m: f"{m.group(1)}{m.group(2)}.0{m.group(3)}",
            body.rstrip("\n"),
        ) + ("\n" if body.endswith("\n") else "")
        # 1e-4 and friends.
        body = re.sub(
            r"(:\s*)([+-]?(?:\d+\.?\d*|\.\d+)[eE][+-]?\d+)(\s*)$",
            lambda m: f"{m.group(1)}{fmt(float(m.group(2)))}{m.group(3)}",
            body.rstrip("\n"),
        ) + ("\n" if body.endswith("\n") else "")

    out = body
    if comment:
        out = out.rstrip("\n") + "#" + comment
    if angles and "# was" not in out:
        out = out.rstrip("\n") + f"  # was {original.split(':', 1)[1].strip()}\n"
    if not out.endswith("\n"):
        out += "\n"
    return out, False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--node-name", default="/**",
                    help="parameter-namespace key to nest under (default: /**)")
    args = ap.parse_args()

    with open(args.input) as handle:
        lines = handle.readlines()

    converted, dropped = [], 0
    for line in lines:
        new, was_dropped = convert_line(line)
        dropped += was_dropped
        converted.append(new)

    with open(args.output, "w") as handle:
        handle.write(f"# Generated from {args.input} by tools/rosparam2ros2.py.\n")
        handle.write("# rad()/deg() expressions are expanded; the originals are kept in comments.\n")
        handle.write(f"{args.node_name}:\n  ros__parameters:\n")
        for line in converted:
            handle.write(line if line.startswith("#") else "    " + line)

    angle_count = sum(len(ANGLE_RE.findall(line)) for line in lines)
    print(f"{args.input} -> {args.output}: {angle_count} angle expressions expanded, "
          f"{dropped} dead keys dropped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
