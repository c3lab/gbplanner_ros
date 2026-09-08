#!/usr/bin/env python3
"""Prove the ROS 2 parameter YAMLs resolve to the same values as the ROS 1 ones.

The ROS 1 configs are not plain YAML: `rosparam` registers implicit resolvers for
`rad(...)` / `deg(...)` before loading, so the only trustworthy reference for what
a config *means* is ROS 1's own loader. This test asks the existing noetic image
to resolve every original config, converts the same files with
`tools/rosparam2ros2.py`, and compares the two leaf-by-leaf.

Nothing is installed on the host and neither repo is modified: the ROS 1 tree and
its image are mounted read-only.

    test/parity/check_config_parity.py \
        --ros1-config-dir /home/santal/git/gbplanner_ros/gbplanner/config \
        --ros1-image gbplanner:noetic-3.0.0

Exit status is 0 only when every leaf matches or is a declared intentional
deviation (see INTENTIONAL below).
"""

import argparse
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
CONVERTER = REPO / "tools" / "rosparam2ros2.py"

# Deviations we introduce on purpose. Anything else is a failure.
INTENTIONAL = {
    # Dropped: no such parameter is read by voxblox on either ROS 1 or ROS 2.
    "publish_tsdf_info": "dead key, read by nobody in either voxblox",
    # Dropped: the real parameter is `use_symmetric_weight_dropoff` (no underscore
    # before "off"). The YAML key has always been a typo, so it never bound.
    "use_symmetric_weight_drop_off": "misspelled key, never bound in ROS 1 either",
    # ROS 1 loads `1e-4` as the STRING '1e-4' (YAML 1.1 needs 1.0e-4 for a float),
    # so voxblox's float param() call type-mismatches and falls back to its own
    # default, mesh_integrator.h:51 `float min_weight = 1e-4`. The ROS 2 config
    # writes 0.0001 explicitly, which is the same effective value.
    "mesh_min_weight": "string in ROS 1, ignored, C++ default is the same number",
}

TOL_REL = 1e-12
TOL_ABS = 1e-15


def flatten(tree, prefix=""):
    out = {}
    for key, value in tree.items():
        path = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict):
            out.update(flatten(value, path))
        else:
            out[path] = value
    return out


def numeric(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def same(lhs, rhs):
    if isinstance(lhs, list) and isinstance(rhs, list):
        return len(lhs) == len(rhs) and all(same(a, b) for a, b in zip(lhs, rhs))
    if numeric(lhs) and numeric(rhs):
        return math.isclose(float(lhs), float(rhs), rel_tol=TOL_REL, abs_tol=TOL_ABS)
    return lhs == rhs


def resolve_with_ros1(image, config_dir, files):
    """Return {path: resolved dict} as ROS 1's own rosparam loader sees them."""
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as handle:
        handle.write("\n".join(str(p) for p in files))
        list_path = handle.name

    script = (
        "source /opt/ros/noetic/setup.bash >/dev/null 2>&1\n"
        "python3 - <<'EOF'\n"
        "import rosparam, json\n"
        "out = {}\n"
        "for line in open('/list.txt'):\n"
        "    p = line.strip()\n"
        "    if not p:\n"
        "        continue\n"
        f"    cp = p.replace({str(config_dir)!r}, '/cfg')\n"
        "    try:\n"
        "        out[p] = rosparam.load_file(cp)[0][0]\n"
        "    except Exception as exc:\n"
        "        out[p] = {'__error__': str(exc)}\n"
        "print(json.dumps(out))\n"
        "EOF\n"
    )
    proc = subprocess.run(
        ["docker", "run", "--rm",
         "-v", f"{config_dir}:/cfg:ro",
         "-v", f"{list_path}:/list.txt:ro",
         "--entrypoint", "bash", image, "-c", script],
        capture_output=True, text=True, check=True)
    return json.loads(proc.stdout)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ros1-config-dir", required=True, type=Path)
    parser.add_argument("--ros1-image", default="gbplanner:noetic-3.0.0")
    args = parser.parse_args()

    config_dir = args.ros1_config_dir.resolve()
    files = sorted(config_dir.rglob("*.yaml"))
    if not files:
        print(f"no YAML under {config_dir}", file=sys.stderr)
        return 2

    ros1 = resolve_with_ros1(args.ros1_image, config_dir, files)

    workdir = Path(tempfile.mkdtemp(prefix="cfgparity-"))
    leaves = failures = 0
    accepted = []

    for path in files:
        original = ros1.get(str(path), {})
        if "__error__" in original:
            print(f"FAIL {path}: ROS 1 loader error: {original['__error__']}")
            failures += 1
            continue

        converted = workdir / str(path.relative_to(config_dir)).replace("/", "__")
        subprocess.run([sys.executable, str(CONVERTER), str(path), str(converted)],
                       check=True, capture_output=True)
        loaded = yaml.safe_load(converted.read_text())["/**"]["ros__parameters"] or {}

        lhs, rhs = flatten(original), flatten(loaded)
        leaves += len(set(lhs) & set(rhs))

        for key in sorted(set(lhs) - set(rhs)):
            leaf = key.rsplit(".", 1)[-1]
            if leaf in INTENTIONAL:
                accepted.append((path.name, key, INTENTIONAL[leaf]))
            else:
                print(f"FAIL {path}: key dropped in ROS 2: {key}")
                failures += 1
        for key in sorted(set(rhs) - set(lhs)):
            print(f"FAIL {path}: key invented in ROS 2: {key}")
            failures += 1
        for key in sorted(set(lhs) & set(rhs)):
            if same(lhs[key], rhs[key]):
                continue
            leaf = key.rsplit(".", 1)[-1]
            if leaf in INTENTIONAL:
                accepted.append((path.name, key, INTENTIONAL[leaf]))
            else:
                print(f"FAIL {path}: {key}: ros1={lhs[key]!r} ros2={rhs[key]!r}")
                failures += 1

    print(f"\nfiles compared      : {len(files)}")
    print(f"leaf values compared: {leaves}")
    print(f"intentional devs    : {len(accepted)}")
    for name, key, why in accepted:
        print(f"    {name}: {key} — {why}")
    print(f"failures            : {failures}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
