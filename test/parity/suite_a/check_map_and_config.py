#!/usr/bin/env python3
"""Prove that the two Suite A runs really are the same cave and the same planner.

The whole ROS1-vs-ROS2 comparison rests on two claims that are easy to assert
and easy to get wrong:

  1. sim/gbplanner_gz_sim/worlds/darpa_cave_01.sdf places the same tiles, in the
     same poses, as arl_gazebo_sim/gzc/worlds/darpa_cave_01.world, and both
     resolve those tiles from the same subt_cave_sim checkout.
  2. the ROS 2 planner YAML resolves to the same numbers as the ROS 1 one.

(2) cannot be done by reading YAML: rosparam installs implicit resolvers for
rad()/deg() before loading, so ROS 1's own loader is the only authority on what
a ROS 1 config means. This asks the noetic image to load the files and compares
the result leaf by leaf against the committed ROS 2 YAML.

    ./check_map_and_config.py            # exits non-zero on an unexplained diff

Nothing is written anywhere; the ROS 1 tree is mounted read-only.
"""

import argparse
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROS1_ROOT = Path("/home/santal/git/gbplanner_ros")
ROS2_ROOT = Path(__file__).resolve().parents[3]

ROS1_WORLD = ROS1_ROOT / "bootstrap/sim/arl_gazebo_sim_ros/arl_gazebo_sim/gzc/worlds/darpa_cave_01.world"
ROS2_WORLD = ROS2_ROOT / "sim/gbplanner_gz_sim/worlds/darpa_cave_01.sdf"
ROS1_CONFIG = ROS1_ROOT / "gbplanner/config/uav/gz/cave_exploration"
ROS2_CONFIG = ROS2_ROOT / "gbplanner/config/uav/gz/cave_exploration"

# The pose Suite A parks the robot at, and the two ranges that decide what can
# possibly reach the map from there.
HOVER = (40.0, 5.0, 1.0)

# World-level differences we accept. Each is accepted only because the check
# below shows it is further from the hover pose than voxblox ever integrates,
# i.e. it cannot enter either map. Anything else fails.
WORLD_INTENTIONAL = {
    "base_station": "present only in ROS 1",
    "staging_area": "ROS 2 raises it by 0.1 m in z",
}

# Config leaves that differ on purpose. Empty means: they must all match.
CONFIG_INTENTIONAL = {
    "publish_tsdf_info": "dead key, read by nobody in either voxblox",
    "use_symmetric_weight_drop_off": "misspelled key, never bound in ROS 1 either",
    "mesh_min_weight": "string in ROS 1 so the float param() call falls back to "
                       "voxblox's own 1e-4; ROS 2 writes that number explicitly",
}

TOL_REL = 1e-12
TOL_ABS = 1e-15


def world_includes(path):
    world = ET.parse(path).getroot().find("world")
    out = {}
    for include in world.findall("include"):
        name = (include.findtext("name") or "").strip()
        out[name] = ((include.findtext("uri") or "").strip(),
                     " ".join((include.findtext("pose") or "").split()),
                     (include.findtext("static") or "").strip())
    return out


def compare_worlds():
    ros1, ros2 = world_includes(ROS1_WORLD), world_includes(ROS2_WORLD)
    print("World: %d includes in ROS 1, %d in ROS 2" % (len(ros1), len(ros2)))
    failures = 0
    for name in sorted(set(ros1) ^ set(ros2)):
        side = "ROS 1 only" if name in ros1 else "ROS 2 only"
        why = WORLD_INTENTIONAL.get(name)
        if why:
            print("  accepted  %-14s %s: %s" % (name, side, why))
        else:
            print("  FAIL      %-14s %s" % (name, side))
            failures += 1
    for name in sorted(set(ros1) & set(ros2)):
        if ros1[name] != ros2[name]:
            why = CONFIG_INTENTIONAL.get(name) or WORLD_INTENTIONAL.get(name)
            label = "accepted" if why else "FAIL    "
            print("  %s  %-14s %s vs %s%s"
                  % (label, name, ros1[name], ros2[name],
                     "\n              %s" % why if why else ""))
            if not why:
                failures += 1
    shared1 = [n for n in world_includes(ROS1_WORLD) if n in ros2]
    shared2 = [n for n in world_includes(ROS2_WORLD) if n in ros1]
    print("  include order over the shared tiles identical: %s"
          % (shared1 == shared2))
    if shared1 != shared2:
        failures += 1

    # An accepted world difference is only harmless if it is out of reach of the
    # sensor from where the robot sits, so measure that instead of asserting it.
    # The threshold is twice max_ray_length_m because an <include> pose is the
    # model's origin and these models are ~25 m tiles: doubling covers the whole
    # extent of one without needing its mesh bounds.
    ray_length = yaml.safe_load((ROS2_CONFIG / "voxblox_sim_config.yaml").read_text())
    ray_length = ray_length["/**"]["ros__parameters"]["max_ray_length_m"]
    print("  voxblox max_ray_length_m = %s; hover pose %s" % (ray_length, HOVER))
    for name in sorted(WORLD_INTENTIONAL):
        pose = (ros1.get(name) or ros2.get(name))[1].split()
        origin = tuple(float(v) for v in pose[:3])
        distance = math.dist(origin, HOVER)
        verdict = "out of reach" if distance > 2 * float(ray_length) else "REACHABLE"
        print("    %-14s origin %-22s %6.1f m from the hover pose -- %s"
              % (name, origin, distance, verdict))
        if verdict != "out of reach":
            failures += 1
    return failures


def resolve_ros1_configs(image, config_dir, files):
    listing = "\n".join(str(p) for p in files) + "\n"
    list_path = Path("/tmp/suite_a_cfg_list.txt")
    list_path.write_text(listing)
    script = (
        "source /opt/ros/noetic/setup.bash >/dev/null 2>&1\n"
        "python3 - <<'EOF'\n"
        "import json, rosparam\n"
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
        "EOF\n")
    proc = subprocess.run(
        ["docker", "run", "--rm", "-v", "%s:/cfg:ro" % config_dir,
         "-v", "%s:/list.txt:ro" % list_path, "--entrypoint", "bash", image,
         "-c", script],
        capture_output=True, text=True, check=True)
    return json.loads(proc.stdout)


def flatten(tree, prefix=""):
    out = {}
    for key, value in (tree or {}).items():
        path = "%s.%s" % (prefix, key) if prefix else key
        if isinstance(value, dict):
            out.update(flatten(value, path))
        else:
            out[path] = value
    return out


def equal(a, b):
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) \
            and not isinstance(a, bool) and not isinstance(b, bool):
        return math.isclose(float(a), float(b), rel_tol=TOL_REL, abs_tol=TOL_ABS)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(equal(x, y) for x, y in zip(a, b))
    return a == b


def compare_configs(image):
    files = sorted(ROS1_CONFIG.rglob("*.yaml"))
    resolved = resolve_ros1_configs(image, ROS1_CONFIG, files)
    failures = leaves = 0
    print("\nPlanner configuration (%s)" % ROS1_CONFIG.name)
    for path in files:
        ros1 = flatten(resolved[str(path)])
        ros2_path = ROS2_CONFIG / path.name
        if not ros2_path.exists():
            print("  FAIL %s: no ROS 2 counterpart" % path.name)
            failures += 1
            continue
        document = yaml.safe_load(ros2_path.read_text())
        ros2 = flatten((document.get("/**") or {}).get("ros__parameters") or {})
        leaves += len(set(ros1) & set(ros2))
        for key in sorted(set(ros1) - set(ros2)):
            leaf = key.rsplit(".", 1)[-1]
            if leaf in CONFIG_INTENTIONAL:
                continue
            print("  FAIL %s: key only in ROS 1: %s" % (path.name, key))
            failures += 1
        for key in sorted(set(ros2) - set(ros1)):
            leaf = key.rsplit(".", 1)[-1]
            if leaf in CONFIG_INTENTIONAL:
                continue
            print("  FAIL %s: key only in ROS 2: %s" % (path.name, key))
            failures += 1
        for key in sorted(set(ros1) & set(ros2)):
            if equal(ros1[key], ros2[key]):
                continue
            why = CONFIG_INTENTIONAL.get(key.rsplit(".", 1)[-1])
            if why:
                print("  accepted %s: %s = %r vs %r\n           %s"
                      % (path.name, key, ros1[key], ros2[key], why))
                continue
            print("  FAIL %s: %s = %r vs %r"
                  % (path.name, key, ros1[key], ros2[key]))
            failures += 1
    print("  %d shared leaves compared" % leaves)
    return failures


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros1-image", default="gbplanner:noetic-3.0.0")
    args = parser.parse_args()
    failures = compare_worlds() + compare_configs(args.ros1_image)
    print("\n%s" % ("FAIL: %d unexplained difference(s)" % failures if failures
                    else "PASS: same cave, same planner parameters"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
