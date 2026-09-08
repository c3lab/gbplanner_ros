#!/usr/bin/env python3
"""Migrate a gbplanner RViz config to RViz2, keeping our own plugins.

RViz2 ships an official converter at /opt/ros/<distro>/lib/rviz2/rviz1_to_rviz2.py
and it is the authority on class renames, panel classes, the nested Topic/QoS
blocks and the Transformation field RViz2 requires. Use it, do not reimplement it.

What it cannot do is keep plugins it has never heard of: it drops every display
and panel whose class it does not recognise. For these configs that means four,
and only two of them should actually go:

  voxblox_rviz_plugin/VoxbloxMesh   KEEP  - exists in our voxblox fork, same class
                                            name, and it is how the map is shown
  gbplanner_ui/GbPlanner Control    KEEP  - our own panel
  rmf_ui/RMF Control                DROP  - no such package anywhere in the tree
  rectangle_drawer/RectangleDrawTool DROP - likewise

So this wrapper runs the official converter, then re-injects the two survivors at
their original positions, translating VoxbloxMesh's flat ROS 1 Topic/Queue
Size/Unreliable properties into RViz2's nested Topic block the same way the
official converter does for the displays it does know.

Usage (inside a container that has rviz2):
    tools/rviz1_to_rviz2_gbplanner.py IN.rviz OUT.rviz
"""

import argparse
import copy
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

OFFICIAL = "/opt/ros/{distro}/lib/rviz2/rviz1_to_rviz2.py"

# Classes the official converter drops that we want back.
KEEP_DISPLAY = "voxblox_rviz_plugin/VoxbloxMesh"
KEEP_PANEL = "gbplanner_ui/GbPlanner Control"


def to_qos_topic(display):
    """Flat ROS 1 Topic/Queue Size/Unreliable -> RViz2's nested Topic block."""
    out = copy.deepcopy(display)
    topic = out.pop("Topic", "")
    depth = out.pop("Queue Size", 10)
    unreliable = out.pop("Unreliable", False)
    out["Topic"] = {
        "Value": topic,
        "Depth": depth,
        "History Policy": "Keep Last",
        "Reliability Policy": "Best Effort" if unreliable else "Reliable",
        "Durability Policy": "Volatile",
    }
    return out


def collect_kept_displays(displays, path=()):
    """Yield (group-name path, index within that list, migrated display)."""
    for index, display in enumerate(displays or []):
        if not isinstance(display, dict):
            continue
        if display.get("Class") == KEEP_DISPLAY:
            yield path, index, to_qos_topic(display)
        elif "Displays" in display:
            yield from collect_kept_displays(
                display["Displays"], path + (display.get("Name", ""),))


def insert_display(displays, path, index, display):
    """Insert into the group named by `path`, as close to `index` as possible."""
    if path:
        head, rest = path[0], path[1:]
        for entry in displays or []:
            if isinstance(entry, dict) and entry.get("Name") == head and "Displays" in entry:
                insert_display(entry["Displays"], rest, index, display)
                return
        # The group did not survive migration; fall through and append at the top.
    displays.insert(min(index, len(displays)), display)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input")
    parser.add_argument("output")
    parser.add_argument("--distro", default="jazzy")
    args = parser.parse_args()

    official = Path(OFFICIAL.format(distro=args.distro))
    if not official.exists():
        print(f"official converter not found at {official} - run this inside the "
              f"gbplanner-ros2 container", file=sys.stderr)
        return 2

    original = yaml.safe_load(Path(args.input).read_text())

    with tempfile.NamedTemporaryFile(suffix=".rviz", delete=False) as handle:
        migrated_path = handle.name
    result = subprocess.run(
        [sys.executable, str(official), args.input, migrated_path],
        capture_output=True, text=True)
    if result.returncode != 0:
        print(result.stderr, file=sys.stderr)
        return result.returncode

    migrated = yaml.safe_load(Path(migrated_path).read_text())

    # Re-inject the VoxbloxMesh displays the official converter dropped.
    vm = original.get("Visualization Manager", {})
    kept = list(collect_kept_displays(vm.get("Displays", [])))
    target = migrated.setdefault("Visualization Manager", {}).setdefault("Displays", [])
    for path, index, display in kept:
        insert_display(target, path, index, display)

    # Re-inject exactly one control panel. Several ROS 1 configs list it twice,
    # which RViz2 renders as two identical docked panels.
    panels = migrated.setdefault("Panels", [])
    if not any(p.get("Class") == KEEP_PANEL for p in panels if isinstance(p, dict)):
        if any(p.get("Class") == KEEP_PANEL
               for p in original.get("Panels", []) if isinstance(p, dict)):
            panels.append({"Class": KEEP_PANEL, "Name": "GbPlanner Control"})

    # The saved window layout blob refers to panels by name and no longer matches
    # once panels are added or removed. RViz2 rebuilds it on first save.
    migrated.get("Window Geometry", {}).pop("QMainWindow State", None)

    Path(args.output).write_text(yaml.dump(migrated, default_flow_style=False, sort_keys=False))
    print(f"{args.input} -> {args.output}: "
          f"{len(kept)} VoxbloxMesh display(s) re-injected, "
          f"panel {'re-injected' if kept is not None else ''}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
