"""ANYmal exploring the NIOSH mine, planned for as a ground robot.

    ros2 launch gbplanner anymal_gz_niosh_exploration.launch.py

This is the only scenario in the repository where `RobotParams.type` is
`kGroundRobot`. Everything else - including upstream's own UGV demos - plans
ground robots as aerial ones, because `kGroundRobot` routes every sample and
every edge through `Rrg::projectSampleEleMap`, which needs a `grid_map`
"elevation" layer that nothing in gbplanner_ros publishes. Here
`gbplanner_elevation_map` builds that layer from the robot's own point cloud,
which is what makes the mode usable.

What it buys: a sample's height is snapped to the observed terrain rather than
kept at the robot's current altitude, and it is refused outright if a footprint
corner is steeper than `max_inclination` or has never been seen. A ledge is no
longer "free space above a drop", which is what let the Husky drive out of the
tunnel and fall.

Two things this is not. The legs are not simulated - the model rolls on wheels
inside the leg visuals, see its SDF header - so nothing here says anything about
gait. And the planner config is c3lab's real-deployment ANYmal config with the
bounding space opened up to these worlds; it was never tuned for this mine.

The tunnel mesh lives in the subt_cave_sim checkout, deliberately not vendored;
`make run-sim anymal_niosh` mounts it. Without those assets, pass
`world:=cave_box` for the primitives-only world that ships with
gbplanner_gz_sim.
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("gbplanner")
    robot_name = LaunchConfiguration("robot_name")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gbplanner_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "robot_model": "anymal",
            "robot_name": robot_name,
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "headless": LaunchConfiguration("headless"),
            # Without this gz never creates an EGL context and the gpu_lidar
            # raycasts on the CPU.
            "headless_rendering": "true",
            "resource_path": "/opt/subt_cave_sim/models",
            "path_topic": "/pci_command_path",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "rviz": "false",
            # A VLP-16: 16 rings, and 1024 columns rather than the Husky's 2048.
            # The elevation map integrates every return, so the extra columns
            # buy resolution the 0.2 m grid cannot hold.
            "lidar_vertical_samples": "16",
            "lidar_horizontal_samples": "1024",
        }.items(),
    )

    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "gbplanner.launch.py"])
        ),
        launch_arguments={
            "config_folder": PathJoinSubstitution(
                [pkg_share, "config", "anymal", "gz", "niosh_exploration"]),
            "odometry_topic": ["/", robot_name, "/odometry"],
            "pointcloud_topic": ["/", robot_name, "/lidar/points"],
            "command_trajectory_topic": [robot_name, "/command/trajectory"],
            "local_navigation_goal_topic": "/move_base_simple/goal",
            "rviz": LaunchConfiguration("rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world", default_value="niosh_osrf",
                description="World in gbplanner_gz_sim/worlds. Use cave_box when "
                            "the subt_cave_sim assets are not available."),
            DeclareLaunchArgument("robot_name", default_value="anymal"),
            # base_link sits 0.6 m up, so spawning at 0.9 drops it a little onto
            # its wheels rather than through the floor.
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.9"),
            DeclareLaunchArgument(
                "headless", default_value="false",
                description="Run gz without its GUI. RViz is separate and "
                            "follows the rviz argument."),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            # Scoped for the reason spelled out in uav_gz_cave_exploration:
            # IncludeLaunchDescription does not open a scope of its own.
            GroupAction([simulation], scoped=True),
            GroupAction([core], scoped=True),
        ]
    )
