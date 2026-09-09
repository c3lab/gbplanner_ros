"""UGV exploration of the SubT Urban Circuit: simulator, planner, control interface, RViz.

The ROS 2 equivalent of ROS 1's `roslaunch gbplanner ugv_gzc_urban_exploration.launch`,
on Gazebo Harmonic rather than Gazebo Classic.

    ros2 launch gbplanner ugv_gz_urban_exploration.launch.py

This is the multi-level scenario: the tiles are stacked three storeys deep and
the robot starts on the middle one, so the exploration front repeatedly leaves
the plane the robot is standing on. The config differs from the NIOSH one in
exactly the ways that matter - a global bounding space 100 m tall instead of
40, a local space of +/- 2 m in z rather than +/- 1, select_closest_frontier on,
and allow_sudden_dir_change on.

As in the NIOSH scenario the robot is the MARBLE Husky rather than ROS 1's SMB;
see ugv_gz_niosh_exploration.launch.py for why.

The tile meshes live in the subt_cave_sim checkout, deliberately not vendored
here; `make run-sim ugv_urban` mounts it. There is no primitives-only stand-in
for this world - `world:=cave_box` gives a single-level cave, which is the one
thing this scenario is not.
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
            "robot_model": "marble_husky",
            "robot_name": robot_name,
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "headless": LaunchConfiguration("headless"),
            "headless_rendering": "true",
            "resource_path": "/opt/subt_cave_sim/models",
            "path_topic": "/gbplanner_path",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "rviz": "false",
        }.items(),
    )

    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "gbplanner.launch.py"])
        ),
        launch_arguments={
            "config_folder": PathJoinSubstitution(
                [pkg_share, "config", "ugv", "gz", "urban_exploration"]),
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
                "world", default_value="urban_circuit_01",
                description="World in gbplanner_gz_sim/worlds."),
            DeclareLaunchArgument("robot_name", default_value="marble_husky"),
            # The ROS 1 spawn, raised half a metre: that file put the SMB at
            # z = -8.0 and the Husky is spawned rather than dropped, so it has
            # to start clear of the floor.
            DeclareLaunchArgument("x", default_value="70.0"),
            DeclareLaunchArgument("y", default_value="-100.0"),
            DeclareLaunchArgument("z", default_value="-7.5"),
            DeclareLaunchArgument(
                "headless", default_value="false",
                description="Run gz without its GUI. RViz is separate and "
                            "follows the rviz argument."),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            GroupAction([simulation], scoped=True),
            GroupAction([core], scoped=True),
        ]
    )
