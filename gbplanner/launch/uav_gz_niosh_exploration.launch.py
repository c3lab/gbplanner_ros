"""UAV exploration of the NIOSH mine: simulator, planner, control interface, RViz.

The ROS 2 equivalent of ROS 1's `roslaunch gbplanner uav_gzc_niosh_exploration.launch`,
on Gazebo Harmonic rather than Gazebo Classic.

    ros2 launch gbplanner uav_gz_niosh_exploration.launch.py

The robot is the RMF-Owl, as in every other UAV scenario here. ROS 1's Classic
scenarios flew rmf_obelix out of RotorS instead; RotorS is a Classic-only
package with its own controller stack and none of it was ported, while rmf_owl
was already a gz model. The planner config is upstream's unchanged, and the two
airframes are close enough in size that RobotParams did not need touching.

The tunnel mesh lives in the subt_cave_sim checkout (223 MB in one DAE),
deliberately not vendored here; `make run-sim uav_niosh` mounts it. Without it,
pass `world:=cave_box`.
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
            "robot_model": "rmf_owl",
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
                [pkg_share, "config", "uav", "gz", "niosh_exploration"]),
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
            DeclareLaunchArgument("robot_name", default_value="rmf_owl"),
            # The spawn pose the ROS 1 launch used.
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.5"),
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
