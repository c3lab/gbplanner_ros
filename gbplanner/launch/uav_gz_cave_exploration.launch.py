"""UAV cave exploration: simulator, planner, control interface and RViz.

The ROS 2 equivalent of ROS 1's `roslaunch gbplanner uav_gzc_cave_exploration.launch`,
on Gazebo Harmonic rather than Gazebo Classic - Classic is end of life and has no
Jazzy packages, so only the `gz` path carries over.

    ros2 launch gbplanner uav_gz_cave_exploration.launch.py

That is the whole command: it starts gz with the cave, spawns rmf_owl with its
lidar, brings up the planner and the control interface, and opens RViz. Trigger
exploration with

    ros2 service call /planner_control_interface/std_srvs/automatic_planning \
        std_srvs/srv/Trigger

The default world is the DARPA cave, whose tiles live in the subt_cave_sim
checkout - 3.9 GB with git-lfs, deliberately not vendored here. `make run-sim`
mounts it. Without those assets, pass `world:=cave_box` for the small
primitives-only cave that ships with gbplanner_gz_sim - but pass a spawn pose
with it. The x/y/z below are the DARPA cave's staging area, and in cave_box
(x within +/- 40) that lands the vehicle a few centimetres off the end wall. It
flies from there, with less room than it should have.
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
            "robot_name": robot_name,
            # The spawn pose the ROS 1 cave launch used.
            "x": "40.0",
            "y": "5.0",
            "z": "1.5",
            "headless": LaunchConfiguration("headless"),
            # Without this gz never creates an EGL context and the gpu_lidar
            # raycasts on the CPU: 6.7 Hz against its nominal 10.
            "headless_rendering": "true",
            # Where docker-compose mounts the subt_cave_sim tiles.
            "resource_path": "/opt/subt_cave_sim/models",
            # What the trajectory follower flies. The control interface publishes
            # the planner's path here.
            "path_topic": "/pci_command_path",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            # RViz belongs to the planner stack below, not to the simulator.
            "rviz": "false",
        }.items(),
    )

    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "gbplanner.launch.py"])
        ),
        launch_arguments={
            "config_folder": PathJoinSubstitution(
                [pkg_share, "config", "uav", "gz", "cave_exploration"]),
            # Not ground_truth/odometry_throttled as in ROS 1: that name came from
            # a topic_tools throttle capping a 50 Hz stream at 100 messages per
            # second, which did nothing. gz's OdometryPublisher publishes here.
            "odometry_topic": ["/", robot_name, "/odometry"],
            "pointcloud_topic": ["/", robot_name, "/lidar/points"],
            "command_trajectory_topic": [robot_name, "/command/trajectory"],
            # Absolute here, as in ROS 1: this scenario is single-robot and the
            # goal comes from the one RViz session.
            "local_navigation_goal_topic": "/move_base_simple/goal",
            "rviz": LaunchConfiguration("rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world", default_value="darpa_cave_01",
                description="World in gbplanner_gz_sim/worlds. Use cave_box when "
                            "the subt_cave_sim assets are not available."),
            DeclareLaunchArgument("robot_name", default_value="rmf_owl"),
            DeclareLaunchArgument(
                "headless", default_value="false",
                description="Run gz without its GUI. RViz is separate and "
                            "follows the rviz argument."),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            # Scoped, and this is load-bearing rather than tidy.
            # IncludeLaunchDescription does NOT open a scope: its
            # launch_arguments become SetLaunchConfiguration in the
            # CURRENT scope and stay there. Without the group, the
            # simulation's rviz:=false - correct, since RViz belongs to
            # the planner stack and not to gz - overwrote the shared
            # rviz configuration, and the core include below then read
            # back false and never started RViz at all.
            # scoped only: forwarding stays on, or the parent's own
            # configurations would not reach the includes either.
            GroupAction([simulation], scoped=True),
            GroupAction([core], scoped=True),
        ]
    )
