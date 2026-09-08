"""UAV cave exploration in Gazebo (gz), the direct port of
launch/uav/gz/uav_gz_cave_exploration.launch.

Picks the cave_exploration config folder and the rmf_owl topic names, then
includes the core stack. It does NOT start the simulator -- see the include
point below.

    ros2 launch gbplanner uav_gz_cave_exploration.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("gbplanner")
    robot_name = LaunchConfiguration("robot_name")

    # ------------------------------------------------------------------
    # SIMULATION INCLUDE POINT -- deliberately not wired up.
    #
    # The ROS 1 file included
    #     $(find arl_gazebo_sim)/gz/uav_sim/launch/uav_gz_cave_exploration.launch
    # which spawned gz, the rmf_owl model and its controllers. The ROS 2
    # replacement is meant to be sim/gbplanner_gz_sim (docker-compose already
    # bind-mounts it as gbplanner3_ws/src/sim/gbplanner_gz_sim); that directory
    # is still empty, so there is no launch file to include and no argument
    # list to honour. Inventing one here would only have to be rewritten.
    #
    # When the package lands, add -- before the include below:
    #
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(PathJoinSubstitution([
    #             FindPackageShare("gbplanner_gz_sim"),
    #             "launch", "<the cave world file>.launch.py"])),
    #         launch_arguments={"world_name": world_name,
    #                           "robot_name": robot_name}.items(),
    #     )
    #
    # `world_name` is declared below purely so it is already there to forward.
    # Until then, start the simulation separately and then this file.
    # ------------------------------------------------------------------

    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "gbplanner.launch.py"])
        ),
        launch_arguments={
            "config_folder": PathJoinSubstitution(
                [pkg_share, "config", "uav", "gz", "cave_exploration"]),
            "odometry_topic": ["/", robot_name, "/ground_truth/odometry_throttled"],
            "pointcloud_topic": ["/", robot_name, "/lidar/points"],
            "command_trajectory_topic": [robot_name, "/command/trajectory"],
            # Absolute here, as in ROS 1: this scenario is single-robot and the
            # goal comes from the one RViz session.
            "local_navigation_goal_topic": "/move_base_simple/goal",
            "rviz": LaunchConfiguration("rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "launch_prefix": LaunchConfiguration("launch_prefix"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_name",
                default_value="cosmos",
                description="Forwarded to the simulation include once that "
                            "package exists; unused today.",
            ),
            DeclareLaunchArgument("robot_name", default_value="rmf_owl"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("launch_prefix", default_value=""),
            core,
        ]
    )
