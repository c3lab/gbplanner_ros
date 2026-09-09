"""UGV exploration of the NIOSH mine: simulator, planner, control interface, RViz.

The ROS 2 equivalent of ROS 1's `roslaunch gbplanner ugv_gzc_niosh_exploration.launch`,
on Gazebo Harmonic rather than Gazebo Classic.

    ros2 launch gbplanner ugv_gz_niosh_exploration.launch.py

Two differences from the ROS 1 demo are worth knowing before comparing them:

  * The robot is the MARBLE Husky, not the SMB. The ROS 1 launch spawned
    smb_gazebo's SMB and tracked paths with smb_path_tracker's pure pursuit;
    neither package was ported, while marble_husky was already a gz model whose
    every system ships with Harmonic. The footprints are within a few
    centimetres of each other, so the planner config is upstream's except for
    RobotParams.center_offset, which lifts the collision box to where the SMB's
    higher base frame put it; the config comment says why.
  * The world is niosh_osrf, which is what the launch file's *name* promises and
    what uav_gzc_niosh_exploration.launch loads. The ROS 1 UGV file actually
    defaults to pittsburgh_mine.world; that looks like a copy-paste that was
    never noticed, and following it would leave "niosh" naming a world that is
    not NIOSH.

How long the robot sits still after the trigger is a property of the world, not
of the robot. On niosh_osrf there is no warm-up: the first planning call already
returns 240-270 vertices. On cave_box the Husky starts half a metre from a wall
in a world with nothing else in it and spends roughly 1400 planning calls on the
root vertex alone before the map has enough free space to sample through.

Two of four 420 s runs on niosh_osrf ended badly - one wedged on the tunnel
floor after 4 m with the follower still commanding 0.54 m/s, one fell through
the mesh. The other two explored 20 m and 22 m from the start. README.md has
both under "Known differences"; neither is a planner fault and neither is fixed.

The tunnel mesh lives in the subt_cave_sim checkout (223 MB in one DAE),
deliberately not vendored here; `make run-sim ugv_niosh` mounts it. Without
those assets, pass `world:=cave_box` for the primitives-only world that ships
with gbplanner_gz_sim - it has a flat floor and drives fine.
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
            # Without this gz never creates an EGL context and the gpu_lidar
            # raycasts on the CPU.
            "headless_rendering": "true",
            "resource_path": "/opt/subt_cave_sim/models",
            "path_topic": "/pci_command_path",
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
                [pkg_share, "config", "ugv", "gz", "niosh_exploration"]),
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
            DeclareLaunchArgument("robot_name", default_value="marble_husky"),
            # The spawn pose the ROS 1 UGV launch used, lifted to where the
            # Husky's wheels clear the floor: base_link sits 0.13 m above the
            # ground, and a model spawned intersecting the terrain is ejected
            # rather than settled.
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.5"),
            DeclareLaunchArgument(
                "headless", default_value="false",
                description="Run gz without its GUI. RViz is separate and "
                            "follows the rviz argument."),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            # Scoped for the reason spelled out in uav_gz_cave_exploration:
            # IncludeLaunchDescription does not open a scope of its own, so the
            # simulation's rviz:=false would otherwise overwrite the shared
            # configuration the planner include reads back.
            GroupAction([simulation], scoped=True),
            GroupAction([core], scoped=True),
        ]
    )
