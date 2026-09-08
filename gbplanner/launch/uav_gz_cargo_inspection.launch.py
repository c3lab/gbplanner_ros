"""UAV cargo-bay inspection in Gazebo (gz), the direct port of
launch/uav/gz/uav_gz_cargo_inspection.launch.

Same core stack as the cave scenario; the differences are the config folder
(active_cam_inspection, which turns on the actively pitched camera) and the two
camera topics that carry the pitch command and its state.

    ros2 launch gbplanner uav_gz_cargo_inspection.launch.py
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

    # ------------------------------------------------------------------
    # SIMULATION INCLUDE POINT -- deliberately not wired up.
    #
    # The ROS 1 file included
    #     $(find arl_gazebo_sim)/gz/uav_sim/launch/uav_gz_cargo_inspection.launch
    # which spawned gz, the rmf_owl model, its controllers and the pitching
    # camera joint that feeds /camera_joint_pitch_state. The ROS 2 replacement
    # is meant to be sim/gbplanner_gz_sim (docker-compose already bind-mounts it
    # as gbplanner3_ws/src/sim/gbplanner_gz_sim); that directory is still empty,
    # so there is no launch file to include and no argument list to honour.
    #
    # When the package lands, add -- before the include below:
    #
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(PathJoinSubstitution([
    #             FindPackageShare("gbplanner_gz_sim"),
    #             "launch", "<the cargo world file>.launch.py"])),
    #         launch_arguments={"world_name": world_name,
    #                           "robot_name": robot_name}.items(),
    #     )
    #
    # Without it the camera topics below have no publisher and no subscriber,
    # so the inspection planner will sit on its default pitch.
    # ------------------------------------------------------------------

    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "gbplanner.launch.py"])
        ),
        launch_arguments={
            "config_folder": PathJoinSubstitution(
                [pkg_share, "config", "uav", "gz", "active_cam_inspection"]),
            "odometry_topic": ["/", robot_name, "/ground_truth/odometry_throttled"],
            "pointcloud_topic": ["/", robot_name, "/lidar/points"],
            "command_trajectory_topic": [robot_name, "/command/trajectory"],
            # Read by both nodes: gbplanner_node uses the pitch to place the
            # sensor frustum, the PCI to close the loop on its own command.
            "cam_pitch_topic": LaunchConfiguration("cam_pitch_topic"),
            "act_cam_cmd_topic": LaunchConfiguration("act_cam_cmd_topic"),
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
            DeclareLaunchArgument(
                "cam_pitch_topic", default_value="/camera_joint_pitch_state"),
            DeclareLaunchArgument(
                "act_cam_cmd_topic", default_value="/camera_joint_pitch"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("launch_prefix", default_value=""),
            core,
        ]
    )
