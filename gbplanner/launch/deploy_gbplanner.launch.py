"""One namespaced gbplanner stack on a real robot.

Port of launch/custom_robot/deploy_gbplanner.launch. The sibling
sim_gbplanner.launch.py is the same file with simulation defaults; keep the two
in sync.

The three topic arguments are the whole integration surface: point them at
whatever the vehicle actually publishes.

    ros2 launch gbplanner deploy_gbplanner.launch.py \\
        namespace:=my_robot odometry_topic:=odometry \\
        pointcloud_topic:=lidar/points

Namespacing caveat, inherited from ROS 1 and NOT fixed here: several names are
hard-coded absolute in C++ and so ignore `namespace` -- see the note in
sim_gbplanner.launch.py and gbplanner/NODE_API.md.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("gbplanner")
    config_folder = PathJoinSubstitution([pkg_share, "config", "custom_robot"])

    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "gbplanner.launch.py"])
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "config_folder": config_folder,
            # custom_robot spells these without the _sim suffix the core
            # defaults to, so they are passed explicitly.
            "voxblox_config_file": PathJoinSubstitution(
                [config_folder, "voxblox_config.yaml"]),
            "pci_config_file": PathJoinSubstitution(
                [config_folder, "planner_control_interface_config.yaml"]),
            "odometry_topic": LaunchConfiguration("odometry_topic"),
            "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
            "command_trajectory_topic": LaunchConfiguration(
                "command_trajectory_topic"),
            "local_navigation_goal_topic": LaunchConfiguration(
                "local_navigation_goal_topic"),
            "land_service": LaunchConfiguration("land_service"),
            "rviz": LaunchConfiguration("rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "launch_prefix": LaunchConfiguration("launch_prefix"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="my_robot",
                description="Robot identity; everything relative lands under "
                            "/<namespace>/.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="On a real robot the driver stack owns the clock.",
            ),
            DeclareLaunchArgument("odometry_topic", default_value="odometry"),
            DeclareLaunchArgument("pointcloud_topic", default_value="lidar/points"),
            DeclareLaunchArgument(
                "command_trajectory_topic", default_value="command/trajectory"),
            DeclareLaunchArgument(
                "local_navigation_goal_topic",
                default_value="move_base_simple/goal",
                description="Relative, so each namespaced robot gets its own "
                            "RViz nav goal.",
            ),
            DeclareLaunchArgument(
                "land_service",
                default_value="land_srv",
                description="gbplanner_node calls this once when the "
                            "auto-landing budget expires; identity by default, "
                            "so nothing happens until it is pointed at the "
                            "vehicle's landing service.",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("launch_prefix", default_value=""),
            core,
        ]
    )
