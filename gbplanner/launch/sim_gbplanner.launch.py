"""One namespaced gbplanner stack against a simulated robot.

This is what docker-compose's `run-sim` service calls, once per container:

    ros2 launch gbplanner sim_gbplanner.launch.py \\
        namespace:=${NAMESPACE} rviz:=${RVIZ} use_sim_time:=true

Port of launch/custom_robot/sim_gbplanner.launch. Identical to
deploy_gbplanner.launch.py apart from the defaults marked [sim]: the topic
names the gz simulation exposes, and sim time on because the simulator owns
/clock. Keep the two files in sync.

Namespacing caveat, inherited from ROS 1 and NOT fixed here: several names are
hard-coded absolute in C++ and so ignore `namespace`. gbplanner.launch.py
reproduces every remap the ROS 1 file had for them, but the ones ROS 1 left
alone stay broken under a namespace -- most visibly gbplanner_node's
/robot_status subscription, which keeps listening at the root while this
launch's PCI publishes into /<namespace>/robot_status. Battery-time reporting
therefore does not connect when `namespace` is non-empty.
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
            DeclareLaunchArgument("use_sim_time", default_value="true"),  # [sim]
            DeclareLaunchArgument(
                "odometry_topic",
                default_value="sensor_measurements/odom"),               # [sim]
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="sensor_measurements/lidar/points"),       # [sim]
            DeclareLaunchArgument(
                "command_trajectory_topic", default_value="command/trajectory"),
            DeclareLaunchArgument(
                "local_navigation_goal_topic",
                default_value="move_base_simple/goal",
                description="Relative, so each namespaced robot gets its own "
                            "RViz nav goal.",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("launch_prefix", default_value=""),
            core,
        ]
    )
