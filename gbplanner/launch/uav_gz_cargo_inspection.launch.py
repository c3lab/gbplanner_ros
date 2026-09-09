"""UAV cargo-tank inspection with the actuated camera: simulator, planner, RViz.

The ROS 2 equivalent of ROS 1's
`roslaunch gbplanner uav_gz_cargo_act_cam_inspection.launch`, on Gazebo Harmonic.
Unlike the other scenarios this one was already gz-flavoured upstream, so the
world needed renaming rather than porting.

    ros2 launch gbplanner uav_gz_cargo_inspection.launch.py

What makes it different from the exploration scenarios is the camera. The
active_cam_inspection config plans viewpoints for a camera that is pitched under
the vehicle, so the loop has to be closed through Gazebo:

    pci_general  --(std_msgs/Float64)-->  /rmf_owl/camera_pitch
                                            -> JointPositionController
    JointStatePublisher --(JointState)-->  /rmf_owl/camera_pitch_state
                                            -> gbplanner_node, pci_general

`cam_pitch:=true` on the simulation is what puts that joint, its controller and
its state publisher into the spawned model and bridges both topics. The image
sensors stay off: nothing in the stack subscribes to them, and each one costs a
render pass the gpu_lidar is already competing for.

The tank meshes live in the subt_cave_sim checkout - under 100 KB, the cheapest
asset set of any scenario here, but still not vendored; `make run-sim uav_cargo`
mounts it. There is no primitives-only stand-in: the thing being inspected is
the geometry.
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
            # The spawn pose the ROS 1 cargo launch used.
            "x": "0.0",
            "y": "-5.0",
            "z": "1.5",
            "headless": LaunchConfiguration("headless"),
            "headless_rendering": "true",
            "resource_path": "/opt/subt_cave_sim/models",
            "path_topic": "/gbplanner_path",
            "cam_pitch": "true",
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
                [pkg_share, "config", "uav", "gz", "active_cam_inspection"]),
            # Not ground_truth/odometry_throttled as in ROS 1: that name came
            # from a topic_tools throttle that no longer exists, and gz's
            # OdometryPublisher publishes here.
            "odometry_topic": ["/", robot_name, "/odometry"],
            "pointcloud_topic": ["/", robot_name, "/lidar/points"],
            "command_trajectory_topic": [robot_name, "/command/trajectory"],
            "local_navigation_goal_topic": "/move_base_simple/goal",
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
                "world", default_value="cargo_hold",
                description="World in gbplanner_gz_sim/worlds."),
            DeclareLaunchArgument("robot_name", default_value="rmf_owl"),
            DeclareLaunchArgument(
                "cam_pitch_topic", default_value="/rmf_owl/camera_pitch_state"),
            DeclareLaunchArgument(
                "act_cam_cmd_topic", default_value="/rmf_owl/camera_pitch"),
            DeclareLaunchArgument(
                "headless", default_value="false",
                description="Run gz without its GUI. RViz is separate and "
                            "follows the rviz argument."),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("launch_prefix", default_value=""),
            GroupAction([simulation], scoped=True),
            GroupAction([core], scoped=True),
        ]
    )
