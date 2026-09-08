"""gbplanner core stack: planner node + planner control interface + RViz2.

This is the reusable piece; every scenario entry point in this directory
includes it and only overrides the config folder and the robot's topic names.
Launched on its own it runs the cave-exploration config against neutral topic
names, which is what the smoke test does:

    ros2 launch gbplanner gbplanner.launch.py rviz:=false

Two things about this file are consequences of the ROS 2 port rather than
choices:

  * There is no global parameter namespace any more, so each node carries its
    own ``parameters=[...]`` list.  ``gbplanner_node`` hosts a
    ``voxblox::TsdfServer`` in the same process, which is why it needs both
    ``gbplanner_config.yaml`` and the voxblox YAML -- in that order, the later
    file winning on any key they share.  Both are ``/**:``-scoped so they apply
    whatever the node ends up being named or namespaced.

  * voxblox's sensor input is node-private in the ROS 2 port (``~/pointcloud``,
    i.e. ``/<ns>/gbplanner_node/pointcloud``), where ROS 1 had a global
    ``/pointcloud``.  The remap below therefore matches ``~/pointcloud``, not
    ``/pointcloud``; writing the fully qualified name instead would stop
    matching the moment a namespace is pushed.

Names that a namespace does NOT reach are listed in the two nodes below.  They
are hard-coded absolute in C++ (see gbplanner/NODE_API.md) and were absolute in
ROS 1 too; the remaps that ROS 1 shipped for them are reproduced here, the rest
are called out in comments where they bite.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("gbplanner")
    default_config_folder = PathJoinSubstitution(
        [pkg_share, "config", "uav", "gz", "cave_exploration"]
    )

    namespace = LaunchConfiguration("namespace")
    config_folder = LaunchConfiguration("config_folder")
    gbplanner_config_file = LaunchConfiguration("gbplanner_config_file")
    voxblox_config_file = LaunchConfiguration("voxblox_config_file")
    pci_config_file = LaunchConfiguration("pci_config_file")
    behavior_tree_file = LaunchConfiguration("behavior_tree_file")
    rviz_config = LaunchConfiguration("rviz_config")

    odometry_topic = LaunchConfiguration("odometry_topic")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    command_trajectory_topic = LaunchConfiguration("command_trajectory_topic")
    local_navigation_goal_topic = LaunchConfiguration("local_navigation_goal_topic")
    cam_pitch_topic = LaunchConfiguration("cam_pitch_topic")
    act_cam_cmd_topic = LaunchConfiguration("act_cam_cmd_topic")
    land_service = LaunchConfiguration("land_service")

    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_prefix = LaunchConfiguration("launch_prefix")

    sim_time_param = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}

    stack = GroupAction(
        [
            PushRosNamespace(namespace),
            Node(
                package="gbplanner",
                executable="gbplanner_node",
                name="gbplanner_node",
                output="screen",
                emulate_tty=True,
                prefix=launch_prefix,
                parameters=[
                    gbplanner_config_file,
                    voxblox_config_file,
                    {
                        # A plain node parameter in ROS 2, where ROS 1 had a
                        # <param> child. Absent, the node falls back to
                        # <share>/gbplanner/config/bt_xml/main_tree.xml.
                        "behavior_tree_path": ParameterValue(
                            behavior_tree_file, value_type=str
                        ),
                    },
                    sim_time_param,
                ],
                remappings=[
                    ("odometry", odometry_topic),
                    # voxblox's main sensor input, node-private in ROS 2.
                    ("~/pointcloud", pointcloud_topic),
                    ("local_navigation_goal", local_navigation_goal_topic),
                    ("cam_pitch", cam_pitch_topic),
                    # A client, not a server: fired once when the auto-landing
                    # budget expires. Identity by default; point it at the
                    # vehicle's own landing service to arm it.
                    ("land_srv", land_service),
                    # These two are absolute in C++, and the PCI pulls its own
                    # ends of them back to relative names. Without the matching
                    # pull here the pair straddles the namespace boundary: under
                    # namespace:=robot0 the PCI would publish /robot0/robot_status
                    # while the planner listened on /robot_status, and battery-time
                    # reporting would never connect. ROS 1 had the same asymmetry
                    # because its launch only remapped the publisher.
                    ("/robot_status", "robot_status"),
                    ("/gbplanner_path", "gbplanner_path"),
                    # Left absolute on purpose: published by a node this launch
                    # does not start, so there is no second end to keep in step.
                    #   /traversability_estimation/untraversable_polygon
                ],
            ),
            Node(
                package="pci_general",
                executable="pci_general_ros_node",
                name="pci_general_ros_node",
                output="screen",
                emulate_tty=True,
                parameters=[pci_config_file, sim_time_param],
                remappings=[
                    ("command/trajectory", command_trajectory_topic),
                    # The two clients that drive the planner. gbplanner_node
                    # advertises gbplanner_ros (behaviour-tree entry point) and
                    # gbplanner/homing.
                    ("planner_server", "gbplanner_ros"),
                    ("planner_homing_server", "gbplanner/homing"),
                    ("odometry", odometry_topic),
                    ("cam_pitch", cam_pitch_topic),
                    ("act_cam_cmd", act_cam_cmd_topic),
                    # pci_general.cpp hard-codes these three publishers and
                    # planner_control_interface.cpp this subscription with a
                    # leading slash, so they escape any namespace. Pulled back
                    # to relative names, exactly as the ROS 1 launch did.
                    ("/gbplanner_path", "gbplanner_path"),
                    ("/gbplanner_is_homing", "gbplanner_is_homing"),
                    ("/robot_status", "robot_status"),
                    ("/move_base_simple/goal", local_navigation_goal_topic),
                    # Both servers live on gbplanner_node under a relative
                    # name, so these clients have to follow it into the
                    # namespace or the inspection trigger and the trigger-mode
                    # switch never reach a namespaced planner.
                    ("/gbplanner/get_inspection_path", "gbplanner/get_inspection_path"),
                    ("/gbplanner/set_planning_trigger_mode",
                     "gbplanner/set_planning_trigger_mode"),
                    # Left absolute: the other end of each of these belongs to a
                    # node this launch does not start.
                    #   /matrice/status, /pci_general/path_following,
                    #   /global_planner/waypoint_request, /gazebo/unpause_physics
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                # Node name left at rviz2's default: the panel's clients live on
                # rviz2's own internally created node and a __node override was
                # not verifiable without a display.
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[sim_time_param],
                condition=IfCondition(rviz),
                remappings=[
                    # gbplanner_ui's panel creates these four clients absolute
                    # while the PCI advertises them relative; under a namespace
                    # they would never meet.
                    (
                        "/planner_control_interface/std_srvs/automatic_planning",
                        "planner_control_interface/std_srvs/automatic_planning",
                    ),
                    (
                        "/planner_control_interface/std_srvs/single_planning",
                        "planner_control_interface/std_srvs/single_planning",
                    ),
                    (
                        "/planner_control_interface/std_srvs/stop",
                        "planner_control_interface/std_srvs/stop",
                    ),
                    (
                        "/planner_control_interface/std_srvs/homing_trigger",
                        "planner_control_interface/std_srvs/homing_trigger",
                    ),
                    (
                        "/planner_control_interface/std_srvs/go_to_waypoint",
                        "planner_control_interface/std_srvs/go_to_waypoint",
                    ),
                ],
            ),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="ROS namespace for the whole stack; empty = root.",
            ),
            DeclareLaunchArgument(
                "config_folder",
                default_value=default_config_folder,
                description="Folder holding this scenario's YAML, main_tree.xml "
                            "and ui.rviz.",
            ),
            # The custom_robot folder spells its files without the _sim suffix,
            # so deploy_gbplanner/sim_gbplanner override these three by name.
            DeclareLaunchArgument(
                "gbplanner_config_file",
                default_value=PathJoinSubstitution(
                    [config_folder, "gbplanner_config.yaml"]),
            ),
            DeclareLaunchArgument(
                "voxblox_config_file",
                default_value=PathJoinSubstitution(
                    [config_folder, "voxblox_sim_config.yaml"]),
            ),
            DeclareLaunchArgument(
                "pci_config_file",
                default_value=PathJoinSubstitution(
                    [config_folder, "planner_control_interface_sim_config.yaml"]),
            ),
            DeclareLaunchArgument(
                "behavior_tree_file",
                default_value=PathJoinSubstitution([config_folder, "main_tree.xml"]),
                description="BT XML. Its <include path=...> entries are resolved "
                            "relative to it, so it has to be read from the "
                            "installed share tree, not the source tree.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution([config_folder, "ui.rviz"]),
            ),
            DeclareLaunchArgument(
                "odometry_topic",
                default_value="odometry",
                description="The only robot state source the stack is wired to; "
                            "gbplanner_node also accepts pose/pose_stamped.",
            ),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="pointcloud",
                description="Sensor cloud into voxblox.",
            ),
            DeclareLaunchArgument(
                "command_trajectory_topic", default_value="command/trajectory"),
            DeclareLaunchArgument(
                "local_navigation_goal_topic",
                default_value="move_base_simple/goal",
                description="Relative on purpose: keeps two namespaced robots "
                            "from sharing one RViz nav goal.",
            ),
            DeclareLaunchArgument("cam_pitch_topic", default_value="cam_pitch"),
            DeclareLaunchArgument("act_cam_cmd_topic", default_value="act_cam_cmd"),
            DeclareLaunchArgument("land_service", default_value="land_srv"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "launch_prefix",
                default_value="",
                description="e.g. 'gdb -ex run --args' for gbplanner_node.",
            ),
            stack,
        ]
    )
