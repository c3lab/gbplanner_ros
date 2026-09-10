"""Unitree Go2 exploring, planned for as a ground robot.

    ros2 launch gbplanner go2_gz_exploration.launch.py

The robot and its gait come from RB0609/Unitree_Go2_Edu, which is CHAMP - the
MIT Cheetah controller - running on ROS 2 Jazzy and Gazebo Harmonic. That
combination is the reason this scenario exists at all: upstream CHAMP's ROS 2
branch is Humble on Gazebo Classic, and the one quadruped stack that is already
Jazzy on Harmonic gives twelve links the same names as their joints, which
SDFormat refuses to load.

What that buys, beyond a robot that exists: a legged robot turns on the spot
without a minimum radius, so the whole family of failures the wheeled models
produced - inverted wheel axes, skid-steer scrub, a tracker orbiting a
lookahead point inside its own turning circle - simply does not arise.

CHAMP walks from a geometry_msgs/Twist on /cmd_vel, which is exactly what the
path follower already publishes, so nothing had to be written between the
planner and the legs.

The pieces, and where each comes from:

    gz, the robot, the gait, the EKF   unitree_go2_sim (third party)
    the elevation layer                elevation_mapping (ANYbotics)
    planner and control interface      gbplanner
    path to Twist                      gbplanner_gz_control

Trigger it the way the RViz panel does, and in this order - the initialisation
walk is not optional for a ground robot, because nothing sees the ground
underneath one until it has walked over it:

    ros2 service call /pci_initialization_trigger planner_msgs/srv/PciInitialization
    ros2 service call /planner_control_interface/std_srvs/automatic_planning \\
        std_srvs/srv/Trigger
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("gbplanner")
    sim_share = get_package_share_directory("gbplanner_gz_sim")

    # The robot brings its own gz, so the world is handed to it rather than
    # started here. Ours live in gbplanner_gz_sim; theirs is an indoor depot.
    world = PathJoinSubstitution(
        [FindPackageShare("gbplanner_gz_sim"), "worlds",
         [LaunchConfiguration("world"), ".sdf"]])

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("unitree_go2_sim"), "launch",
                 "unitree_go2_launch.py"])
        ),
        launch_arguments={
            "world": world,
            "world_init_x": LaunchConfiguration("x"),
            "world_init_y": LaunchConfiguration("y"),
            "world_init_z": LaunchConfiguration("z"),
            "world_init_heading": LaunchConfiguration("yaw"),
            "rviz": "false",
            # Their robot, with gz's odometry publisher switched back on - see
            # the wrapper's header. Without it robot_localization has no
            # translation to integrate and the whole stack plans for a robot
            # standing at the origin.
            "unitree_go2_description_path": PathJoinSubstitution(
                [FindPackageShare("gbplanner_gz_sim"), "models", "go2",
                 "go2_with_odometry.xacro"]),
        }.items(),
    )

    # The layer the ground-robot mode projects onto. Started a little late so
    # its one-shot floor-plane initializer has a TF tree to look base_link up
    # in; the robot is spawned standing, so there is no drop to wait out.
    elevation_map = TimerAction(
        period=8.0,
        actions=[Node(
            package="elevation_mapping",
            executable="elevation_mapping",
            name="elevation_mapping",
            output="screen",
            parameters=[
                os.path.join(sim_share, "config", "go2_elevation_mapping.yaml"),
                {"use_sim_time": True},
            ],
            remappings=[("elevation_map", "/elevation_map")],
        )],
    )

    # world -> odom, identity.
    #
    # Everything in this repository works in "world": gbplanner's
    # PlanningParams.global_frame_id, voxblox's world_frame, the elevation map's
    # map_frame_id. CHAMP's tree is rooted at "odom" and no such frame exists in
    # it, so without this the point clouds cannot be transformed and every graph
    # is the root vertex alone - measured, 92 planning calls at one vertex each
    # while the robot walked 35 m.
    #
    # Identity is correct rather than convenient: odom is fixed at the spawn
    # point, and the spawn point is where the world's origin is put.
    world_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_odom",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "odom"],
        parameters=[{"use_sim_time": True}],
    )

    # Path to Twist. CHAMP's own smoother sits behind /cmd_vel.
    follower = Node(
        package="gbplanner_gz_control",
        executable="ugv_path_follower_node",
        name="ugv_path_follower_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("gbplanner_gz_control"),
                "config", "go2_path_follower.yaml"),
            {"use_sim_time": True},
        ],
        remappings=[
            ("command/path", "/pci_command_path"),
            ("odometry", "/odom"),
            ("command/velocity", "/cmd_vel"),
        ],
    )

    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "gbplanner.launch.py"])
        ),
        launch_arguments={
            "config_folder": PathJoinSubstitution(
                [pkg_share, "config", "go2", "gz", "exploration"]),
            # The EKF's filtered output. Not /odom/raw, which is CHAMP's
            # own leg-kinematics estimate and reports no translation at all.
            "odometry_topic": "/odom",
            # gz appends /points to a lidar's topic, and the bridge carries the
            # name through unchanged.
            "pointcloud_topic": "/velodyne_points/points",
            "command_trajectory_topic": "command/trajectory",
            "local_navigation_goal_topic": "/move_base_simple/goal",
            "rviz": LaunchConfiguration("rviz"),
            "use_sim_time": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            # Where docker-compose mounts the subt_cave_sim tiles. Set here
            # because the robot's launch starts gz and knows nothing about them.
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                "/opt/subt_cave_sim/models:" + os.environ.get("GZ_SIM_RESOURCE_PATH", "")),
            DeclareLaunchArgument(
                "world", default_value="cave_box",
                description="World in gbplanner_gz_sim/worlds. cave_box needs no "
                            "external assets; niosh_osrf and the rest need "
                            "subt_cave_sim."),
            # 0.40, which is what the robot's own launch spawns it at. Not the
            # 0.32 a standing Go2's base_link sits at: a quadruped is spawned
            # with its legs folded and has to push itself up, so dropping it in
            # at standing height puts the feet through the floor and it never
            # stands - measured, the body ended at z = 0.236 with the gait never
            # starting and the odometry flat at zero.
            #
            # That costs a small settle, which matters because
            # elevation_mapping's floor-plane initializer samples base_link
            # once; the node is started eight seconds in, by which time the
            # robot is standing.
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.40"),
            # Their launch defaults the spawn heading to 0 and we never
            # passed it, so there was no way to point the robot at all.
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("rviz", default_value="true"),
            GroupAction([robot], scoped=True),
            world_to_odom,
            elevation_map,
            follower,
            GroupAction([core], scoped=True),
        ]
    )
