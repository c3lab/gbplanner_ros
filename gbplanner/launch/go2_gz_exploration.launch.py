"""Unitree Go2 exploring one of this repository's worlds, planned for as a ground robot.

    ros2 launch gbplanner go2_gz_exploration.launch.py world:=cave_box

The robot, its gait and its state estimation come from RB0609/Unitree_Go2_Edu -
CHAMP, the MIT Cheetah controller, on ROS 2 Jazzy and Gazebo Harmonic. The
world, the gz process and the spawn are ours, exactly as they are for rmf_owl,
marble_husky and the ANYmal.

That split is the point, and it is why this file no longer includes their
unitree_go2_launch.py. That launch declares a `world` argument and never reads
it: its gz_args names unitree_go2_description/worlds/model.sdf literally, so
passing a world does nothing and fails silently. The Go2 ran in their indoor
depot - ROOF, WALLS, PILLERS, FANS - while every measurement was being recorded
against cave_box. The depot also declares no gz-sim-imu-system, so the robot's
IMU was never instantiated and the EKF that consumes it ran blind.

So: their nodes, our scene. Nothing in their package is edited.

Trigger it in this order - the initialisation walk is not optional for a ground
robot, because nothing sees the ground underneath one until it has walked over
it:

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
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    gb_share = get_package_share_directory("gbplanner")
    sim_share = get_package_share_directory("gbplanner_gz_sim")
    go2_sim = get_package_share_directory("unitree_go2_sim")
    champ_base = get_package_share_directory("champ_base")

    joints_config = os.path.join(go2_sim, "config", "joints", "joints.yaml")
    links_config = os.path.join(go2_sim, "config", "links", "links.yaml")
    gait_config = os.path.join(go2_sim, "config", "gait", "gait.yaml")
    bridge_yaml = os.path.join(go2_sim, "scripts", "gazebo_bridge.yaml")

    sim_time = {"use_sim_time": True}

    # Their robot with gz's odometry publisher switched on - see the wrapper's
    # README. ParameterValue(..., str) because a bare Command substitution is
    # parsed as YAML, and any prose in the generated XML fails the launch.
    description = os.path.join(sim_share, "models", "go2", "go2_with_odometry.xacro")

    def urdf():
        return ParameterValue(Command(["xacro ", description]), value_type=str)

    # ---- our scene -------------------------------------------------------
    world = PathJoinSubstitution(
        [FindPackageShare("gbplanner_gz_sim"), "worlds",
         [LaunchConfiguration("world"), ".sdf"]])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"),
                         "launch", "gz_sim.launch.py")),
        # --physics-engine, and it is the difference between a quadruped that
        # walks and one that ends up on its back. Their depot world declares
        #   <physics type="bullet-featherstone"> max_step_size 0.004
        # and CHAMP's gait is tuned against it. Our worlds leave the engine
        # unset, so gz picks DART - and DART's default solver on twelve joints
        # under stiff effort control is what the vibration and the flip were.
        # Measured on a live run: gz reported roll = -3.14159 and body z = 0.106
        # against 0.24 standing, while the two_d_mode EKF went on reporting a
        # level robot at yaw 0, so the planner never saw it happen.
        #
        # Passed here rather than written into the worlds: the UAV and the
        # wheeled robots run correctly on DART and share those files.
        launch_arguments={
            "gz_args": [world, " -r -v 1 --physics-engine gz-physics-bullet-featherstone-plugin"],
        }.items(),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_go2",
        output="screen",
        arguments=[
            "-name", "go2",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-Y", LaunchConfiguration("yaw"),
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_bridge",
        output="screen",
        parameters=[{"config_file": bridge_yaml}, sim_time],
    )

    # ---- their robot -----------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf()}, sim_time],
    )

    # Their timings, kept: the controller manager lives inside gz and is not
    # ready until the world has loaded, which takes longer for our worlds than
    # for their depot.
    spawn_joint_states = TimerAction(period=20.0, actions=[Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["--controller-manager-timeout", "120", "joint_states_controller"],
        parameters=[sim_time])])
    spawn_effort = TimerAction(period=30.0, actions=[Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["--controller-manager-timeout", "120", "joint_group_effort_controller"],
        parameters=[sim_time])])

    quadruped_controller = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            sim_time,
            {"gazebo": True},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            {"joint_controller_topic": "joint_group_effort_controller/joint_trajectory"},
            {"urdf": urdf()},
            joints_config, links_config, gait_config,
            {"hardware_connected": False},
            {"close_loop_odom": True},
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    state_estimator = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        parameters=[
            sim_time,
            {"orientation_from_imu": True},
            {"urdf": urdf()},
            joints_config, links_config, gait_config,
        ],
    )

    base_to_footprint_ekf = Node(
        package="robot_localization", executable="ekf_node",
        name="base_to_footprint_ekf", output="screen",
        parameters=[
            {"base_link_frame": "base_link"}, sim_time,
            os.path.join(champ_base, "config", "ekf", "base_to_footprint.yaml"),
        ],
        remappings=[("odometry/filtered", "odom/local")],
    )

    footprint_to_odom_ekf = Node(
        package="robot_localization", executable="ekf_node",
        name="footprint_to_odom_ekf", output="screen",
        parameters=[
            sim_time,
            {"map_frame": "map"}, {"base_link_frame": "base_footprint"},
            {"odom_frame": "odom"}, {"world_frame": "odom"},
            {"publish_tf": True}, {"frequency": 20.0}, {"two_d_mode": True},
            {"odom0": "odom/raw"},
            {"odom0_config": [False, False, False, False, False, False,
                              True, True, False, False, False, True,
                              False, False, False]},
            {"imu0": "imu/data"},
            {"imu0_config": [False, False, False, False, False, True,
                             False, False, False, False, False, True,
                             False, False, False]},
        ],
        remappings=[("odometry/filtered", "odom")],
    )

    # world -> odom, identity. Everything in this repository works in "world":
    # gbplanner's global_frame_id, voxblox's world_frame, the elevation map's
    # map_frame_id. CHAMP's tree is rooted at "odom". Their launch publishes a
    # map -> odom instead; this replaces it rather than joining it, because odom
    # cannot have two parents.
    world_to_odom = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="world_to_odom", output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "odom"],
        parameters=[sim_time],
    )

    # ---- the planner's inputs -------------------------------------------
    # The elevation layer the ground-robot mode projects onto. Late enough that
    # its one-shot floor-plane initializer has a TF tree to look base_link up in,
    # and that the robot is standing rather than folded.
    elevation_map = TimerAction(period=35.0, actions=[Node(
        package="elevation_mapping", executable="elevation_mapping",
        name="elevation_mapping", output="screen",
        parameters=[os.path.join(sim_share, "config", "go2_elevation_mapping.yaml"),
                    sim_time],
        remappings=[("elevation_map", "/elevation_map")],
    )])

    follower = Node(
        package="gbplanner_gz_control",
        executable="ugv_path_follower_node",
        name="ugv_path_follower_node",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("gbplanner_gz_control"),
                         "config", "go2_path_follower.yaml"),
            sim_time,
        ],
        remappings=[
            ("command/path", "/pci_command_path"),
            ("odometry", "/odom"),
            ("command/velocity", "/cmd_vel"),
        ],
    )

    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gb_share, "launch", "gbplanner.launch.py")),
        launch_arguments={
            "config_folder": os.path.join(gb_share, "config", "go2", "gz", "exploration"),
            # The EKF's filtered output. Not /odom/raw, which is CHAMP's own
            # leg-kinematics estimate and reports no translation at all.
            "odometry_topic": "/odom",
            # gz appends /points to a lidar's topic and the bridge carries the
            # name through unchanged.
            "pointcloud_topic": "/velodyne_points/points",
            "command_trajectory_topic": "command/trajectory",
            "local_navigation_goal_topic": "/move_base_simple/goal",
            "rviz": LaunchConfiguration("rviz"),
            "use_sim_time": "true",
        }.items(),
    )

    return LaunchDescription([
        # Where docker-compose mounts the subt_cave_sim tiles, for every world
        # except cave_box.
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            "/opt/subt_cave_sim/models:" + os.environ.get("GZ_SIM_RESOURCE_PATH", "")),
        DeclareLaunchArgument(
            "world", default_value="cave_box",
            description="World in gbplanner_gz_sim/worlds. cave_box needs no "
                        "external assets; the others need subt_cave_sim."),
        # A standing Go2's base_link is 0.24 m up; 0.40 is what the robot's own
        # launch spawns it at, because a quadruped starts with its legs folded
        # and has to push itself up. Worlds whose floor is not at zero need this
        # raised by the floor's height - urban_circuit_01's is about -8.0, where
        # the proven spawn is x=70, y=-100.
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.40"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("rviz", default_value="true"),
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        spawn_joint_states,
        spawn_effort,
        quadruped_controller,
        state_estimator,
        base_to_footprint_ekf,
        footprint_to_odom_ekf,
        world_to_odom,
        elevation_map,
        follower,
        GroupAction([core], scoped=True),
    ])
