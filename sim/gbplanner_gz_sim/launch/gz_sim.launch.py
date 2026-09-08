"""Gazebo Harmonic simulation for gbplanner: world, robot, bridge, controller.

This is the include point for `gbplanner`'s own launch files. It owns everything
below the planner and publishes exactly what the planner consumes -- see the
"Interface" block below, which is the contract the two halves are joined on.

Three things happen here that are not just "start Gazebo":

  * The robot SDF is a template. Its plugins hard-code their gz topics under
    <robotNamespace>, so N copies of one SDF would all fly on one
    `rmf_owl/command/velocity`. `robot_name` is substituted into the template at
    launch time and the result written to a generated file.
  * The world's `<world name=...>` is read out of the SDF rather than taken as
    an argument. Every gz service and topic path contains it, and a world file
    whose name does not match what the launch file assumed fails as a spawn that
    silently never happens.
  * The velocity controller in the robot SDF ignores commands until something
    publishes `true` on its enable topic. `uav_path_follower_node` does that;
    see gbplanner_gz_control.

Interface
---------
Arguments (all optional):

    world               worlds/<name>.sdf in this package, or an absolute path.
                        default: cave_box
    robot_name          gz model name, gz topic prefix, TF frame prefix and the
                        ROS namespace of every bridged robot topic.
                        default: rmf_owl
    x y z roll pitch yaw  spawn pose. default: 0 0 1 0 0 0
    headless            gz server with no GUI. default: true
    use_sim_time        on the nodes started here. default: true
    bridge              start ros_gz_bridge. default: true
    controller          start uav_path_follower_node. default: true
    rviz                start rviz2 on rviz/sim.rviz. default: false
    camera              simulate and bridge the RGB + depth cameras. Off means
                        gz never renders them. default: false
    lidar               simulate and bridge the gpu_lidar. default: true
    path_topic          the nav_msgs/Path the controller follows.
                        default: /gbplanner_path
    resource_path       extra dirs for GZ_SIM_RESOURCE_PATH (the subt_cave_sim
                        models darpa_cave_01 needs). default: ''
    world_frame         TF root the planner works in. An identity link is
                        published from it to the gz world frame, which gz names
                        after the world. default: world
    lidar_horizontal_samples / lidar_vertical_samples
                        default: 2048 / 128, the ROS 1 values
    spawn_timeout       seconds to wait for the gz world's create service
                        before giving up on the spawn. default: 180
    gz_verbosity        gz -v level. default: 1
    controller_config   override the follower's parameter YAML. default: the
                        one installed by gbplanner_gz_control

Published for the planner (with robot_name=rmf_owl):

    /clock                          rosgraph_msgs/Clock
    /rmf_owl/odometry               nav_msgs/Odometry           50 Hz
    /rmf_owl/lidar/points           sensor_msgs/PointCloud2     10 Hz
                                    header.frame_id rmf_owl/laser_link
    /rmf_owl/imu                    sensor_msgs/Imu            200 Hz
    /tf                             world -> <gz world name> -> rmf_owl
                                          -> rmf_owl/{base,laser}_link, rotor_*

Consumed from the planner:

    /gbplanner_path                 nav_msgs/Path (pci_general publishes it)

So the planner side needs, on gbplanner_node:

    odometry                  -> /rmf_owl/odometry
    ~/pointcloud              -> /rmf_owl/lidar/points
    use_sim_time              := true
    voxblox world_frame        = world   (already the config default)
"""

import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Where rendered per-robot SDFs go. Not the package share dir: --symlink-install
# would make that a write into the source tree, and two robots racing on one
# file is a spawn that silently gets the wrong name.
GENERATED_SDF_DIR = Path("/tmp/gbplanner_gz_sim")

_ARGS = [
    ("world", "cave_box", "worlds/<name>.sdf in this package, or an absolute path."),
    ("robot_name", "rmf_owl", "gz model name; prefixes every gz topic and TF frame."),
    ("x", "0.0", "Spawn x [m]."),
    ("y", "0.0", "Spawn y [m]."),
    ("z", "1.0", "Spawn z [m]."),
    ("roll", "0.0", "Spawn roll [rad]."),
    ("pitch", "0.0", "Spawn pitch [rad]."),
    ("yaw", "0.0", "Spawn yaw [rad]."),
    ("headless", "true", "Run the gz server without its GUI."),
    ("use_sim_time", "true", "Set use_sim_time on the nodes started here."),
    ("bridge", "true", "Start ros_gz_bridge."),
    ("controller", "true", "Start uav_path_follower_node."),
    ("rviz", "false", "Start rviz2 on this package's sim.rviz."),
    ("camera", "false", "Simulate and bridge the RGB and depth cameras."),
    ("lidar", "true", "Simulate and bridge the gpu_lidar."),
    ("path_topic", "/gbplanner_path", "nav_msgs/Path the controller follows."),
    ("resource_path", "", "Extra directories for GZ_SIM_RESOURCE_PATH."),
    ("world_frame", "world", "TF root the planner works in; tied to the gz world frame."),
    ("lidar_horizontal_samples", "2048", "gpu_lidar horizontal beams."),
    ("lidar_vertical_samples", "128", "gpu_lidar vertical beams."),
    ("spawn_timeout", "180", "Seconds to wait for the gz create service before spawning."),
    ("gz_verbosity", "1", "gz sim -v level."),
    ("controller_config", "", "Override the follower's parameter YAML."),
]


def _resolve_world(share: str, world: str) -> str:
    """Accept either a bare world name or a path, and insist the file exists."""
    if os.path.sep in world or world.endswith(".sdf"):
        path = os.path.abspath(os.path.expanduser(world))
    else:
        path = os.path.join(share, "worlds", world + ".sdf")
    if not os.path.isfile(path):
        raise RuntimeError(f"world file not found: {path}")
    return path


def _world_name(world_file: str) -> str:
    name = ET.parse(world_file).getroot().find("world").get("name")
    if not name:
        raise RuntimeError(f"{world_file} has no <world name=...>")
    return name


def _render_model(share: str, robot_name: str, blocks: dict, subs: dict) -> str:
    """Substitute {{...}} and drop the optional blocks that are switched off."""
    template = Path(share) / "models" / "rmf_owl" / "model.sdf.in"
    text = template.read_text()

    for block, keep in blocks.items():
        marked = re.compile(f"<!--BEGIN:{block}-->(.*?)<!--END:{block}-->", re.DOTALL)
        text = marked.sub((lambda m: m.group(1)) if keep else "", text)

    for key, value in subs.items():
        text = text.replace("{{" + key + "}}", str(value))

    GENERATED_SDF_DIR.mkdir(parents=True, exist_ok=True)
    out = GENERATED_SDF_DIR / f"{robot_name}.generated.sdf"
    out.write_text(text)
    return str(out)


def _setup(context, *args, **kwargs):
    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        return cfg(name).lower() in ("true", "1", "yes")

    share = get_package_share_directory("gbplanner_gz_sim")
    robot = cfg("robot_name")
    world_file = _resolve_world(share, cfg("world"))
    world = _world_name(world_file)

    model_file = _render_model(
        share,
        robot,
        blocks={"camera": flag("camera"), "lidar": flag("lidar")},
        subs={
            "robot_name": robot,
            "lidar_horizontal_samples": cfg("lidar_horizontal_samples"),
            "lidar_vertical_samples": cfg("lidar_vertical_samples"),
        },
    )

    use_sim_time = flag("use_sim_time")

    # ros_gz_sim's launch file reads GZ_SIM_RESOURCE_PATH out of os.environ and
    # hands it to the gz process, so setting it here is enough. The world's tile
    # models (darpa_cave_01) are the reason this exists; cave_box needs nothing.
    resource_path = os.pathsep.join(
        p for p in [cfg("resource_path"), os.environ.get("GZ_SIM_RESOURCE_PATH", "")] if p
    )

    gz_args = f"-r -v {cfg('gz_verbosity')} {'-s ' if flag('headless') else ''}{world_file}"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
    )

    # `create` calls /world/<name>/create once and does not retry, and it reports
    # success even when Gazebo then rejects the SDF. A fixed delay would have to
    # be tuned per world -- darpa_cave_01 spends ~30 s loading tile meshes before
    # that service exists, cave_box under a second -- and getting it wrong is a
    # simulation that comes up with no robot in it and no error anywhere. So wait
    # for the service itself.
    spawn_command = (
        f'for i in $(seq 1 {int(float(cfg("spawn_timeout")))}); do '
        f'  gz service -l 2>/dev/null | grep -qx "/world/{world}/create" && break; sleep 1; '
        f'done; '
        f'exec ros2 run ros_gz_sim create -world {world} -file {model_file} '
        f'-name {robot} -allow_renaming false '
        f'-x {cfg("x")} -y {cfg("y")} -z {cfg("z")} '
        f'-R {cfg("roll")} -P {cfg("pitch")} -Y {cfg("yaw")}'
    )
    spawn = ExecuteProcess(
        cmd=["bash", "-c", spawn_command], name="spawn_" + robot, output="screen"
    )

    # gz -> ROS uses '[', ROS -> gz uses ']'. The gz topic names are the ones
    # baked into the robot SDF by _render_model, so both sides of every pair
    # below are the same string and no gz-side remap is needed.
    bridge_args = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        f"/model/{robot}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        f"/{robot}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        f"/{robot}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        f"/{robot}/command/velocity@geometry_msgs/msg/Twist]gz.msgs.Twist",
        f"/{robot}/enable@std_msgs/msg/Bool]gz.msgs.Boolean",
    ]
    if flag("lidar"):
        bridge_args.append(
            f"/{robot}/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked")
    if flag("camera"):
        bridge_args += [
            f"/{robot}/camera_front@sensor_msgs/msg/Image[gz.msgs.Image",
            f"/{robot}/camera_front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            f"/{robot}/depth_camera_front@sensor_msgs/msg/Image[gz.msgs.Image",
            f"/{robot}/depth_camera_front/points"
            f"@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ]

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("bridge")),
        arguments=bridge_args,
        # PosePublisher's output is the whole TF tree for this robot. It goes to
        # /tf and not /tf_static even for the fixed joints: tf2's static listener
        # subscribes transient_local, the bridge publishes volatile, and those
        # two QoS profiles do not connect. See the SDF for the rest of it.
        remappings=[(f"/model/{robot}/pose", "/tf")],
        # The bridge is what republishes gz's /clock, so it cannot itself wait
        # on that clock to exist.
        parameters=[{"use_sim_time": False}],
    )

    controller_config = cfg("controller_config") or os.path.join(
        get_package_share_directory("gbplanner_gz_control"), "config", "uav_path_follower.yaml"
    )
    controller = Node(
        package="gbplanner_gz_control",
        executable="uav_path_follower_node",
        name="uav_path_follower_node",
        namespace=robot,
        output="screen",
        condition=IfCondition(LaunchConfiguration("controller")),
        parameters=[controller_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("command/path", cfg("path_topic")),
            ("odometry", f"/{robot}/odometry"),
            ("command/velocity", f"/{robot}/command/velocity"),
            ("enable", f"/{robot}/enable"),
        ],
    )

    # gz's PosePublisher roots the tree at the *world's name* -- "cave_box",
    # "cosmos" -- while the planner's voxblox config, the OdometryPublisher and
    # every marker header say "world". Without this identity link the tree is
    # two disconnected halves and voxblox integrates nothing: "Could not find a
    # connection between 'world' and 'rmf_owl/laser_link'". Renaming the world
    # to "world" would do the same job and lose the world's identity in every gz
    # topic path, so the link is published instead.
    world_frame = cfg("world_frame")
    frame_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_frame_link",
        output="log",
        condition=IfCondition("true" if world_frame != world else "false"),
        arguments=["--frame-id", world_frame, "--child-frame-id", world],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="gbplanner_gz_sim_rviz",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", os.path.join(share, "rviz", "sim.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        gazebo,
        bridge,
        spawn,
        frame_link,
        controller,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in _ARGS]
        + [OpaqueFunction(function=_setup)]
    )
