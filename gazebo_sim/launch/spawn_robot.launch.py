from dataclasses import dataclass

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare

from launch_pal.include_utils import include_scoped_launch_py_description

from gazebo_sim.launch.launch_params_subs import LaunchArgumentsBaseParam
from gazebo_sim.launch_arguments.common import GazeboCommonArgs
from gazebo_sim.launch_arguments.robot import RobotArgs


# Models whose entire spawn lives in robot_profiles/launch/<model>_spawn.launch.py
# (the profile owns the spawn node, description, control and namespacing). jackal
# and ghost_robot are still spawned inline below for now.
PROFILE_DISPATCHED = {"turtlebot3", "turtlebot2"}

# Contract flags a fleet entry may override per-robot (else the global launch arg
# applies). They map straight onto the spawn contract each profile accepts.
PER_ROBOT_FLAGS = (
    "use_gazebo_controllers",
    "use_collision_sensor",
    "use_lidar_gpu",
)

# Extra per-robot keys forwarded verbatim into a PROFILE_DISPATCHED robot's launch
# (e.g. turtlebot3's sensor toggles). launch rejects launch_arguments the included
# file does not declare, so only list keys the profile(s) actually declare.
PROFILE_PASSTHROUGH = (
    "enable_lidar",
    "enable_camera",
)


def _flag(value):
    """Normalise a per-robot flag to the 'true'/'false' string the profiles and
    xacro expect (YAML decodes booleans, but the contract is string-typed)."""
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBaseParam):
    """Arguments required to spawn a supported robot and its gazebo controllers."""

    robot_name: DeclareLaunchArgument = GazeboCommonArgs.robot_name
    namespace: DeclareLaunchArgument = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="ROS namespace for this robot (empty = no namespace, "
        "single-robot behaviour).",
    )
    robots: DeclareLaunchArgument = DeclareLaunchArgument(
        "robots",
        default_value="",
        description="YAML/JSON-encoded list of robots to spawn, e.g. "
        "\"[{name: r0, model: jackal, spawn: [0,0,0]}]\". When non-empty, the "
        "whole fleet is spawned (one namespaced robot per entry). Empty = spawn "
        "the single robot described by robot_name/x/y/yaw. Each entry may also "
        "carry per-robot overrides of the contract flags (use_gazebo_controllers, "
        "use_collision_sensor, use_lidar_gpu), e.g. \"{name: r1, model: jackal, "
        "spawn: [2,0,0], use_lidar_gpu: false}\"; absent flags inherit the global "
        "launch argument. Profile-specific keys are also forwarded, e.g. "
        "turtlebot3 sensor toggles enable_lidar/enable_camera.",
    )
    x: DeclareLaunchArgument = GazeboCommonArgs.x
    y: DeclareLaunchArgument = GazeboCommonArgs.y
    z: DeclareLaunchArgument = GazeboCommonArgs.z
    yaw: DeclareLaunchArgument = GazeboCommonArgs.yaw
    use_sim_time: DeclareLaunchArgument = GazeboCommonArgs.use_sim_time
    use_gazebo_controllers: DeclareLaunchArgument = RobotArgs.use_gazebo_controllers
    use_collision_sensor: DeclareLaunchArgument = RobotArgs.use_collision_sensor
    use_lidar_gpu: DeclareLaunchArgument = RobotArgs.use_lidar_gpu


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()
    launch_arguments.add_to_launch_description(ld)

    ld.add_action(
        OpaqueFunction(
            function=_spawn,
            args=[
                launch_arguments.launch_configurations_dict(),
            ],
        )
    )

    return ld


def _spawn(context, launch_configurations):
    """Spawn one robot, or a whole fleet when a 'robots' list is provided."""
    robots_arg = LaunchConfiguration("robots").perform(context).strip()
    fleet = yaml.safe_load(robots_arg) if robots_arg else []
    if not isinstance(fleet, list):
        raise ValueError("'robots' must decode to a list of robot mappings")

    use_gazebo_controllers = LaunchConfiguration("use_gazebo_controllers").perform(context)
    use_collision_sensor = LaunchConfiguration("use_collision_sensor").perform(context)
    use_lidar_gpu = LaunchConfiguration("use_lidar_gpu").perform(context)
    z = LaunchConfiguration("z").perform(context)

    if not fleet:
        # Single-robot path. Run the lone robot fully namespaced under its own
        # name so its (now namespaced) plugins, control, EKF and Nav2 all line up.
        robot_name = LaunchConfiguration("robot_name").perform(context)
        lone_ns = LaunchConfiguration("namespace").perform(context).strip("/") or robot_name
        return _spawn_robot_with_controllers(
            context,
            robot_name=robot_name,
            namespace=lone_ns,
            x=LaunchConfiguration("x").perform(context),
            y=LaunchConfiguration("y").perform(context),
            z=z,
            yaw=LaunchConfiguration("yaw").perform(context),
            use_gazebo_controllers=use_gazebo_controllers,
            use_collision_sensor=use_collision_sensor,
            use_lidar_gpu=use_lidar_gpu,
        )

    # Fleet path: build each robot's namespaced spawn group directly (no
    # recursive self-include). _spawn_robot_with_controllers already wraps its
    # actions in PushRosNamespace when a namespace is given.
    global_flags = {
        "use_gazebo_controllers": use_gazebo_controllers,
        "use_collision_sensor": use_collision_sensor,
        "use_lidar_gpu": use_lidar_gpu,
    }
    actions = [LogInfo(msg=f"Spawning fleet of {len(fleet)} robots")]
    for robot in fleet:
        x, y, yaw = robot.get("spawn", [0.0, 0.0, 0.0])
        # Each robot may override the contract flags; otherwise inherit the global.
        per_robot_flags = {
            flag: _flag(robot[flag]) if flag in robot else global_flags[flag]
            for flag in PER_ROBOT_FLAGS
        }
        # Profile-specific per-robot args (e.g. turtlebot3 sensor toggles); only
        # those the entry actually sets are forwarded.
        extra_profile_args = {
            key: _flag(robot[key]) for key in PROFILE_PASSTHROUGH if key in robot
        }
        actions += _spawn_robot_with_controllers(
            context,
            robot_name=str(robot.get("model", "jackal")),
            namespace=str(robot["name"]),
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            extra_profile_args=extra_profile_args,
            **per_robot_flags,
        )
    return actions


def _spawn_robot_with_controllers(
    context,
    robot_name,
    namespace,
    x,
    y,
    z,
    yaw,
    use_gazebo_controllers,
    use_collision_sensor,
    use_lidar_gpu,
    extra_profile_args=None,
):
    namespace = namespace.strip("/")

    # Profile-owned robots: dispatch the whole spawn (node + description + control
    # + namespacing) to robot_profiles/launch/<model>_spawn.launch.py. The profile
    # self-namespaces, so return its include directly. jackal/ghost_robot are still
    # handled inline below; migrating them here is the dispatch refactor (separate).
    if robot_name in PROFILE_DISPATCHED:
        return [
            include_scoped_launch_py_description(
                pkg_name="robot_profiles",
                paths=["launch", f"{robot_name}_spawn.launch.py"],
                launch_arguments={
                    "namespace": namespace,
                    "x": str(x),
                    "y": str(y),
                    "z": str(z),
                    "yaw": str(yaw),
                    "use_gazebo_controllers": use_gazebo_controllers,
                    "use_collision_sensor": use_collision_sensor,
                    "use_lidar_gpu": use_lidar_gpu,
                    **(extra_profile_args or {}),
                },
            )
        ]

    actions = []

    # The Gazebo entity name must be unique. In fleet mode several robots share
    # the same model (robot_name), so use the namespace as the entity name;
    # robot_name still selects the description/control branch below.
    entity_name = namespace if namespace else robot_name

    # include_scoped_launch_py_description isolates launch configs, so the outer
    # PushRosNamespace does NOT reach the included description/control launches.
    # Pass the namespace explicitly (as bringup.launch.py does) so each robot's
    # robot_state_publisher / controllers land under /{namespace}/... instead of
    # clobbering the global /robot_description. Empty namespace -> unchanged.
    ns_kwargs = {"namespace": namespace} if namespace else {}

    controller_manager_name = (
        f"/{namespace}/controller_manager" if namespace else "/controller_manager"
    )

    entity_description_arguments = (
        [
            "-database",
            robot_name,
        ]
        if robot_name == "ghost_robot"
        else ["-topic", "robot_description"]
    )

    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_robot",
        arguments=[
            "-entity",
            entity_name,
        ]
        + (["-robot_namespace", namespace] if namespace else [])
        + entity_description_arguments
        + [
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            str(z),
            "-Y",
            str(yaw),
        ],
        output="screen",
    )
    actions.append(spawn_robot_node)

    if robot_name == "jackal":
        # prefix namespaces the URDF's Gazebo plugins (imu/collision/ground_truth).
        # Lone robot is treated as the first robot, "jackal".
        description_ns = namespace if namespace else "jackal"
        config_jackal_velocity_controller = PathJoinSubstitution(
            [FindPackageShare("jackal_gazebo"), "config", "control.yaml"]
        )
        config_jackal_localization = PathJoinSubstitution(
            [FindPackageShare("jackal_gazebo"), "config", "localization.yaml"]
        )
        config_twist_mux = PathJoinSubstitution(
            [FindPackageShare("jackal_gazebo"), "config", "twist_mux.yaml"]
        )

        robot_description_command = [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("jackal_description"), "urdf", "jackal.urdf.xacro"]
            ),
            " ",
            "use_gazebo_controllers:=",
            use_gazebo_controllers,
            " ",
            "use_collision_sensor:=",
            use_collision_sensor,
            " ",
            "use_lidar_gpu:=",
            use_lidar_gpu,
            " ",
            "gazebo_sim:=True",
            " ",
            "prefix:=",
            description_ns,
            " ",
            "gazebo_controllers:=",
            config_jackal_velocity_controller,
        ]

        description_launch = include_scoped_launch_py_description(
            pkg_name="jackal_description",
            paths=["launch", "description.launch.py"],
            launch_arguments={"robot_description_command": robot_description_command},
            **ns_kwargs,
        )
        actions.append(description_launch)

        jackal_control = include_scoped_launch_py_description(
            pkg_name="jackal_control",
            paths=["launch", "control.launch.py"],
            launch_arguments={
                "robot_description_command": robot_description_command,
                "gazebo_sim": "True",
                "config_jackal_velocity": config_jackal_velocity_controller,
                "config_jackal_localization": config_jackal_localization,
            },
            condition=IfCondition(use_gazebo_controllers),
            **ns_kwargs,
        )
        actions.append(jackal_control)
        teleop_base = include_scoped_launch_py_description(
            pkg_name="jackal_control",
            paths=["launch", "teleop_base.launch.py"],
            launch_arguments={
                "config_twist_mux": config_twist_mux,
            },
            **ns_kwargs,
        )
        actions.append(teleop_base)

        controller_spawners = GroupAction(
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "jackal_velocity_controller",
                        "-c",
                        controller_manager_name,
                    ],
                    output="screen",
                    condition=IfCondition(use_gazebo_controllers),
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "joint_state_broadcaster",
                        "-c",
                        controller_manager_name,
                    ],
                    output="screen",
                    condition=IfCondition(use_gazebo_controllers),
                ),
            ]
        )

        controller_spawners_callback = RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot_node,
                on_exit=[
                    LogInfo(msg="Robot spawned, now spawning controllers..."),
                    controller_spawners,
                ],
            ),
            condition=IfCondition(use_gazebo_controllers),
        )
        stop_jackal_controllers_on_exit = RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot_node,
                on_exit=[
                    ExecuteProcess(
                        cmd=[
                            "ros2 topic pub /stop/cmd_vel geometry_msgs/msg/Twist",
                            '"linear: {}"',
                        ],
                        output="log",
                        shell=True,
                        on_exit=Shutdown(),
                        condition=UnlessCondition(use_gazebo_controllers),
                    ),
                ],
            )
        )
        actions.append(stop_jackal_controllers_on_exit)
        actions.append(controller_spawners_callback)

    if namespace:
        return [
            GroupAction(
                actions=[
                    PushRosNamespace(namespace),
                    SetRemap("/tf", "tf"),
                    SetRemap("/tf_static", "tf_static"),
                ]
                + actions
            )
        ]
    return actions
