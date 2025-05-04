from launch.actions import (
    RegisterEventHandler,
    TimerAction,
    LogInfo,
    DeclareLaunchArgument,
)

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from gazebo_sim.launch_arguments.common import GazeboCommonArgs
from gazebo_sim.launch_arguments.hunav import HunavArgs
from gazebo_sim.launch.launch_utils import path_to_file_in_pkg

from dataclasses import dataclass
from launch_pal.arg_utils import LaunchArgumentsBase
from launch.substitutions import LaunchConfiguration

# import execute_process
from launch.actions import ExecuteProcess


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    agents_configuration_file: DeclareLaunchArgument = (
        HunavArgs.agents_configuration_file
    )
    config_pkg_name: DeclareLaunchArgument = HunavArgs.config_pkg_name
    world_pkg_name: DeclareLaunchArgument = HunavArgs.world_pkg_name
    use_sim_time: DeclareLaunchArgument = GazeboCommonArgs.use_sim_time
    base_world: DeclareLaunchArgument = HunavArgs.base_world
    use_gazebo_obs: DeclareLaunchArgument = HunavArgs.use_gazebo_obs
    update_rate: DeclareLaunchArgument = HunavArgs.update_rate
    robot_name: DeclareLaunchArgument = HunavArgs.robot_name
    global_frame_to_publish: DeclareLaunchArgument = HunavArgs.global_frame_to_publish
    ignore_models: DeclareLaunchArgument = HunavArgs.ignore_models
    use_navgoal_to_start: DeclareLaunchArgument = HunavArgs.use_navgoal_to_start
    use_collision: DeclareLaunchArgument = HunavArgs.use_collision


def generate_launch_description():

    launch_arguments = LaunchArguments()

    # World generation parameters
    agent_conf_file = path_to_file_in_pkg(
        pkg_name=LaunchConfiguration("config_pkg_name"),
        paths=["config", LaunchConfiguration("agents_configuration_file")],
    )

    # Read the yaml file and load the parameters
    hunav_loader_node = Node(
        package="hunav_agent_manager",
        executable="hunav_loader",
        output="screen",
        parameters=[agent_conf_file],
    )

    # world base file
    world_file = path_to_file_in_pkg(
        pkg_name=LaunchConfiguration("world_pkg_name"),
        paths=["worlds", LaunchConfiguration("base_world")],
    )
    # if desired to spawn goal model in Gazebo

    # the node looks for the base_world file in the directory 'worlds'
    # of the package hunav_gazebo_plugin direclty. So we do not need to
    # indicate the path
    hunav_gazebo_worldgen_node = Node(
        package="hunav_gazebo_wrapper",
        executable="hunav_gazebo_world_generator",
        output="screen",
        parameters=[
            {"base_world": world_file},
            {"use_gazebo_obs": LaunchConfiguration("use_gazebo_obs")},
            {"update_rate": LaunchConfiguration("update_rate")},
            {"robot_name": LaunchConfiguration("robot_name")},
            {"global_frame_to_publish": LaunchConfiguration("global_frame_to_publish")},
            {"use_navgoal_to_start": LaunchConfiguration("use_navgoal_to_start")},
            {"ignore_models": LaunchConfiguration("ignore_models")},
            {"use_collision": LaunchConfiguration("use_collision")},
        ],
    )

    ordered_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_loader_node,
            on_start=[
                LogInfo(
                    msg="HunNavLoader started, launching HuNav_Gazebo_world_generator after 2 seconds..."
                ),
                TimerAction(
                    period=5.0,
                    actions=[hunav_gazebo_worldgen_node],
                ),
            ],
        )
    )

    # Execute a process after the world generation to print the end of the process
    ordered_launch_event_2 = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_gazebo_worldgen_node,
            on_start=TimerAction(
                period=5.0,
                actions=[
                    ExecuteProcess(
                        cmd=["echo", "Hunav world generation finished"],
                        output="screen",
                    )
                ],
            ),
        )
    )

    # hunav_manager node
    hunav_manager_node = Node(
        package="hunav_agent_manager",
        executable="hunav_agent_manager",
        name="hunav_agent_manager",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    ld = LaunchDescription()
    launch_arguments.add_to_launch_description(ld)
    # Generate the world with the agents
    # launch hunav_loader and the WorldGenerator
    # 2 seconds later
    ld.add_action(hunav_loader_node)
    ld.add_action(ordered_launch_event)
    ld.add_action(hunav_manager_node)
    ld.add_action(ordered_launch_event_2)
    return ld
