from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction

from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart, OnProcessExit, OnProcessIO
from launch.events.process import ProcessExited
from launch_pal.include_utils import include_scoped_launch_py_description
from gazebo_sim.launch.launch_params_subs import LaunchArgumentsBaseParam
from dataclasses import dataclass
from gazebo_sim.launch_arguments.common import GazeboCommonArgs
from gazebo_sim.launch_arguments.hunav import HunavArgs
from gazebo_sim.launch_arguments.robot import RobotArgs
from gazebo_sim.launch.launch_utils import path_to_file_in_pkg, map_to_odom_identity


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBaseParam):
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
    x: DeclareLaunchArgument = GazeboCommonArgs.x
    y: DeclareLaunchArgument = GazeboCommonArgs.y
    yaw: DeclareLaunchArgument = GazeboCommonArgs.yaw
    use_navgoal_to_start: DeclareLaunchArgument = HunavArgs.use_navgoal_to_start
    use_collision: DeclareLaunchArgument = HunavArgs.use_collision
    use_gazebo_controllers: DeclareLaunchArgument = RobotArgs.use_gazebo_controllers
    use_collision_sensor: DeclareLaunchArgument = RobotArgs.use_collision_sensor
    use_lidar_gpu: DeclareLaunchArgument = RobotArgs.use_lidar_gpu
    headless: DeclareLaunchArgument = GazeboCommonArgs.headless


def launch_jackal_gazebo_sim(context, *args, **kwargs):

    return [
        include_scoped_launch_py_description(
            pkg_name="jackal_gazebo",
            paths=["launch", "gazebo.launch.py"],
            launch_arguments={
                "world_path": path_to_file_in_pkg(
                    pkg_name=LaunchConfiguration("world_pkg_name").perform(context),
                    paths=["worlds", "generatedWorld.world"],
                ),
            }
            | LaunchArguments().launch_configurations_dict(),
        )
    ]


def generate_launch_description():

    # World generation parameters

    hunav_world_generation = include_scoped_launch_py_description(
        pkg_name="gazebo_sim",
        paths=["launch", "hunav_generation.launch.py"],
        launch_arguments=LaunchArguments().launch_configurations_dict(),
    )

    # Then, launch the generated world in Gazebo
    # the world generator will create this world
    # in this path

    launch_simulation = RegisterEventHandler(
        OnProcessIO(
            on_stdout=lambda event: (
                [
                    LogInfo(msg="Gazebo starting..."),
                    OpaqueFunction(function=launch_jackal_gazebo_sim),
                    LogInfo(msg="Gazebo started"),
                ]
                if b"generation finished" in event.text
                else []
            ),
        )
    )

    ld = LaunchDescription()
    # Declare the launch arguments
    launch_arguments = LaunchArguments()
    launch_arguments.add_to_launch_description(ld)
    # Generate the world with the agents
    ld.add_action(hunav_world_generation)
    # launch the simulation when the world generation is finished
    ld.add_action(launch_simulation)
    # static transform publisher
    ld.add_action(
        map_to_odom_identity(use_sim_time=LaunchConfiguration("use_sim_time"))
    )
    return ld
