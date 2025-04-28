from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration

from launch.event_handlers import OnProcessStart, OnProcessIO
from launch_pal.include_utils import include_scoped_launch_py_description
from launch_pal.arg_utils import LaunchArgumentsBase
from dataclasses import dataclass
from gazebo_sim.launch_arguments.common import GazeboCommonArgs
from gazebo_sim.launch_arguments.hunav import HunavArgs
from gazebo_sim.launch_arguments.robot import RobotArgs
from gazebo_sim.launch.launch_utils import spawn_goal_entity

from launch_ros.actions import RosTimer


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
    gzpose_x: DeclareLaunchArgument = GazeboCommonArgs.x
    gzpose_y: DeclareLaunchArgument = GazeboCommonArgs.y
    gzpose_Y: DeclareLaunchArgument = GazeboCommonArgs.yaw
    use_navgoal_to_start: DeclareLaunchArgument = HunavArgs.use_navgoal_to_start
    use_gazebo_controllers: DeclareLaunchArgument = RobotArgs.use_gazebo_controllers
    goal_x: DeclareLaunchArgument = GazeboCommonArgs.goal_x
    goal_y: DeclareLaunchArgument = GazeboCommonArgs.goal_y


def generate_launch_description():

    ld = LaunchDescription()
    # Declare launch arguments
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)
    spawn_goal = spawn_goal_entity(
        x=LaunchConfiguration("goal_x"),
        y=LaunchConfiguration("goal_y"),
    )

    # Include the launch description for the Gazebo simulation

    jackal_gazebo = include_scoped_launch_py_description(
        pkg_name="gazebo_sim",
        paths=["launch", "jackal_social.launch.py"],
        launch_arguments={
            "config_pkg_name": LaunchConfiguration("config_pkg_name"),
            "world_pkg_name": LaunchConfiguration("world_pkg_name"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "base_world": LaunchConfiguration("base_world"),
            "use_gazebo_obs": LaunchConfiguration("use_gazebo_obs"),
            "update_rate": LaunchConfiguration("update_rate"),
            "robot_name": LaunchConfiguration("robot_name"),
            "global_frame_to_publish": LaunchConfiguration("global_frame_to_publish"),
            "ignore_models": LaunchConfiguration("ignore_models"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "yaw": LaunchConfiguration("yaw"),
            "use_gazebo_controllers": LaunchConfiguration("use_gazebo_controllers"),
            "use_navgoal_to_start": LaunchConfiguration("use_navgoal_to_start"),
        },
    )
    ld.add_action(jackal_gazebo)
    ld.add_action(spawn_goal)
    # Add the event handler to spawn the goal entity when the Gazebo process starts
    ld.add_action(
        RegisterEventHandler(
            # event_handler=OnProcessStart(
            #     target_action=jackal_gazebo,
            #     on_start=[
            #         LogInfo(msg="Gazebo process started, spawning goal entity..."),
            #         RosTimer(
            #             period=0.1,
            #             actions=[spawn_goal],
            #         ),
            #     ],
            # )
            OnProcessIO(
                on_stdout=lambda event: (
                    [
                        RosTimer(
                            period=0.1,
                            actions=[spawn_goal],
                        ),
                        LogInfo(msg="Gazebo process started, spawning goal entity..."),
                    ]
                    if b"Gazebo started" in event.text
                    else []
                ),
            )
        )
    )

    return ld
