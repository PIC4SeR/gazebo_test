from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration

from launch.event_handlers import OnProcessStart, OnProcessIO
from launch_pal.include_utils import include_scoped_launch_py_description
from gazebo_sim.launch.launch_params_subs import LaunchArgumentsBaseParam
from dataclasses import dataclass
from gazebo_sim.launch_arguments.common import GazeboCommonArgs
from gazebo_sim.launch_arguments.hunav import HunavArgs
from gazebo_sim.launch_arguments.robot import RobotArgs

from launch_ros.actions import RosTimer


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
    use_lidar_gpu: DeclareLaunchArgument = RobotArgs.use_lidar_gpu
    use_collision_sensor: DeclareLaunchArgument = RobotArgs.use_collision_sensor
    goal_x: DeclareLaunchArgument = GazeboCommonArgs.goal_x
    goal_y: DeclareLaunchArgument = GazeboCommonArgs.goal_y
    headless: DeclareLaunchArgument = GazeboCommonArgs.headless


def generate_launch_description():

    ld = LaunchDescription()
    # Declare launch arguments
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    # Include the launch description for the Gazebo simulation

    jackal_gazebo = include_scoped_launch_py_description(
        pkg_name="gazebo_sim",
        paths=["launch", "jackal_social.launch.py"],
        launch_arguments=launch_arguments.launch_configurations_dict(),
    )
    ld.add_action(jackal_gazebo)

    return ld
