from launch.actions import DeclareLaunchArgument
from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory
import os


@dataclass(frozen=True, kw_only=True)
class ExperimentManagerArgs:
    """This class contains a collection of frequently used LaunchArguments for the experiment manager launch file."""

    algorithm_name: DeclareLaunchArgument = DeclareLaunchArgument(
        "algorithm_name",
        default_value="",
        description="Specify the name of the algorithm to be used",
    )
    base_path: DeclareLaunchArgument = DeclareLaunchArgument(
        "base_path",
        default_value="",
        description="Base path for the experiment manager configuration files",
    )
    timeout_duration: DeclareLaunchArgument = DeclareLaunchArgument(
        "timeout_duration",
        default_value="",
        description="Timeout duration for the experiment manager",
    )
    use_recorder: DeclareLaunchArgument = DeclareLaunchArgument(
        "use_recorder",
        choices=["true", "false"],
        default_value="false",
        description="Specify whether to use the bag recorder",
    )
    repetitions: DeclareLaunchArgument = DeclareLaunchArgument(
        "repetitions",
        default_value="",
        description="Number of repetitions for the experiment",
    )
    goals_and_poses_file: DeclareLaunchArgument = DeclareLaunchArgument(
        "goals_and_poses_file",
        default_value="",
        description="Path to the YAML file containing goals and poses",
    )
