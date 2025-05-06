from launch.actions import DeclareLaunchArgument
from dataclasses import dataclass


@dataclass(frozen=True, kw_only=True)
class RobotArgs:
    """This class contains a collection of frequently used LaunchArguments for the robot launch file."""

    use_gazebo_controllers: DeclareLaunchArgument = DeclareLaunchArgument(
        "use_gazebo_controllers",
        default_value="false",
        choices=["true", "false"],
        description="Whether to use Gazebo controllers or not",
    )
    use_collision_sensor: DeclareLaunchArgument = DeclareLaunchArgument(
        "use_collision_sensor",
        default_value="true",
        choices=["true", "false"],
        description="Whether to use collision sensor or not",
    )
