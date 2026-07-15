# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString
from dataclasses import dataclass
from gazebo_test.launch_arguments.navigation import NavigationArgs
from gazebo_sim.launch_arguments.common import GazeboCommonArgs
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.include_utils import include_scoped_launch_py_description


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    """This class contains a collection of frequently used LaunchArguments for the navigation launch file."""

    use_sim_time: DeclareLaunchArgument = GazeboCommonArgs.use_sim_time
    use_namespace: DeclareLaunchArgument = NavigationArgs.use_namespace
    namespace: DeclareLaunchArgument = NavigationArgs.namespace
    map: DeclareLaunchArgument = NavigationArgs.map
    params_file: DeclareLaunchArgument = NavigationArgs.params_file
    autostart: DeclareLaunchArgument = NavigationArgs.autostart
    use_composition: DeclareLaunchArgument = NavigationArgs.use_composition
    use_respawn: DeclareLaunchArgument = NavigationArgs.use_respawn
    log_level: DeclareLaunchArgument = NavigationArgs.log_level
    robots: DeclareLaunchArgument = DeclareLaunchArgument(
        "robots",
        default_value="",
        description="YAML/JSON-encoded list of robots, e.g. "
        "\"[{name: r0, nav2_params: r0.yaml}]\". When non-empty, bring up one "
        "namespaced Nav2 stack per robot (each may declare its own "
        "'nav2_params'). Empty = bring up a single stack under 'namespace'.",
    )


def _single_bringup_group(context):
    """The single-stack Nav2 bringup (one namespace)."""
    namespace = LaunchConfiguration("namespace")
    map = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map}

    # The obstacle layer's scan topic must be absolute: Nav2 resolves a relative
    # name against the costmap's own nested namespace (/<ns>/local_costmap), not
    # the robot's. Templates write '<robot_namespace>/front/scan'; fill it in here,
    # eagerly, because the scoped include below is a fresh launch scope.
    ns = namespace.perform(context).strip("/")
    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": f"/{ns}" if ns else ""},
    ).perform(context)

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            Node(
                condition=IfCondition(use_composition),
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params, {"autostart": autostart}],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                emulate_tty=True,
                output="screen",
                namespace=namespace,
            ),
            include_scoped_launch_py_description(
                pkg_name="gazebo_test",
                paths=["launch", "navigation.launch.py"],
                launch_arguments={
                    "namespace": namespace,
                    "map": map,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                },
                namespace=namespace,  # type: ignore
            ),
        ]
    )
    return bringup_cmd_group


def _bringup(context, *args, **kwargs):
    """Bring up one Nav2 stack, or one per robot when a 'robots' list is given."""
    robots_arg = LaunchConfiguration("robots").perform(context).strip()
    fleet = yaml.safe_load(robots_arg) if robots_arg else []
    if not isinstance(fleet, list):
        raise ValueError("'robots' must decode to a list of robot mappings")

    if not fleet:
        return [_single_bringup_group(context)]

    # Fleet path: re-enter this launch file once per namespaced robot. Each child
    # gets robots="" so it takes the single-stack path.
    default_params = LaunchConfiguration("params_file").perform(context)
    map_yaml = LaunchConfiguration("map").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    actions = [LogInfo(msg=f"Bringing up Nav2 for {len(fleet)} robots")]
    for robot in fleet:
        actions.append(
            include_scoped_launch_py_description(
                pkg_name="gazebo_test",
                paths=["launch", "bringup.launch.py"],
                launch_arguments={
                    "namespace": str(robot["name"]),
                    "use_namespace": "true",
                    "map": map_yaml,
                    "params_file": str(robot.get("nav2_params") or default_params),
                    "use_sim_time": use_sim_time,
                    "robots": "",
                },
            )
        )
    return actions


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")
    )
    LaunchArguments().add_to_launch_description(ld)
    ld.add_action(OpaqueFunction(function=_bringup))
    return ld
