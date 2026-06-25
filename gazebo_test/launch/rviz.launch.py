import os

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import ReplaceString

from ament_index_python.packages import get_package_share_directory


def _launch_rviz(context, *args, **kwargs):
    # View one robot's stack. The namespace is the first entry of the 'robots'
    # fleet list (consistent with spawn/bringup), or the explicit 'namespace'
    # arg, or empty for the original single-robot view.
    robots_arg = LaunchConfiguration("robots").perform(context).strip()
    fleet = yaml.safe_load(robots_arg) if robots_arg else []
    if fleet:
        namespace = str(fleet[0]["name"]).strip("/")
    else:
        namespace = LaunchConfiguration("namespace").perform(context).strip("/")

    # The config carries <robot_namespace> tokens; fill them so absolute stack
    # topics resolve under the namespace. Empty namespace -> original config.
    prefix = f"/{namespace}" if namespace else ""
    rviz_config = ReplaceString(
        source_file=LaunchConfiguration("rviz_config_file"),
        replacements={"<robot_namespace>": prefix},
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        # Subscribe to the robot's namespaced TF when namespaced.
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    if namespace:
        return [GroupAction([PushRosNamespace(namespace), rviz_node])]
    return [rviz_node]


def generate_launch_description():
    this_pkg_dir = get_package_share_directory("gazebo_test")

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "robots",
            default_value="",
            description="YAML/JSON-encoded fleet list; RViz follows the first "
            "robot. Empty = use 'namespace' (or the single-robot view).",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Robot namespace to view when 'robots' is not given. "
            "Empty = the original un-namespaced single-robot view.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=os.path.join(this_pkg_dir, "rviz", "jackal.rviz"),
            description="Full path to the RViz config file to use",
        )
    )
    ld.add_action(OpaqueFunction(function=_launch_rviz))
    return ld
