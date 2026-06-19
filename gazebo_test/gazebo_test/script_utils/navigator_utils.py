"""Gazebo-flavoured wrapper around the shared :mod:`nav2_profiles` rendering helper.

The Nav2 navigator/controller/planner/costmap templates live in the
simulator-neutral ``nav2_profiles`` package (shared with ``real_test``). This
module re-exports that helper and renders the templates with the
Gazebo/Jackal-specific values the retired ``gazebo_navigators`` package used to
hard-code:

* ``odom_topic`` — the Jackal Gazebo sim publishes ground-truth odometry on
  ``jackal/ground_truth`` (it is not remapped to ``/odom`` at launch).
* ``bt_xml_pkg`` — ``gazebo_test`` ships the custom behaviour trees under
  ``behavior_trees/``.

These defaults are overridable per call via ``render_vars`` (e.g. values parsed
from an experiment's ``robot_config_file``), so a different robot/sim can supply
its own odometry topic and behaviour-tree package without code changes.
"""

import pathlib
from typing import Dict, Optional

from nav2_profiles.navigator_utils import (  # noqa: F401  (re-exported)
    get_navigator_additional_launch_files,
    get_navigator_config,
    get_navigator_navigation_launch,
    get_navigator_yaml as _get_navigator_yaml,
    get_template_search_paths,
    list_available_navigators,
    resources,
)

# Defaults the legacy gazebo_navigators templates baked in for the Jackal sim.
GAZEBO_RENDER_DEFAULTS: Dict[str, str] = {
    "odom_topic": "jackal/ground_truth",
    "bt_xml_pkg": "gazebo_test",
}


def get_navigator_yaml(
    navigator: str, render_vars: Optional[Dict] = None
) -> pathlib.Path:
    """Render a navigator config using the Gazebo/Jackal defaults.

    Args:
        navigator: Name of the registered navigator profile.
        render_vars: Optional overrides merged on top of
            :data:`GAZEBO_RENDER_DEFAULTS` (e.g. from a ``robot_config_file``).
    """
    merged = dict(GAZEBO_RENDER_DEFAULTS)
    if render_vars:
        merged.update(render_vars)
    return _get_navigator_yaml(navigator, render_vars=merged)
