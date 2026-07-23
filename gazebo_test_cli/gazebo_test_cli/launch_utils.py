"""ROS-free helpers for experiment launch commands."""

import shlex
from typing import Optional


def swarm_algorithm_name(launch_command: Optional[str]) -> Optional[str]:
    """Infer a result label from swarm-control profile launch arguments."""
    if not launch_command:
        return None
    try:
        parts = shlex.split(launch_command)
    except ValueError:
        return None
    if len(parts) < 2 or parts[0] != "swarm_control":
        return None
    arguments = dict(
        token.split(":=", 1) for token in parts[2:] if ":=" in token
    )
    return (
        f"{arguments.get('profile', 'combined')}_"
        f"{arguments.get('memory_profile', 'normalized')}"
    )
