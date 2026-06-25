"""Experiment task interface and registry.

An :class:`ExperimentTask` owns the *per-episode behaviour* of an experiment --
which entities exist, what actors/navigators drive them, how an episode is
reset and run, and what counts as success. The
:class:`~gazebo_test.experiment_manager.ExperimentManager` owns the
*infrastructure* (job scheduling, checkpoints, watchdog, results) and delegates
the behaviour to whichever task is selected by the ``task`` parameter.

Add a new task by subclassing :class:`ExperimentTask` and decorating it with
``@register_task("name")``; it then becomes selectable via ``task:=name``.
"""

from typing import TYPE_CHECKING, Callable, Dict, List, Type

from gazebo_test.utils.evaluation_handler import ExperimentResult

if TYPE_CHECKING:
    from pathlib import Path
    from gazebo_test.experiment_manager import ExperimentManager


class ExperimentTask:
    """Per-episode behaviour for one experiment type."""

    def __init__(self, manager: "ExperimentManager") -> None:
        # ``manager`` is the rclpy Node; tasks read shared infra from it
        # (navigation_backend, gazebo_env_handler, evaluation_handler, ...).
        self.manager = manager

    def load_entities(self, yaml_path: "Path") -> List[str]:
        """Parse the goals/poses YAML and return the list of episode tags."""
        raise NotImplementedError

    def watchdog_required_nodes(self) -> List[str]:
        """Nodes whose absence the watchdog treats as an unhealthy stack."""
        raise NotImplementedError

    async def setup(self) -> None:
        """Build actors and bring up their navigation. Called once on init."""
        raise NotImplementedError

    async def run_episode(self, experiment_tag: str, run_id: int) -> ExperimentResult:
        """Reset, run, and evaluate a single episode."""
        raise NotImplementedError

    async def cancel(self) -> None:
        """Interrupt the in-flight episode (watchdog recovery). Optional."""
        return

    async def shutdown(self) -> None:
        """Cleanup after all episodes complete. Optional."""
        return


TASKS: Dict[str, Type[ExperimentTask]] = {}


def register_task(name: str) -> Callable[[Type[ExperimentTask]], Type[ExperimentTask]]:
    def decorator(cls: Type[ExperimentTask]) -> Type[ExperimentTask]:
        TASKS[name] = cls
        return cls

    return decorator
