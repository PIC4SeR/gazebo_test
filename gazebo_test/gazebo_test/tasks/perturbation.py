"""Formation *station-keeping* task (a perturbation variant of ``formation``).

The formation task steers the swarm toward a distant target centroid and
succeeds the moment ``swarm_control``'s orchestrator declares convergence. The
perturbation task inverts that goal: the fleet already starts *in formation* and
the swarm-level goal is its **own group center** -- the swarm must hold that
centroid steady while HuNav agents push through/around it, all the way to the
end of the episode.

That flips three things relative to :class:`FormationTask`:

* **Hold point.** The per-episode target is the group center, so it need not be
  declared in the YAML: if ``centroid`` is omitted for an episode we compute it
  as the mean of that episode's start poses (the formation's own centroid).
  An explicit ``centroid`` still wins, in case you want the swarm to hold a
  point offset from where it spawned.

* **Active drift check.** "Group center stable" is measured, not assumed: a
  background monitor polls each robot's world-frame ground-truth pose from
  Gazebo, computes the live centroid, and fails the episode if it strays more
  than ``drift_tolerance`` metres from the hold point. This is the core metric
  -- it directly tests whether ``swarm_control`` kept the center put under the
  agents' perturbation.

* **Success condition.** Because the swarm starts *at* its target, the
  orchestrator would report convergence almost immediately -- and, worse, stop
  the workers, leaving the fleet uncontrolled. So the experiment launches the
  orchestrator with ``convergence_criterion: hold`` (never converges; workers
  run until we cancel), the episode runs for the full timeout window, and
  *reaching the end without a collision or a drift violation* is SUCCESS (the
  swarm held station).

ponytail: everything except the hold-point computation, the drift monitor, and
the inverted run-until-timeout wait is inherited from ``FormationTask`` (which in
turn reuses the multi-robot collision machinery).
"""

import asyncio
import math
from pathlib import Path
from typing import List

import yaml

from gazebo_test.tasks.base import register_task
from gazebo_test.tasks.formation import FormationTask
from gazebo_test.utils.common_utils import parse_fleet_yaml
from gazebo_test.utils.evaluation_handler import ExperimentResult


@register_task("perturbation")
class PerturbationTask(FormationTask):
    """Hold the formation's group center steady for the whole episode."""

    # How far the group center may drift from the hold point before the episode
    # is failed, and how often we poll ground-truth poses to check. Both are
    # overridable via top-level keys in the goals/poses YAML.
    DEFAULT_DRIFT_TOLERANCE_M = 0.75
    DEFAULT_DRIFT_POLL_PERIOD_S = 0.5

    def __init__(self, manager) -> None:
        super().__init__(manager)
        self._drift_tolerance = self.DEFAULT_DRIFT_TOLERANCE_M
        self._drift_poll_period = self.DEFAULT_DRIFT_POLL_PERIOD_S
        # Max centroid drift seen in the current episode (for logging).
        self._max_drift = 0.0

    def load_entities(self, yaml_path: Path) -> List[str]:
        # Same parse as the formation task, but the centroid is the *hold point*
        # and is optional: default it to the mean of each episode's start poses
        # (the formation's own group center) when the YAML omits it.
        parsed = parse_fleet_yaml(yaml_path)
        self.fleet = parsed["fleet"]
        self.initial_state_entities = parsed["initial_state_entities"]
        self.goal_entities = parsed["goal_entities"]
        self.centroid = dict(parsed["centroid"])
        tags = list(self.initial_state_entities.keys())
        for tag in tags:
            self.centroid.setdefault(tag, self._default_centroid(tag))

        # Optional drift knobs from the same YAML.
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f) or {}
        self._drift_tolerance = float(
            data.get("drift_tolerance", self.DEFAULT_DRIFT_TOLERANCE_M)
        )
        self._drift_poll_period = float(
            data.get("drift_poll_period", self.DEFAULT_DRIFT_POLL_PERIOD_S)
        )

        self.manager.get_logger().info(
            f"Loaded perturbation fleet of {len(self.fleet)} robots: "
            f"{[r['name'] for r in self.fleet]}; hold points {self.centroid}; "
            f"drift_tolerance={self._drift_tolerance} m"
        )
        return tags

    def _default_centroid(self, experiment_tag: str) -> tuple:
        """Group center = mean (x, y) of this episode's start poses."""
        states = self.initial_state_entities[experiment_tag].values()
        xs = [s.pose.position.x for s in states]
        ys = [s.pose.position.y for s in states]
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    async def _monitor_drift(self, experiment_tag: str) -> float:
        """Poll ground-truth poses until the group center drifts too far.

        Returns the offending drift distance once it exceeds the tolerance. If
        the center holds, this coroutine simply keeps polling until it is
        cancelled by the timeout in :meth:`_drive_swarm`.
        """
        manager = self.manager
        hx, hy = self.centroid[experiment_tag]
        names = list(self._robot_names)
        self._max_drift = 0.0
        while True:
            await asyncio.sleep(self._drift_poll_period)
            try:
                states = await asyncio.gather(
                    *(manager.gazebo_env_handler.get_entity_state(n) for n in names)
                )
            except Exception as exc:  # transient service failure -- keep watching
                manager.get_logger().debug(f"[perturbation] pose poll failed: {exc}")
                continue
            valid = [s for s in states if s is not None]
            if not valid:
                continue
            cx = sum(s.pose.position.x for s in valid) / len(valid)
            cy = sum(s.pose.position.y for s in valid) / len(valid)
            drift = math.hypot(cx - hx, cy - hy)
            self._max_drift = max(self._max_drift, drift)
            if drift > self._drift_tolerance:
                manager.get_logger().warning(
                    f"[perturbation] group center drifted {drift:.2f} m "
                    f"(> {self._drift_tolerance} m) from hold point {(hx, hy)}"
                )
                return drift

    async def _drive_swarm(self, goal_pose, experiment_tag: str) -> ExperimentResult:
        """Send the hold goal, then keep station until the episode times out.

        Unlike the formation base -- which returns as soon as the orchestrator
        reports convergence -- we ignore convergence and run the full timeout
        window. The episode fails on the first of: any robot collision, or the
        group center drifting past ``drift_tolerance``. Surviving the window with
        the center held is SUCCESS.
        """
        manager = self.manager
        nav = self._swarm_nav.navigator  # BasicNavigator (action backend)
        nav.clearEvents()
        accepted = await nav.goToPose(goal_pose)
        if not accepted:
            manager.get_logger().warning(
                "[perturbation] orchestrator rejected the hold goal"
            )
            return ExperimentResult.FAILURE_NAVIGATION

        # Race collisions and the drift monitor against the episode timeout.
        waiters = {}
        for ns in self._robot_names:
            waiters[asyncio.ensure_future(self._collision_event[ns].wait())] = ns
        drift_task = asyncio.ensure_future(self._monitor_drift(experiment_tag))
        waiters[drift_task] = None  # None ns == the drift watcher, not a robot

        done, pending = await asyncio.wait(
            set(waiters),
            timeout=manager.evaluation_handler.timeout_duration,
            return_when=asyncio.FIRST_COMPLETED,
        )
        for task in pending:
            task.cancel()

        collisions = [
            self._collision_result[ns]
            for task, ns in waiters.items()
            if ns is not None and task in done and self._collision_result[ns] is not None
        ]
        if collisions:
            result = self._aggregate(collisions)
            manager.get_logger().warning(f"[perturbation] {result}")
            return result

        if drift_task in done and not drift_task.cancelled():
            # The monitor returned -> the group center drifted out of tolerance.
            return ExperimentResult.FAILURE_DRIFT

        # Timeout reached with no collision and the center in tolerance: held.
        manager.get_logger().info(
            f"Episode '{experiment_tag}' held formation center for the full window "
            f"(max drift {self._max_drift:.2f} m)"
        )
        return ExperimentResult.SUCCESS
