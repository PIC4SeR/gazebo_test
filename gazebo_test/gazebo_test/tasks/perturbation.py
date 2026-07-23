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

* **Active divergence check.** "Formation stable" is measured, not assumed: a
  background monitor polls each robot's world-frame ground-truth pose from
  Gazebo and fails the episode if the fleet *comes apart* -- i.e. its RMS radius
  about its own centroid grows more than ``spread_tolerance`` metres beyond its
  starting size, and stays there. The centroid itself is free to settle wherever
  the agents leave it; only the robots separating is a failure.

* **Success condition.** Because the swarm starts *at* its target, the
  orchestrator would report convergence almost immediately -- and, worse, stop
  the workers, leaving the fleet uncontrolled. So the experiment launches the
  orchestrator with ``convergence_criterion: hold`` (never converges; workers
  run until we cancel) and the episode instead ends at **steady state**: the
  agents cross once and park (``cyclic_goals: false`` in their agents.yaml), so
  once they and the fleet are all below ``settle_speed`` for ``settle_hold``
  seconds nothing further can happen and the swarm stayed a swarm -> SUCCESS.
  The fleet is already still at t=0, so steady state does not count until the
  agents have actually moved. Timing out without settling means the swarm never
  came back to rest -> FAILURE_TIMEOUT.

ponytail: everything except the hold-point computation, the spread monitor, and
the inverted run-until-timeout wait is inherited from ``FormationTask`` (which in
turn reuses the multi-robot collision machinery).
"""

import asyncio
import math
from pathlib import Path
from typing import List

import yaml

from people_msgs.msg import People

from gazebo_test.tasks.base import register_task
from gazebo_test.tasks.formation import FormationTask
from gazebo_test.utils.common_utils import parse_fleet_yaml
from gazebo_test.utils.evaluation_handler import ExperimentResult


@register_task("perturbation")
class PerturbationTask(FormationTask):
    """Hold the formation's group center steady for the whole episode."""

    # How much the formation may grow (metres of RMS radius about its own
    # centroid, over its starting size) before the episode is failed, and how
    # often we poll ground-truth poses to check. All overridable via top-level
    # keys in the goals/poses YAML.
    DEFAULT_SPREAD_TOLERANCE_M = 0.75
    DEFAULT_SPREAD_POLL_PERIOD_S = 0.5
    # A transient stretch past the tolerance is expected (that *is* the
    # perturbation); only failing to pull back together is divergence.
    DEFAULT_SPREAD_GRACE_S = 10.0
    # Steady state: nothing in the scene -- humans or robots -- moving faster
    # than this, for this long. The agents cross once and stop (their
    # agents.yaml sets ``cyclic_goals: false``), so once they are parked and the
    # swarm has re-converged nothing else can happen; that is the episode.
    # ponytail: a speed threshold, not a full equilibrium test -- raise
    # settle_speed if the sim's idle jitter keeps the fleet "moving".
    DEFAULT_SETTLE_SPEED_MPS = 0.05
    DEFAULT_SETTLE_HOLD_S = 3.0

    def __init__(self, manager) -> None:
        super().__init__(manager)
        self._spread_tolerance = self.DEFAULT_SPREAD_TOLERANCE_M
        self._spread_poll_period = self.DEFAULT_SPREAD_POLL_PERIOD_S
        self._spread_grace = self.DEFAULT_SPREAD_GRACE_S
        self._settle_speed = self.DEFAULT_SETTLE_SPEED_MPS
        self._settle_hold = self.DEFAULT_SETTLE_HOLD_S
        # Max formation spread seen in the current episode (for logging).
        self._max_spread = 0.0
        # Fastest human right now, and whether any of them ever got going: the
        # fleet is *already* still at t=0, so without this gate the episode
        # would settle before the perturbation ever happened.
        self._humans_speed = 0.0
        self._humans_moved = False

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

        # Optional spread knobs from the same YAML.
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f) or {}
        self._spread_tolerance = float(
            data.get("spread_tolerance", self.DEFAULT_SPREAD_TOLERANCE_M)
        )
        self._spread_poll_period = float(
            data.get("spread_poll_period", self.DEFAULT_SPREAD_POLL_PERIOD_S)
        )
        self._spread_grace = float(
            data.get("spread_grace", self.DEFAULT_SPREAD_GRACE_S)
        )
        self._settle_speed = float(
            data.get("settle_speed", self.DEFAULT_SETTLE_SPEED_MPS)
        )
        self._settle_hold = float(data.get("settle_hold", self.DEFAULT_SETTLE_HOLD_S))

        self.manager.get_logger().info(
            f"Loaded perturbation fleet of {len(self.fleet)} robots: "
            f"{[r['name'] for r in self.fleet]}; hold points {self.centroid}; "
            f"spread_tolerance={self._spread_tolerance} m sustained over "
            f"{self._spread_grace} s"
        )
        return tags

    def _default_centroid(self, experiment_tag: str) -> tuple:
        """Group center = mean (x, y) of this episode's start poses."""
        states = self.initial_state_entities[experiment_tag].values()
        xs = [s.pose.position.x for s in states]
        ys = [s.pose.position.y for s in states]
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    async def setup(self) -> None:
        await super().setup()
        # The humans' half of the steady-state test: hunav_agent_manager's
        # ground truth for every agent (not /<ns>/detected_people, which is the
        # robot's FOV-limited sensor -- a human parked out of view would settle
        # the episode early). Needs hunav_loader.publish_people (the default);
        # without it no agent ever "moves" and every episode times out, which
        # the timeout warning reports.
        self.manager.create_subscription(People, "/people", self._on_humans, 10)

    def _on_humans(self, msg: People) -> None:
        # velocity.z carries the angular rate, not a third linear axis.
        self._humans_speed = max(
            (
                math.hypot(person.velocity.x, person.velocity.y)
                for person in msg.people
            ),
            default=0.0,
        )
        if self._humans_speed > self._settle_speed:
            self._humans_moved = True

    @staticmethod
    def _spread(points: List[tuple]) -> float:
        """RMS distance of the robots from their own centroid (formation size)."""
        cx = sum(x for x, _ in points) / len(points)
        cy = sum(y for _, y in points) / len(points)
        return math.sqrt(
            sum((x - cx) ** 2 + (y - cy) ** 2 for x, y in points) / len(points)
        )

    async def _monitor_formation(self) -> ExperimentResult:
        """Poll ground-truth states until the episode is decided.

        Returns FAILURE_DRIFT if the formation comes apart and stays apart, or
        SUCCESS once the scene reaches steady state. Neither is instantaneous:

        * *Divergence* -- the fleet is *expected* to stretch while the agents
          push through, so only a violation sustained for ``spread_grace``
          seconds counts. Where the centroid ends up is deliberately not
          checked -- see the module docstring.
        * *Steady state* -- the agents have crossed and parked, and the swarm
          has re-converged: nothing moving faster than ``settle_speed`` for
          ``settle_hold`` seconds. Nothing can perturb the fleet after that, so
          there is no point burning the rest of the timeout window.

        If neither happens the coroutine keeps polling until :meth:`_drive_swarm`
        cancels it at the timeout.
        """
        manager = self.manager
        names = list(self._robot_names)
        self._max_spread = 0.0
        baseline = None
        over_for = 0.0
        settled_for = 0.0
        while True:
            await asyncio.sleep(self._spread_poll_period)
            try:
                states = await asyncio.gather(
                    *(manager.gazebo_env_handler.get_entity_state(n) for n in names)
                )
            except Exception as exc:  # transient service failure -- keep watching
                manager.get_logger().debug(f"[perturbation] pose poll failed: {exc}")
                continue
            valid = [s for s in states if s is not None]
            if len(valid) < 2:  # a single pose has no formation to speak of
                continue
            spread = self._spread(
                [(s.pose.position.x, s.pose.position.y) for s in valid]
            )
            if baseline is None:
                baseline = spread  # the formation's own starting size
                continue
            self._max_spread = max(self._max_spread, spread)

            over_for = (
                over_for + self._spread_poll_period
                if spread - baseline > self._spread_tolerance
                else 0.0
            )
            if over_for >= self._spread_grace:
                manager.get_logger().warning(
                    f"[perturbation] formation spread {spread:.2f} m vs "
                    f"{baseline:.2f} m at start (> +{self._spread_tolerance} m) "
                    f"for {over_for:.1f} s -- not closing back up"
                )
                return ExperimentResult.FAILURE_DRIFT

            still = (
                self._humans_moved
                # A fleet that flew apart and then stopped is at rest, but it is
                # not a formation: steady state means settled *and* intact.
                and over_for == 0.0
                and self._humans_speed <= self._settle_speed
                and all(
                    math.hypot(s.twist.linear.x, s.twist.linear.y)
                    <= self._settle_speed
                    for s in valid
                )
            )
            settled_for = settled_for + self._spread_poll_period if still else 0.0
            if settled_for >= self._settle_hold:
                manager.get_logger().info(
                    f"[perturbation] steady state: agents parked and fleet still "
                    f"for {settled_for:.1f} s (spread {spread:.2f} m vs "
                    f"{baseline:.2f} m at start)"
                )
                return ExperimentResult.SUCCESS

    async def _drive_swarm(self, goal_pose, experiment_tag: str) -> ExperimentResult:
        """Send the hold goal, then run until the scene settles or breaks.

        Unlike the formation base we ignore the orchestrator's convergence (it
        fires ~immediately -- the swarm starts at its goal). The episode ends on
        the first of: any robot collision, the formation coming apart
        (FAILURE_DRIFT), the scene reaching steady state (SUCCESS), or the
        timeout -- which now means the fleet never settled, i.e. FAILURE_TIMEOUT.
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

        # Race collisions and the formation monitor against the episode timeout.
        waiters = {}
        for ns in self._robot_names:
            waiters[asyncio.ensure_future(self._collision_event[ns].wait())] = ns
        monitor_task = asyncio.ensure_future(self._monitor_formation())
        waiters[monitor_task] = None  # None ns == the watcher, not a robot

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

        if monitor_task in done and not monitor_task.cancelled():
            # The monitor decided: steady state (SUCCESS) or divergence.
            return monitor_task.result()

        # Timeout with no collision and no steady state: the agents parked long
        # ago (they cross once) and the swarm is still moving, or they never got
        # going at all. Either way the episode never settled.
        manager.get_logger().warning(
            f"Episode '{experiment_tag}' never reached steady state "
            f"(max spread {self._max_spread:.2f} m, agents moved: "
            f"{self._humans_moved})"
        )
        return ExperimentResult.FAILURE_TIMEOUT
