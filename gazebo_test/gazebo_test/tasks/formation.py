"""Formation-keeping multi-robot task (driven by ``swarm_control``).

Unlike the multi-robot go-to-pose task -- where each robot runs its own Nav2
stack toward its own goal -- formation-keeping is delegated to the
``swarm_control`` package. Its hybrid orchestrator exposes a single swarm-level
``navigate_to_pose`` action whose goal is the **target centroid** (the point
the whole formation should translate toward); the per-robot control law
(formation + target attraction + asymmetric Gaussian human avoidance) holds the
shape along the way and the orchestrator declares global convergence.

So this task only orchestrates the *experiment*: reset the fleet onto its start
formation, spawn the centroid goal box, send one goal to the orchestrator, and
race that against any robot's collision sensor / the timeout. The fleet, start
poses and per-episode ``centroid`` come from the multi-robot goals/poses YAML
(see :func:`parse_fleet_yaml`); ``swarm_control`` (workers + orchestrator) is
launched via the experiment's ``navigation_launch`` override pointing at
``swarm_control hybrid.launch.py``.

ponytail: collision detection and result aggregation are inherited wholesale
from the multi-robot task; only the *single* swarm action replaces its N
per-robot Nav2 navigators.
"""

import asyncio
from pathlib import Path
from typing import List, Optional

import rclpy.duration

from gazebo_collision_msgs.msg import Collision

from gazebo_test.tasks.base import register_task
from gazebo_test.tasks.go_to_pose_multirobot import MultiRobotGoToPoseTask
from gazebo_test.utils.basic_navigator import TaskResult
from gazebo_test.utils.common_utils import (
    _entity_from_pose,
    get_posestamped_from_entity,
    parse_fleet_yaml,
)
from gazebo_test.utils.evaluation_handler import ExperimentResult
from gazebo_test.utils.navigation_handler import NavigationHandler


@register_task("formation")
class FormationTask(MultiRobotGoToPoseTask):
    """Steer the whole fleet to a centroid while ``swarm_control`` holds shape."""

    def __init__(self, manager) -> None:
        super().__init__(manager)
        # episode -> (x, y) target centroid for the swarm.
        self.centroid: dict = {}
        # The hybrid orchestrator publishes /navigate_to_pose at the root, so the
        # swarm navigator is un-namespaced.
        self._orchestrator_ns: str = ""
        self._swarm_nav: Optional[NavigationHandler] = None

    def load_entities(self, yaml_path: Path) -> List[str]:
        parsed = parse_fleet_yaml(yaml_path)
        self.fleet = parsed["fleet"]
        self.initial_state_entities = parsed["initial_state_entities"]
        self.goal_entities = parsed["goal_entities"]
        self.centroid = parsed["centroid"]
        tags = list(self.initial_state_entities.keys())
        missing = [tag for tag in tags if tag not in self.centroid]
        if missing:
            raise ValueError(
                "Formation task needs a per-episode 'centroid: [x, y]' target; "
                f"missing for episodes {missing}."
            )
        self.manager.get_logger().info(
            f"Loaded formation fleet of {len(self.fleet)} robots: "
            f"{[r['name'] for r in self.fleet]}"
        )
        return tags

    def watchdog_required_nodes(self) -> List[str]:
        # The fleet is driven by swarm_control, not Nav2.
        nodes = ["/gazebo", "/swarm_orchestrator"]
        if self.manager.use_evaluator:
            nodes.append("/hunav_evaluator_node")
        nodes.extend(["/hunav_agent_manager", "/hunav_plugin"])
        return nodes

    async def setup(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._robot_names = {robot["name"] for robot in self.fleet}
        # Per-robot collision detection (inherited callback machinery).
        for robot in self.fleet:
            ns = robot["name"]
            self._collision_event[ns] = asyncio.Event()
            self._collision_result[ns] = None
            self.manager.create_subscription(
                Collision,
                f"/{ns}/collision",
                self._make_collision_callback(ns),
                10,
            )
        if self.manager.use_recorder or self.manager.use_evaluator:
            self.manager.get_logger().warning(
                "Bag recording and HuNav evaluation are not supported in "
                "formation mode yet; ignoring them for this run."
            )
        # One swarm-level action client to the hybrid orchestrator. The "action"
        # backend skips Nav2 lifecycle (the orchestrator is a plain action
        # server, not a lifecycle stack).
        self._swarm_nav = NavigationHandler(
            node=self.manager,
            backend="action",
            namespace=self._orchestrator_ns,
        )
        await self._swarm_nav.initialize_navigation()

    async def run_episode(
        self, experiment_tag: str, run_id: int
    ) -> ExperimentResult:
        manager = self.manager
        initial_states = self.initial_state_entities[experiment_tag]
        target = self.centroid[experiment_tag]
        goal_entity = _entity_from_pose("goal_box", target[0], target[1], 0.0, z=0.0)

        await manager.gazebo_env_handler.reset_environment_for_experiment_multi(
            entities=list(initial_states.values()),
            goal_entities=[goal_entity],
            goal_xml=manager.goal_box_xml,
        )
        await self._swarm_nav.reset_navigation()

        if manager.wait_before_start:
            wait_end = manager.get_clock().now() + rclpy.duration.Duration(
                seconds=manager.wait_before_start
            )
            while manager.get_clock().now() < wait_end:
                await asyncio.sleep(0.1)

        # Arm collision detection: ignore stale messages, clear per-episode state.
        self._episode_start = manager.get_clock().now()
        for ns in self._robot_names:
            self._collision_event[ns].clear()
            self._collision_result[ns] = None

        manager.get_logger().debug(
            f"Steering swarm of {len(self._robot_names)} robots to centroid {target}"
        )
        result = await self._drive_swarm(
            get_posestamped_from_entity(goal_entity, "map"), experiment_tag
        )
        await self.cancel()
        manager.get_logger().info(
            f"Episode '{experiment_tag}' run {run_id} result: {result}"
        )
        return result

    async def _drive_swarm(
        self, goal_pose, experiment_tag: str
    ) -> ExperimentResult:
        manager = self.manager
        nav = self._swarm_nav.navigator  # BasicNavigator (action backend)
        nav.clearEvents()
        accepted = await nav.goToPose(goal_pose)
        if not accepted:
            manager.get_logger().warning("[swarm] orchestrator rejected the goal")
            return ExperimentResult.FAILURE_NAVIGATION

        # Whichever happens first: the swarm converges, any robot collides, or
        # the episode times out.
        waiters = {asyncio.ensure_future(nav.go_to_pose_event.wait()): None}
        for ns in self._robot_names:
            waiters[asyncio.ensure_future(self._collision_event[ns].wait())] = ns
        done, pending = await asyncio.wait(
            set(waiters),
            timeout=manager.evaluation_handler.timeout_duration,
            return_when=asyncio.FIRST_COMPLETED,
        )
        for task in pending:
            task.cancel()

        if not done:
            manager.get_logger().warning(
                f"Episode '{experiment_tag}' timed out before the swarm converged"
            )
            return ExperimentResult.FAILURE_TIMEOUT

        collisions = [
            self._collision_result[ns]
            for task, ns in waiters.items()
            if ns is not None and task in done and self._collision_result[ns] is not None
        ]
        if collisions:
            result = self._aggregate(collisions)
            manager.get_logger().warning(f"[swarm] {result}")
            return result
        if nav.go_to_pose_status == TaskResult.SUCCEEDED:
            return ExperimentResult.SUCCESS
        manager.get_logger().warning(
            f"[swarm] navigation ended: {nav.go_to_pose_status}"
        )
        return ExperimentResult.FAILURE_NAVIGATION

    async def cancel(self) -> None:
        if self._swarm_nav is not None:
            await self._swarm_nav.cancel_navigation()

    async def shutdown(self) -> None:
        await self.cancel()
