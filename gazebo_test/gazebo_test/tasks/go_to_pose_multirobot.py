"""Multi-robot go-to-pose task.

Every robot in the fleet is reset to its initial pose and driven to its own
goal via its own namespaced Nav2 stack. Per robot, the episode races the Nav2
result against that robot's collision sensor (``/<name>/collision``); the
episode succeeds only when *all* robots reach their goal without collision
before the timeout, and otherwise reports the most severe failure observed.

ponytail: aggregated HuNav social metrics are still deferred. Collision
detection per robot is handled here directly (the topics are namespaced after
the multi-robot plugin work) without touching the single-robot evaluator.
"""

import asyncio
from pathlib import Path
from typing import Dict, List, Optional

import rclpy.duration
from rclpy.time import Time

from gazebo_collision_msgs.msg import Collision
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import PoseStamped

from gazebo_test.tasks.base import ExperimentTask, register_task
from gazebo_test.utils.basic_navigator import TaskResult
from gazebo_test.utils.common_utils import (
    get_posestamped_from_entity,
    parse_fleet_yaml,
)
from gazebo_test.utils.evaluation_handler import ExperimentResult
from gazebo_test.utils.navigation_handler import NavigationHandler

# Most-severe-first ordering used to aggregate per-robot outcomes into one
# episode result.
_RESULT_SEVERITY = [
    ExperimentResult.FAILURE_COLLISION_AGENT,
    ExperimentResult.FAILURE_COLLISION_ROBOT,
    ExperimentResult.FAILURE_COLLISION_ENVIRONMENT,
    ExperimentResult.FAILURE_NAVIGATION,
    ExperimentResult.FAILURE_TIMEOUT,
]


@register_task("multirobot")
class MultiRobotGoToPoseTask(ExperimentTask):
    def __init__(self, manager) -> None:
        super().__init__(manager)
        self.fleet: List[dict] = []
        self.navigators: Dict[str, NavigationHandler] = {}
        # episode -> {robot_name -> EntityState}
        self.initial_state_entities: Dict[str, Dict[str, EntityState]] = {}
        self.goal_entities: Dict[str, Dict[str, EntityState]] = {}
        # Per-robot collision detection state.
        self._robot_names: set = set()
        self._collision_event: Dict[str, asyncio.Event] = {}
        self._collision_result: Dict[str, Optional[ExperimentResult]] = {}
        self._episode_start: Optional[Time] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def load_entities(self, yaml_path: Path) -> List[str]:
        parsed = parse_fleet_yaml(yaml_path)
        self.fleet = parsed["fleet"]
        self.initial_state_entities = parsed["initial_state_entities"]
        self.goal_entities = parsed["goal_entities"]
        self.manager.get_logger().info(
            f"Loaded fleet of {len(self.fleet)} robots: "
            f"{[r['name'] for r in self.fleet]}"
        )
        return list(self.initial_state_entities.keys())

    def watchdog_required_nodes(self) -> List[str]:
        # ponytail: loose health check -- per-robot nav nodes are namespaced.
        # Add ``/{ns}/bt_navigator`` checks once strict supervision is needed.
        nodes = ["/gazebo"]
        if self.manager.use_evaluator:
            nodes.append("/hunav_evaluator_node")
        nodes.extend(["/hunav_agent_manager", "/hunav_plugin"])
        return nodes

    async def setup(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._robot_names = {robot["name"] for robot in self.fleet}
        for robot in self.fleet:
            ns = robot["name"]
            self.navigators[ns] = NavigationHandler(
                node=self.manager,
                backend=self.manager.navigation_backend,
                namespace=ns,
            )
            self._collision_event[ns] = asyncio.Event()
            self._collision_result[ns] = None
            # Each robot's collision sensor publishes /<ns>/collision (the plugin
            # namespace == robot name after the multi-robot plugin work).
            self.manager.create_subscription(
                Collision,
                f"/{ns}/collision",
                self._make_collision_callback(ns),
                10,
            )
        if self.manager.use_recorder or self.manager.use_evaluator:
            self.manager.get_logger().warning(
                "Bag recording and HuNav evaluation are not supported in "
                "multi-robot mode yet; ignoring them for this run."
            )
        await asyncio.gather(
            *(nh.initialize_navigation() for nh in self.navigators.values())
        )

    async def run_episode(
        self, experiment_tag: str, run_id: int
    ) -> ExperimentResult:
        manager = self.manager
        initial_states = self.initial_state_entities[experiment_tag]
        goals = self.goal_entities[experiment_tag]

        await manager.gazebo_env_handler.reset_environment_for_experiment_multi(
            entities=list(initial_states.values()),
            goal_entities=list(goals.values()),
            goal_xml=manager.goal_box_xml,
        )
        await asyncio.gather(
            *(nh.reset_navigation() for nh in self.navigators.values())
        )

        if manager.wait_before_start:
            wait_end = manager.get_clock().now() + rclpy.duration.Duration(
                seconds=manager.wait_before_start
            )
            while manager.get_clock().now() < wait_end:
                await asyncio.sleep(0.1)

        # Arm collision detection: ignore any sensor message stamped before now,
        # and clear per-robot state from the previous episode.
        self._episode_start = manager.get_clock().now()
        for ns in self.navigators:
            self._collision_event[ns].clear()
            self._collision_result[ns] = None

        manager.get_logger().debug(
            f"Driving {len(self.navigators)} robots to their goals"
        )
        try:
            # ponytail: wall-clock timeout (sim usually runs ~realtime). Switch to
            # a sim-time ROS timer like the single-robot evaluator if RTF drifts.
            results = await asyncio.wait_for(
                asyncio.gather(
                    *(
                        self._run_single_robot(
                            ns, get_posestamped_from_entity(goals[ns], "map")
                        )
                        for ns in self.navigators
                    )
                ),
                timeout=manager.evaluation_handler.timeout_duration,
            )
            episode_result = self._aggregate(results)
        except asyncio.TimeoutError:
            manager.get_logger().warning(
                f"Episode '{experiment_tag}' timed out before all robots reached goal"
            )
            episode_result = ExperimentResult.FAILURE_TIMEOUT

        await self.cancel()
        manager.get_logger().info(
            f"Episode '{experiment_tag}' run {run_id} result: {episode_result}"
        )
        return episode_result

    async def _run_single_robot(
        self, ns: str, goal_pose: PoseStamped
    ) -> ExperimentResult:
        nav = self.navigators[ns].navigator  # BasicNavigator
        nav.clearEvents()
        accepted = await nav.goToPose(goal_pose)
        if not accepted:
            self.manager.get_logger().warning(f"[{ns}] goal rejected")
            return ExperimentResult.FAILURE_NAVIGATION

        # Whichever happens first: the robot reaches its goal, or it collides.
        nav_wait = asyncio.ensure_future(nav.go_to_pose_event.wait())
        col_wait = asyncio.ensure_future(self._collision_event[ns].wait())
        done, pending = await asyncio.wait(
            {nav_wait, col_wait}, return_when=asyncio.FIRST_COMPLETED
        )
        for task in pending:
            task.cancel()

        collision_result = self._collision_result[ns]
        if col_wait in done and collision_result is not None:
            self.manager.get_logger().warning(f"[{ns}] {collision_result}")
            return collision_result
        if nav.go_to_pose_status == TaskResult.SUCCEEDED:
            return ExperimentResult.SUCCESS
        self.manager.get_logger().warning(
            f"[{ns}] navigation ended: {nav.go_to_pose_status}"
        )
        return ExperimentResult.FAILURE_NAVIGATION

    def _make_collision_callback(self, ns: str):
        def _callback(msg: Collision) -> None:
            # Ignore stale messages from before the episode started.
            if self._episode_start is None:
                return
            if Time.from_msg(msg.header.stamp) <= self._episode_start:
                return
            if self._collision_event[ns].is_set():
                return
            self._collision_result[ns] = self._classify_collision(ns, msg)
            if self._loop is not None:
                self._loop.call_soon_threadsafe(self._collision_event[ns].set)

        return _callback

    def _classify_collision(self, ns: str, msg: Collision) -> ExperimentResult:
        hits = list(getattr(msg, "objects_hit", []) or [])
        hit = hits[0] if hits else ""
        if "agent" in hit:
            return ExperimentResult.FAILURE_COLLISION_AGENT
        # Another fleet robot (not this one) named in the contact.
        if any(other != ns and other in hit for other in self._robot_names):
            return ExperimentResult.FAILURE_COLLISION_ROBOT
        return ExperimentResult.FAILURE_COLLISION_ENVIRONMENT

    @staticmethod
    def _aggregate(results: List[ExperimentResult]) -> ExperimentResult:
        """Reduce per-robot outcomes to one episode result (most severe wins)."""
        if all(r == ExperimentResult.SUCCESS for r in results):
            return ExperimentResult.SUCCESS
        for severe in _RESULT_SEVERITY:
            if severe in results:
                return severe
        return ExperimentResult.FAILURE_NAVIGATION

    async def cancel(self) -> None:
        await asyncio.gather(
            *(nh.cancel_navigation() for nh in self.navigators.values())
        )

    async def shutdown(self) -> None:
        # ponytail: best-effort shutdown of every robot's nav stack at process end.
        await asyncio.gather(
            *(nh.shutdown_navigation() for nh in self.navigators.values()),
            return_exceptions=True,
        )
