"""Single-robot go-to-pose task.

One robot is reset to its initial pose and navigated to a goal; the episode
succeeds if it reaches the goal without collision before the timeout. This is
the per-episode behaviour only -- scheduling, checkpoints and results live in
:class:`~gazebo_test.experiment_manager.ExperimentManager`.
"""

import asyncio
from pathlib import Path
from typing import Dict, List

import rclpy.duration

from gazebo_msgs.msg import EntityState

from gazebo_test.tasks.base import ExperimentTask, register_task
from gazebo_test.utils.common_utils import (
    get_posestamped_from_entity,
    parse_fleet_yaml,
)
from gazebo_test.utils.evaluation_handler import ExperimentResult
from gazebo_test.utils.navigation_handler import NavigationHandler


@register_task("go_to_pose")
class GoToPoseTask(ExperimentTask):
    def __init__(self, manager) -> None:
        super().__init__(manager)
        self.navigator: NavigationHandler = None  # type: ignore[assignment]
        self.initial_state_entities: Dict[str, EntityState] = {}
        self.goal_entities: Dict[str, EntityState] = {}
        # The lone robot runs fully namespaced (spawn, control, Nav2) under its
        # own name, to match its now-namespaced Gazebo plugins.
        self.robot_namespace: str = "jackal"

    def load_entities(self, yaml_path: Path) -> List[str]:
        # Same fleet schema as the multi-robot task -- a single-robot config is
        # just a ``robots:`` list of one. Collapse the per-robot dicts to the lone
        # robot the single-robot pipeline drives.
        parsed = parse_fleet_yaml(yaml_path)
        fleet = parsed["fleet"]
        if len(fleet) != 1:
            raise ValueError(
                f"'{yaml_path}' has {len(fleet)} robots; the 'go_to_pose' task "
                "drives a single robot. Use 'task: multirobot' for a fleet."
            )
        ns = self.robot_namespace = fleet[0]["name"]
        self.robot_model = fleet[0].get("model")
        episodes = list(parsed["initial_state_entities"].keys())
        for episode in episodes:
            if ns not in parsed["goal_entities"].get(episode, {}):
                raise ValueError(
                    f"Episode '{episode}' is missing a 'goals' entry for '{ns}'."
                )
        self.initial_state_entities = {
            episode: parsed["initial_state_entities"][episode][ns]
            for episode in episodes
        }
        self.goal_entities = {
            episode: parsed["goal_entities"][episode][ns] for episode in episodes
        }
        return episodes

    def watchdog_required_nodes(self) -> List[str]:
        ns = self.robot_namespace
        nodes = ["/gazebo"]
        if self.manager.navigation_backend.strip().lower() == "nav2":
            nodes.extend(
                [
                    f"/{ns}/bt_navigator",
                    f"/{ns}/controller_server",
                    f"/{ns}/planner_server",
                ]
            )
        if self.manager.use_evaluator:
            nodes.append("/hunav_evaluator_node")
        nodes.extend(["/hunav_agent_manager", "/hunav_plugin"])
        return nodes

    async def setup(self) -> None:
        self.navigator = NavigationHandler(
            node=self.manager,
            navigation_result_callback=self.manager.evaluation_handler.set_navigation_result_event,
            backend=self.manager.navigation_backend,
            namespace=self.robot_namespace,
        )
        if self.manager.use_recorder:
            # The lone robot publishes under its own namespace (scan, cmd_vel,
            # odom, plan, tf, ...), so register those alongside the global
            # defaults -- same registration the fleet task does.
            self.manager.bag_recorder.add_robot_namespaces(
                [self.robot_namespace],
                models={self.robot_namespace: self.robot_model},
            )
        await self.navigator.initialize_navigation()

    async def run_episode(
        self, experiment_tag: str, run_id: int
    ) -> ExperimentResult:
        manager = self.manager
        log = manager.get_logger()

        entities_to_reset = [self.initial_state_entities[experiment_tag]]
        goal_entity = self.goal_entities[experiment_tag]
        goal_pose = get_posestamped_from_entity(goal_entity, "map")

        await manager.gazebo_env_handler.reset_environment_for_experiment(
            entities_to_reset,
            goal_entity=goal_entity,
            goal_xml=manager.goal_box_xml,
        )
        await self.navigator.reset_navigation()

        if manager.wait_before_start:
            log.debug(
                f"Waiting {manager.wait_before_start}s before starting the experiment"
            )
            wait_end = manager.get_clock().now() + rclpy.duration.Duration(
                seconds=manager.wait_before_start
            )
            while manager.get_clock().now() < wait_end:
                await asyncio.sleep(0.1)

        self.navigator.start_navigation_task(
            goal_pose=goal_pose,
            navigation_result_callback=manager.evaluation_handler.set_navigation_result_event,
        )
        log.debug("Environment reset successfully")

        if manager.use_recorder:
            manager.bag_recorder.start_recording_and_set_goal(
                experiment_name=experiment_tag,
                run_id=str(run_id),
                goal=goal_pose,
            )
        if manager.use_evaluator:
            log.info("Starting hunav evaluator recording ...")
            await manager.hunav_evaluator_handler.start_recording(
                goal=goal_pose,
                experiment_tag=experiment_tag,
                run_id=run_id,
            )

        experiment_result = await manager.evaluation_handler.run_experiment()
        log.debug("Experiment completed")

        if manager.use_recorder:
            manager.bag_recorder.set_result_and_stop_recording(
                result=str(experiment_result)
            )
        if manager.use_evaluator:
            await manager.hunav_evaluator_handler.stop_recording()
            log.info("Hunav evaluator recording stopped")

        log.debug(
            f"Experiment {experiment_tag} run {run_id} result: {experiment_result}"
        )
        await self.navigator.cancel_navigation()
        return experiment_result

    async def cancel(self) -> None:
        if self.navigator is not None:
            await self.navigator.cancel_navigation()
        # Break evaluation_handler.run_experiment() so the episode reports failure.
        eval_handler = self.manager.evaluation_handler
        if hasattr(eval_handler, "collision_event"):
            eval_handler.experiment_result = ExperimentResult.FAILURE_NAVIGATION
            eval_handler.collision_event.set()

    async def shutdown(self) -> None:
        if self.navigator is not None:
            await self.navigator.shutdown_navigation()
