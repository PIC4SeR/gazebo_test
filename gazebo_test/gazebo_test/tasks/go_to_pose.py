from collections import defaultdict, deque
from rclpy.node import Node
import rclpy

from gazebo_test.utils.gazebo_env_handler import GazeboEnvironmentHandler
from gazebo_test.utils.evaluation_handler import ExperimentEvaluator, ExperimentResult
from gazebo_test.utils.hunav_handler import HunavEvaluatorHandler
from gazebo_test.utils.bag_recorder import BagRecorder
from gazebo_test.utils.common_utils import (
    parse_entity_state_yaml,
    get_posestamped_from_entity,
)
from gazebo_test.utils.checkpoint_store import (
    ExperimentCheckpointStore,
    ExperimentJobDescriptor,
)
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import PoseStamped
from typing import List, Dict, Optional, Iterator, Set, Tuple, Union, Deque
from pathlib import Path
import pandas as pd
import os
import socket
import traceback
import uuid
import fcntl
import hashlib
import re
import asyncio
import fnmatch
import time
from urllib.parse import urlparse

import atexit
from contextlib import contextmanager

from gazebo_test.utils.navigation_handler import NavigationHandler
from rcl_interfaces.msg import Log as RosoutLog
from rclpy.qos import QoSProfile


def _coerce_ros_log_level(value: Union[int, float, str, bytes, bytearray]) -> int:
    """Normalize ROS log level values to plain integers."""
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, (bytes, bytearray)):
        # uint8 constants in ROS messages may show up as single-byte strings
        return int.from_bytes(value, byteorder="little", signed=False)
    if isinstance(value, (int,)):  # noqa: UP036
        return int(value)
    if isinstance(value, float):
        return int(value)
    if isinstance(value, str):
        cleaned = value.strip().lower()
        lookup = {
            "debug": 10,
            "info": 20,
            "warn": 30,
            "warning": 30,
            "error": 40,
            "fatal": 50,
        }
        if cleaned in lookup:
            return lookup[cleaned]
        if cleaned.isdigit():
            return int(cleaned)
        raise ValueError(f"Unsupported log level string '{value}'")
    raise TypeError(f"Unsupported log level type: {type(value)!r}")


try:
    _LOG_LEVEL_ERROR = _coerce_ros_log_level(getattr(RosoutLog, "ERROR"))
except Exception:  # noqa: BLE001
    _LOG_LEVEL_ERROR = 40

from ament_index_python.packages import get_package_share_directory
from rclpy.parameter import Parameter


class ExperimentManager(Node):
    def __init__(self):
        super().__init__("experiment_manager")

        self._asyncio_loop: Optional[asyncio.AbstractEventLoop] = None

        declared_algorithm = self.declare_parameter(
            "algorithm_name", Parameter.Type.STRING
        ).value
        self.algorithm_name = str(declared_algorithm or "").strip() or "unknown"

        yaml_path_param = self.declare_parameter(
            "goals_and_poses_file",
            Parameter.Type.STRING,
        ).value

        if not yaml_path_param:
            raise ValueError("Parameter 'goals_and_poses_file' must be provided")
        yaml_path = str(yaml_path_param)

        base_path_value = self.declare_parameter(
            "base_path",
            "results/gazebo_test",
        ).value
        self._base_path_root = Path(
            str(base_path_value) or "results/gazebo_test"
        ).expanduser()
        self._experiment_identifier = Path(yaml_path).stem

        timeout_duration = float(
            self.declare_parameter("timeout_duration", Parameter.Type.DOUBLE).value  # type: ignore
        )

        self.use_recorder = bool(
            self.declare_parameter("use_recorder", Parameter.Type.BOOL).value
        )
        self.use_evaluator = bool(
            self.declare_parameter("use_evaluator", Parameter.Type.BOOL).value
        )
        self.repetitions = int(self.declare_parameter("repetitions", Parameter.Type.INTEGER).value)  # type: ignore
        self.record_maps = bool(
            self.declare_parameter("record_maps", Parameter.Type.BOOL).value
        )
        self.wait_before_start = int(
            self.declare_parameter("wait_before_start", Parameter.Type.INTEGER).value  # type: ignore
        )
        checkpoint_dsn_param = self.declare_parameter(
            "checkpoint_dsn",
            "",
        ).value
        env_checkpoint_dsn = os.environ.get("GAZEBO_TEST_CHECKPOINT_DSN", "")
        checkpoint_dsn = str(checkpoint_dsn_param or "").strip()

        if not checkpoint_dsn and env_checkpoint_dsn:
            checkpoint_dsn = env_checkpoint_dsn.strip()
        if not checkpoint_dsn:
            raise ValueError(
                "A PostgreSQL DSN must be provided via 'checkpoint_dsn' parameter or 'GAZEBO_TEST_CHECKPOINT_DSN' environment variable"
            )
        self.checkpoint_dsn = checkpoint_dsn

        self._checkpoint_namespace = self._derive_checkpoint_namespace(
            self.checkpoint_dsn
        )
        self.base_path = (
            self._base_path_root
            / self._checkpoint_namespace
            / self.algorithm_name
            / self._experiment_identifier
        ).expanduser()
        self.base_path.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(
            f"Results base path set to: {self.base_path} (checkpoint namespace: {self._checkpoint_namespace})"
        )

        heartbeat_interval_param = self.declare_parameter(
            "job_heartbeat_interval_sec",
            5.0,
        ).value
        if heartbeat_interval_param is None:
            heartbeat_interval = 5.0
        else:
            heartbeat_interval = float(heartbeat_interval_param)
        self._job_heartbeat_interval = max(heartbeat_interval, 1.0)

        heartbeat_timeout_param = self.declare_parameter(
            "job_heartbeat_timeout_sec",
            30.0,
        ).value
        if heartbeat_timeout_param is None:
            heartbeat_timeout = max(self._job_heartbeat_interval * 3.0, 15.0)
        else:
            heartbeat_timeout = float(heartbeat_timeout_param)
        self._job_heartbeat_timeout = max(
            heartbeat_timeout, self._job_heartbeat_interval * 1.5
        )

        required_nodes_param = self.declare_parameter(
            "watchdog_required_nodes",
            [
                "/gazebo",
                "/bt_navigator",
                "/controller_server",
                "/planner_server",
                "/hunav_evaluator_node" if self.use_evaluator else "",
                "/hunav_agent_manager",
                "/hunav_plugin",
            ],
        ).value
        if isinstance(required_nodes_param, (list, tuple)):
            required_nodes = [
                str(node).strip() for node in required_nodes_param if str(node).strip()
            ]
        elif isinstance(required_nodes_param, str):
            required_nodes = (
                [required_nodes_param.strip()] if required_nodes_param else []
            )
        else:
            required_nodes = []
        self._watchdog_required_nodes: List[str] = required_nodes

        module_grace_param = self.declare_parameter(
            "watchdog_startup_grace_sec",
            20.0,
        ).value
        if module_grace_param is None:
            module_grace = 20.0
        else:
            module_grace = float(module_grace_param)
        self._module_health_grace_sec = max(module_grace, 0.0)
        self._module_health_event: Optional[asyncio.Event] = None
        self._module_failure_active = False
        self._missing_modules: Set[str] = set()
        self._last_reported_missing_nodes: Set[str] = set()
        self._waiting_for_modules_logged = False
        self._watchdog_monitor_started_at = 0.0

        raw_error_patterns = self.declare_parameter(
            "watchdog_error_patterns",
            ["/hunav_plugin|Service /compute_agents timeout"],
        ).value
        self._watchdog_error_patterns = self._parse_watchdog_error_patterns(
            raw_error_patterns
        )
        threshold_param = self.declare_parameter(
            "watchdog_error_threshold",
            3,
        ).value
        window_param = self.declare_parameter(
            "watchdog_error_window_sec",
            5.0,
        ).value
        min_level_param = self.declare_parameter(
            "watchdog_error_min_level",
            _LOG_LEVEL_ERROR,
        ).value
        self._watchdog_error_threshold = max(
            1, int(threshold_param) if threshold_param is not None else 1
        )
        self._watchdog_error_window_sec = max(
            0.5, float(window_param) if window_param is not None else 5.0
        )
        if min_level_param is None:
            self._watchdog_error_min_level = _LOG_LEVEL_ERROR
        else:
            try:
                self._watchdog_error_min_level = _coerce_ros_log_level(min_level_param)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(
                    f"Invalid watchdog_error_min_level value '{min_level_param}': {exc}; using default"
                )
                self._watchdog_error_min_level = _LOG_LEVEL_ERROR
        self._log_pattern_windows: Dict[str, Deque[float]] = defaultdict(deque)
        self._active_log_faults: Set[str] = set()
        self._rosout_subscription = None
        if self._watchdog_error_patterns:
            qos_profile = QoSProfile(depth=100)
            self._rosout_subscription = self.create_subscription(
                RosoutLog,
                "/rosout",
                self._rosout_callback,
                qos_profile=qos_profile,
            )
            self.get_logger().info(
                "Log watchdog active for patterns: "
                + ", ".join(
                    f"{node or '*'}|{pattern}"
                    for _, node, pattern in self._watchdog_error_patterns
                )
            )

        self._heartbeat_task: Optional[asyncio.Task[None]] = None
        self._watchdog_task: Optional[asyncio.Task[None]] = None
        self._heartbeat_lost = False

        checkpoint_connect_timeout_param = self.declare_parameter(
            "checkpoint_connect_timeout",
            5.0,
        ).value
        if checkpoint_connect_timeout_param is None:
            checkpoint_connect_timeout = 5.0
        else:
            checkpoint_connect_timeout = float(checkpoint_connect_timeout_param)

        self.get_logger().info(
            "Checkpoint database DSN configured for experiment checkpoints"
        )
        self.resume_checkpoint = bool(
            self.declare_parameter("resume_checkpoint", Parameter.Type.BOOL).value
        )
        self.auto_recover_running = bool(
            self.declare_parameter(
                "auto_recover_running",
                Parameter.Type.BOOL,
            ).value
        )
        worker_id_param = self.declare_parameter(
            "worker_id",
            Parameter.Type.STRING,
        ).value
        if worker_id_param:
            self.worker_id = str(worker_id_param)
        else:
            hostname = socket.gethostname()
            pid = os.getpid()
            suffix = uuid.uuid4().hex[:8]
            self.worker_id = f"{hostname}-{pid}-{suffix}"
        self.get_logger().info(f"Worker id set to: {self.worker_id}")

        self.gazebo_env_handler = GazeboEnvironmentHandler(self)
        self.evaluation_handler = ExperimentEvaluator(
            self,
            timeout_duration=timeout_duration,
        )
        if self.use_recorder:
            self.bag_recorder = BagRecorder(
                self,
                algorithm=self.algorithm_name,  # type: ignore
                base_path=self.base_path,  # type: ignore
                record_maps=self.record_maps,
            )
        if self.use_evaluator:
            self.hunav_evaluator_handler = HunavEvaluatorHandler(
                self,
                algorithm=self.algorithm_name,  # type: ignore
                base_path=self.base_path,  # type: Path
            )

        self.navigator = NavigationHandler(
            node=self,
            navigation_result_callback=self.evaluation_handler.set_navigation_result_event,
        )

        self.initial_state_entities: Dict[str, EntityState] = {}
        self.goal_entities: Dict[str, EntityState] = {}

        self.experiment_outcomes_path = Path(
            f"{self.base_path.resolve()}/{self.algorithm_name}_outcomes.csv"
        )
        self._outcomes_lock_path = Path(f"{self.experiment_outcomes_path}.lock")

        entity_dictionary = parse_entity_state_yaml(Path(yaml_path))
        self.initial_state_entities = entity_dictionary["initial_state_entities"]
        self.goal_entities = entity_dictionary["goal_entities"]

        self.goal_box_xml = (
            Path(
                f"{get_package_share_directory('gazebo_sim')}/models/goal_box/model.sdf"
            )
            .open("r")
            .read()
        )
        self.checkpoint_store = ExperimentCheckpointStore(
            self.checkpoint_dsn,
            connect_timeout_sec=checkpoint_connect_timeout,
        )
        if self.auto_recover_running:
            recovered = self.checkpoint_store.recover_stale_jobs(
                worker_hint=self.worker_id
            )
            if recovered:
                self.get_logger().warning(
                    f"Recovered {recovered} previously running job(s)"
                )
        job_descriptors: List[ExperimentJobDescriptor] = []
        for experiment_tag in self.initial_state_entities.keys():
            for run_id in range(1, self.repetitions + 1):
                job_descriptors.append(
                    ExperimentJobDescriptor(
                        algorithm=self.algorithm_name,
                        experiment_identifier=self._experiment_identifier,
                        experiment_tag=experiment_tag,
                        run_id=run_id,
                    )
                )
        self.checkpoint_store.register_jobs(job_descriptors)
        if not self.resume_checkpoint:
            reset_count = self.checkpoint_store.reset_jobs(
                algorithm=self.algorithm_name,
                experiment_identifier=self._experiment_identifier,
            )
            self.get_logger().info(
                f"Reset {reset_count} job(s) to PENDING for a fresh run"
            )
        self._active_job_id: Optional[int] = None
        atexit.register(self._cleanup_active_job)
        self.end = False
        self.get_logger().info("ExperimentManager initialized")
        # set log level to debug
        # self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

    async def initialize(self):
        # Placeholder for initialization logic
        self.get_logger().debug("Initializing experiment ...")
        if self._asyncio_loop is None:
            try:
                self._asyncio_loop = asyncio.get_running_loop()
            except RuntimeError:
                self._asyncio_loop = None
        # Initialize the experiment manager
        await self.gazebo_env_handler.wait_for_gazebo_ready()
        self.get_logger().debug("All entities are in the world")
        await self.navigator.initialize_navigation()
        await self.gazebo_env_handler.pause_gazebo()
        self.get_logger().debug("Navigation stack initialized")
        self.evaluation_handler.initialize()
        self.get_logger().debug("Experiment evaluator initialized")
        if self.use_evaluator:
            await self.hunav_evaluator_handler.wait_for_hunav_evaluator()
            self.get_logger().debug("Hunav evaluator handler initialized")

    async def run_experiments(self):
        """
        Run the experiments sequentially.
        This method will run the experiments for each experiment_tag defined in the YAML file.
        """
        self.get_logger().debug("Running experiments ...")
        if self._module_health_event is None:
            self._module_health_event = asyncio.Event()
            self._module_health_event.set()
        self._watchdog_monitor_started_at = time.monotonic()
        if self._watchdog_task is None or self._watchdog_task.done():
            self._watchdog_task = asyncio.create_task(self._stale_job_watchdog())
        requeue_failed = getattr(self.checkpoint_store, "requeue_failed_jobs", None)
        if callable(requeue_failed):
            try:
                recovered_failed = requeue_failed(
                    algorithm=self.algorithm_name,
                    experiment_identifier=self._experiment_identifier,
                )
                if recovered_failed:
                    self.get_logger().warning(
                        f"Requeued {recovered_failed} FAILED job(s) for retry"
                    )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"Unable to requeue FAILED jobs before dispatch: {exc}"
                )
        try:
            while True:
                if (
                    self._module_health_event is not None
                    and not self._module_health_event.is_set()
                ):
                    if not self._waiting_for_modules_logged:
                        self.get_logger().warning(
                            "Waiting for required modules to come online before claiming next job"
                        )
                        self._waiting_for_modules_logged = True
                    await self._module_health_event.wait()
                    self._waiting_for_modules_logged = False
                try:
                    requeued = self.checkpoint_store.requeue_stale_jobs(  # type: ignore[attr-defined]
                        algorithm=self.algorithm_name,
                        experiment_identifier=self._experiment_identifier,
                        heartbeat_timeout_sec=self._job_heartbeat_timeout,
                    )
                    if requeued:
                        self.get_logger().warning(
                            f"Requeued {requeued} stale RUNNING job(s) lacking heartbeats"
                        )
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(f"Failed to refresh stale jobs: {exc}")
                job = self.checkpoint_store.claim_next_job(
                    algorithm=self.algorithm_name,
                    experiment_identifier=self._experiment_identifier,
                    worker_id=self.worker_id,
                )
                if job is None:
                    break
                experiment_tag = job.experiment_tag
                run_id = job.run_id
                self.get_logger().info(
                    f"Worker {self.worker_id} starting {experiment_tag} run {run_id}"
                )
                self._active_job_id = job.id
                self._start_job_heartbeat(job.id)
                try:
                    if not self.experiment_outcomes_path.exists():
                        self.experiment_outcomes_path.parent.mkdir(
                            parents=True, exist_ok=True
                        )
                    result = await self.run_experiment(experiment_tag, run_id)
                    if self._heartbeat_lost:
                        self.get_logger().warning(
                            f"Lost heartbeat for job {job.id}; skipping result storage so another worker can retry"
                        )
                        continue
                    self.get_logger().info(
                        f"Run {run_id} for {experiment_tag} completed with result: {result}"
                    )
                    experiment_outcome = pd.DataFrame(
                        {
                            "experiment_tag": [experiment_tag],
                            "run_id": [run_id],
                            "result": [result],
                        }
                    )
                    if self.use_evaluator:
                        hunav_results = self.hunav_evaluator_handler.get_result_df()
                        experiment_outcome = pd.merge(
                            experiment_outcome,
                            hunav_results,
                            on=["experiment_tag", "run_id"],
                            how="left",
                        )
                    with self._acquire_outcomes_lock():
                        file_exists = self.experiment_outcomes_path.exists()
                        experiment_outcome.to_csv(
                            self.experiment_outcomes_path,
                            mode="a",
                            header=not file_exists,
                            index=False,
                        )
                        # sort the outcomes file by experiment_tag and run_id
                        df = pd.read_csv(self.experiment_outcomes_path)
                        df = df.sort_values(by=["experiment_tag", "run_id"])
                        df.to_csv(
                            self.experiment_outcomes_path,
                            mode="w",
                            header=True,
                            index=False,
                        )
                    self.checkpoint_store.mark_job_done(job.id, str(result))
                except Exception as exc:  # noqa: BLE001
                    error_message = (
                        f"Experiment {experiment_tag} run {run_id} failed: {exc}"
                    )
                    self.get_logger().error(error_message)
                    self.get_logger().debug(traceback.format_exc())
                    if not self._heartbeat_lost:
                        self.checkpoint_store.mark_job_failed(job.id, error_message)
                    else:
                        self.get_logger().warning(
                            "Skipping failure mark because job ownership was lost"
                        )
                    raise
                finally:
                    await self._stop_job_heartbeat()
                    self._active_job_id = None
        finally:
            await self._stop_watchdog_task()
        await self.navigator.shutdown_navigation()
        self.get_logger().info("All experiments completed")
        # Save the experiment outcomes to a CSV file
        self.get_logger().info(
            f"Experiment outcomes saved to {self.experiment_outcomes_path}"
        )
        self.end = True

    async def run_experiment(
        self, experiment_tag: str = "episode_1", run_id: int = 1
    ) -> ExperimentResult:
        """
        Run the experiment for a given experiment_tag.
        This method will reset the environment, start the experiment, and return the result.
        The experiment is considered successful if the robot reaches the goal
        without any collisions and within the timeout duration.
        The experiment is considered a failure if:
        - A collision is detected with an agent or the environment
        - The timeout duration is reached
        Args:
            experiment_tag (str): The experiment_tag to run.
            run_id (int): The run ID for the experiment.
        Returns:
            ExperimentResult: The result of the experiment.
        """
        # Placeholder for experiment running logic
        entities_to_reset = [
            self.initial_state_entities[experiment_tag],
        ]
        goal_pose = get_posestamped_from_entity(
            self.goal_entities[experiment_tag], "map"
        )
        await self.gazebo_env_handler.reset_environment_for_experiment(
            entities_to_reset,
            goal_entity=self.goal_entities[experiment_tag],
            goal_xml=self.goal_box_xml,
        )
        await self.navigator.reset_navigation()
        # wait for a few seconds before starting the experiment (to let the environment settle)
        if self.wait_before_start:
            self.get_logger().debug(
                f"Waiting for {self.wait_before_start} seconds before starting the experiment ..."
            )
            # create a rate object to wait
            rclpy_rate = self.create_rate(1 / self.wait_before_start)
            rclpy_rate.sleep()
            self.get_logger().debug("Wait completed")
        # start navigation
        # create task to wait for the robot to reach the goal
        self.navigator.start_navigation_task(
            goal_pose=goal_pose,
            navigation_result_callback=self.evaluation_handler.set_navigation_result_event,
        )
        self.get_logger().debug("Environment reset successfully")

        if self.use_recorder:
            self.get_logger().debug("Starting recording ...")
            self.bag_recorder.start_recording_and_set_goal(
                experiment_name=experiment_tag,
                run_id=str(run_id),
                goal=goal_pose,
            )
        if self.use_evaluator:
            self.get_logger().info("Starting hunav evaluator recording ...")
            await self.hunav_evaluator_handler.start_recording(
                goal=goal_pose,
                experiment_tag=experiment_tag,
                run_id=run_id,
            )

        experiment_result = await self.evaluation_handler.run_experiment()
        # cancel the navigation task if it is still running
        self.get_logger().debug("Experiment completed")
        if self.use_recorder:
            self.bag_recorder.set_result_and_stop_recording(
                result=str(experiment_result)
            )
            self.get_logger().debug("Recording stopped")
        if self.use_evaluator:
            await self.hunav_evaluator_handler.stop_recording()
            self.get_logger().info("Hunav evaluator recording stopped")
        # Log the experiment result
        self.get_logger().debug(
            f"Experiment {experiment_tag} run {run_id} completed with result: {experiment_result}"
        )
        await self.navigator.cancel_navigation()
        return experiment_result

    def _start_job_heartbeat(self, job_id: int) -> None:
        if self._heartbeat_task is not None and not self._heartbeat_task.done():
            self._heartbeat_task.cancel()
        self._heartbeat_lost = False
        self._heartbeat_task = asyncio.create_task(self._heartbeat_loop(job_id))

    async def _stop_job_heartbeat(self) -> None:
        if self._heartbeat_task is None:
            return
        task = self._heartbeat_task
        self._heartbeat_task = None
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    async def _heartbeat_loop(self, job_id: int) -> None:
        try:
            while True:
                await asyncio.sleep(self._job_heartbeat_interval)
                updated = self.checkpoint_store.refresh_job_heartbeat(  # type: ignore[attr-defined]
                    job_id,
                    self.worker_id,
                )
                if not updated:
                    self._heartbeat_lost = True
                    self.get_logger().warning(
                        f"Heartbeat rejected for job {job_id}; job will be requeued"
                    )
                    return
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Failed to refresh heartbeat for job {job_id}: {exc}"
            )

    async def _stale_job_watchdog(self) -> None:
        try:
            while not self.end:
                await asyncio.sleep(self._job_heartbeat_interval)
                try:
                    requeued = self.checkpoint_store.requeue_stale_jobs(  # type: ignore[attr-defined]
                        algorithm=self.algorithm_name,
                        experiment_identifier=self._experiment_identifier,
                        heartbeat_timeout_sec=self._job_heartbeat_timeout,
                    )
                    if requeued:
                        self.get_logger().warning(
                            f"Watchdog requeued {requeued} stale RUNNING job(s)"
                        )
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(
                        f"Watchdog failed to requeue stale jobs: {exc}"
                    )
                try:
                    await self._evaluate_module_health()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(
                        f"Watchdog failed module health check: {exc}"
                    )
        except asyncio.CancelledError:
            raise

    async def _evaluate_module_health(self) -> None:
        if not self._watchdog_required_nodes:
            if self._module_health_event and not self._module_health_event.is_set():
                self._module_health_event.set()
                self._waiting_for_modules_logged = False
            return
        present_nodes = self._collect_node_names()
        missing = self._missing_required_nodes(present_nodes)
        if missing:
            self._missing_modules = missing
            if not self._module_failure_active:
                elapsed = 0.0
                if self._watchdog_monitor_started_at > 0.0:
                    elapsed = time.monotonic() - self._watchdog_monitor_started_at
                if elapsed < self._module_health_grace_sec:
                    return
                await self._handle_missing_required_nodes(missing)
            else:
                if self._module_health_event and self._module_health_event.is_set():
                    self._module_health_event.clear()
                if missing != self._last_reported_missing_nodes:
                    self._last_reported_missing_nodes = set(missing)
                    self.get_logger().error(
                        "Required modules still missing: " + ", ".join(sorted(missing))
                    )
            return
        if self._module_failure_active:
            self._module_failure_active = False
            self._missing_modules.clear()
            self._last_reported_missing_nodes = set()
            if self._module_health_event and not self._module_health_event.is_set():
                self.get_logger().info(
                    "All required modules detected; resuming job dispatch"
                )
                self._module_health_event.set()
            self._waiting_for_modules_logged = False
            self._active_log_faults.clear()
            for window in self._log_pattern_windows.values():
                window.clear()
            return
        if self._module_health_event and not self._module_health_event.is_set():
            self._module_health_event.set()
            self._waiting_for_modules_logged = False

    async def _handle_missing_required_nodes(self, missing: Set[str]) -> None:
        reason = ", ".join(sorted(missing))
        self._module_failure_active = True
        self._last_reported_missing_nodes = set(missing)
        if self._module_health_event and self._module_health_event.is_set():
            self._module_health_event.clear()
        self._waiting_for_modules_logged = False
        self.get_logger().error(
            "Critical modules missing from ROS graph: "
            + reason
            + "; pausing job dispatch"
        )
        active_job = self._active_job_id
        if active_job is not None:
            try:
                requeued = self.checkpoint_store.force_requeue_job(  # type: ignore[attr-defined]
                    active_job,
                    reason=f"Dependent modules unavailable: {reason}",
                    expected_worker_id=self.worker_id,
                )
                if requeued:
                    self.get_logger().warning(
                        f"Requeued job {active_job} because required modules disappeared"
                    )
                else:
                    self.get_logger().warning(
                        f"Job {active_job} could not be requeued; it may have already been released"
                    )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"Failed to requeue job {active_job} after module loss: {exc}"
                )
            finally:
                self._heartbeat_lost = True
                await self._stop_job_heartbeat()
                try:
                    await self.navigator.cancel_navigation()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warning(
                        f"Failed to cancel navigation during module loss recovery: {exc}"
                    )
                try:
                    if hasattr(self.evaluation_handler, "collision_event"):
                        self.evaluation_handler.experiment_result = (
                            ExperimentResult.FAILURE_NAVIGATION
                        )
                        self.evaluation_handler.collision_event.set()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().debug(
                        f"Failed to notify evaluator after module loss: {exc}"
                    )

    def _collect_node_names(self) -> Set[str]:
        nodes: Set[str] = set()
        try:
            discovered = self.get_node_names_and_namespaces()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f"Failed to query node graph: {exc}")
            return nodes
        for name, namespace in discovered:
            full_name = self._fully_qualified_name(namespace, name)
            nodes.add(name)
            nodes.add(full_name)
            nodes.add(full_name.lstrip("/"))
        return nodes

    @staticmethod
    def _fully_qualified_name(namespace: str, name: str) -> str:
        if not namespace or namespace == "/":
            return f"/{name}"
        cleaned = namespace
        if not cleaned.startswith("/"):
            cleaned = f"/{cleaned}"
        cleaned = cleaned.rstrip("/")
        return f"{cleaned}/{name}"

    def _missing_required_nodes(self, present: Set[str]) -> Set[str]:
        missing: Set[str] = set()
        for pattern in self._watchdog_required_nodes:
            if not pattern:
                continue
            if not any(
                self._node_pattern_matches(pattern, candidate) for candidate in present
            ):
                missing.add(pattern)
        return missing

    @staticmethod
    def _node_pattern_matches(pattern: str, candidate: str) -> bool:
        if fnmatch.fnmatch(candidate, pattern):
            return True
        if pattern.startswith("/") and fnmatch.fnmatch(candidate, pattern.lstrip("/")):
            return True
        if not pattern.startswith("/") and fnmatch.fnmatch(f"/{candidate}", pattern):
            return True
        return False

    def _parse_watchdog_error_patterns(
        self,
        raw_patterns: Union[str, List[str], Tuple[str, ...], None],
    ) -> List[Tuple[str, str, str]]:
        patterns: List[Tuple[str, str, str]] = []
        if raw_patterns is None:
            return patterns
        if isinstance(raw_patterns, str):
            entries = [raw_patterns]
        elif isinstance(raw_patterns, (list, tuple)):
            entries = list(raw_patterns)
        else:
            return patterns
        seen: Set[str] = set()
        for entry in entries:
            text = str(entry).strip()
            if not text:
                continue
            if "|" in text:
                node_filter, substring = text.split("|", 1)
            else:
                node_filter, substring = "", text
            normalized_node = node_filter.strip()
            normalized_pattern = substring.strip()
            key = (
                f"{normalized_node}|{normalized_pattern}"
                if normalized_node
                else normalized_pattern
            )
            if key in seen:
                continue
            seen.add(key)
            patterns.append((key, normalized_node, normalized_pattern))
        return patterns

    def _rosout_callback(self, log_msg: RosoutLog) -> None:
        if not self._watchdog_error_patterns:
            return
        if log_msg.level < self._watchdog_error_min_level:
            return
        node_name = log_msg.name or ""
        message_text = log_msg.msg or ""
        timestamp = float(log_msg.stamp.sec) + float(log_msg.stamp.nanosec) * 1e-9
        lowered_message = message_text.lower()
        for pattern_key, node_filter, substring in self._watchdog_error_patterns:
            if node_filter and not self._node_pattern_matches(node_filter, node_name):
                continue
            if substring and substring.lower() not in lowered_message:
                continue
            self._handle_log_pattern_hit(
                pattern_key,
                timestamp,
                node_name,
                message_text,
                substring,
            )

    def _handle_log_pattern_hit(
        self,
        pattern_key: str,
        timestamp: float,
        node_name: str,
        message_text: str,
        pattern_description: str,
    ) -> None:
        window = self._log_pattern_windows[pattern_key]
        window.append(timestamp)
        cutoff = timestamp - self._watchdog_error_window_sec
        while window and window[0] < cutoff:
            window.popleft()
        if pattern_key in self._active_log_faults:
            return
        if len(window) < self._watchdog_error_threshold:
            return
        display_pattern = pattern_description or pattern_key
        reason = (
            f"Log errors from '{node_name or 'unknown'}' matched '{display_pattern}'"
            f" ({self._watchdog_error_threshold} hits in {self._watchdog_error_window_sec:.1f}s)"
        )
        self.get_logger().error(
            f"Watchdog detected repeated log errors from '{node_name or 'unknown'}': {message_text}"
        )
        self._active_log_faults.add(pattern_key)
        window.clear()
        if self._asyncio_loop is None:
            self.get_logger().warning(
                f"Asyncio loop unavailable to process watchdog fault for pattern '{pattern_key}'"
            )
            return
        try:
            asyncio.run_coroutine_threadsafe(
                self._handle_missing_required_nodes({reason}),
                self._asyncio_loop,
            )
        except RuntimeError as exc:
            self.get_logger().error(
                f"Failed to schedule watchdog recovery for pattern '{pattern_key}': {exc}"
            )

    async def _stop_watchdog_task(self) -> None:
        if self._watchdog_task is None:
            return
        task = self._watchdog_task
        self._watchdog_task = None
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    @contextmanager
    def _acquire_outcomes_lock(self) -> Iterator[None]:
        self._outcomes_lock_path.parent.mkdir(parents=True, exist_ok=True)
        with self._outcomes_lock_path.open("a+") as lock_file:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)

    @staticmethod
    def _derive_checkpoint_namespace(dsn: str) -> str:
        digest = hashlib.sha256(dsn.encode("utf-8")).hexdigest()[:12]
        components: List[str] = []
        parsed = urlparse(dsn)
        if parsed.scheme:
            if parsed.hostname:
                components.append(parsed.hostname)
            if parsed.path and parsed.path not in {"", "/"}:
                components.append(parsed.path.strip("/"))
            if parsed.port:
                components.append(str(parsed.port))
        if not components:
            for key, value in re.findall(r"([A-Za-z_]+)=([^\s]+)", dsn):
                if key.lower() in {"dbname", "database", "host", "port"} and value:
                    components.append(value)
        sanitized = [
            re.sub(r"[^A-Za-z0-9_.-]+", "_", piece.strip())
            for piece in components
            if piece.strip()
        ]
        label = "_".join(filter(None, sanitized))
        if label:
            return f"{label}__{digest}"
        return f"checkpoint__{digest}"

    def _cleanup_active_job(self) -> None:
        if self._active_job_id is None:
            return
        try:
            recovery_message = f"Worker {self.worker_id} terminated before finishing job {self._active_job_id}"
            self.checkpoint_store.mark_job_failed(
                self._active_job_id,
                recovery_message,
            )
        except Exception:  # noqa: BLE001
            # Avoid raising during interpreter shutdown; logging may be unavailable.
            pass
        finally:
            self._active_job_id = None

    @property
    def experiment_identifier(self) -> str:
        return self._experiment_identifier

    @property
    def checkpoint_namespace(self) -> str:
        return self._checkpoint_namespace

    @staticmethod
    def _looks_like_dsn(candidate: str) -> bool:
        if not candidate:
            return False
        if "://" in candidate:
            return True
        if "=" in candidate:
            return True
        return candidate.lower().startswith(
            "postgresql"
        ) or candidate.lower().startswith("postgres")
