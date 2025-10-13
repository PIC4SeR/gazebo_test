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
from typing import List, Dict, Optional, Iterator
from pathlib import Path
import pandas as pd
import os
import socket
import traceback
import uuid
import fcntl
import hashlib
import re
from urllib.parse import urlparse

import atexit
from contextlib import contextmanager

from gazebo_test.utils.navigation_handler import NavigationHandler

from ament_index_python.packages import get_package_share_directory
from rclpy.parameter import Parameter


class ExperimentManager(Node):
    def __init__(self):
        super().__init__("experiment_manager")

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
        while True:
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
            try:
                if not self.experiment_outcomes_path.exists():
                    self.experiment_outcomes_path.parent.mkdir(
                        parents=True, exist_ok=True
                    )
                result = await self.run_experiment(experiment_tag, run_id)
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
                self._active_job_id = None
            except Exception as exc:  # noqa: BLE001
                error_message = (
                    f"Experiment {experiment_tag} run {run_id} failed: {exc}"
                )
                self.get_logger().error(error_message)
                self.get_logger().debug(traceback.format_exc())
                self.checkpoint_store.mark_job_failed(job.id, error_message)
                self._active_job_id = None
                raise

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
