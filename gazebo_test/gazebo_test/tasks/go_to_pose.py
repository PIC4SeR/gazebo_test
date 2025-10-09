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
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, PoseStamped
from typing import Optional, List, Dict, Any
from pathlib import Path
import json
import pandas as pd
import time

from gazebo_test.utils.navigation_handler import NavigationHandler

from ament_index_python.packages import get_package_share_directory
from rclpy.parameter import Parameter


class ExperimentManager(Node):
    def __init__(self):
        super().__init__("experiment_manager")

        self.algorithm_name = self.declare_parameter(
            "algorithm_name", Parameter.Type.STRING
        ).value

        yaml_path_param = self.declare_parameter(
            "goals_and_poses_file",
            Parameter.Type.STRING,
        ).value
        
        if not yaml_path_param:
            raise ValueError("Parameter 'goals_and_poses_file' must be provided")
        yaml_path = str(yaml_path_param)

        self.date = time.strftime("%d_%m_%Y__%H_%M_%S")
        base_path = str(self.declare_parameter(
            "base_path",
            "results/gazebo_test",
        ).value)
        exp_prefix = Path(yaml_path).stem
        self.base_path = Path(base_path).joinpath(
            f"{exp_prefix}_exp_{self.date}/",
        )

        timeout_duration = float(self.declare_parameter( 
            "timeout_duration", Parameter.Type.DOUBLE
        ).value) # type: ignore
        
        self.use_recorder = bool(self.declare_parameter("use_recorder", Parameter.Type.BOOL).value)
        self.use_evaluator = bool(
            self.declare_parameter("use_evaluator", Parameter.Type.BOOL).value
        )
        self.repetitions = int(self.declare_parameter("repetitions", Parameter.Type.INTEGER).value)  # type: ignore
        self.record_maps = bool(self.declare_parameter("record_maps", Parameter.Type.BOOL).value)
        self.wait_before_start = int(
            self.declare_parameter("wait_before_start", Parameter.Type.INTEGER).value # type: ignore
        )
        raw_checkpoint_path = self.declare_parameter(
            "checkpoint_path",
            str(Path.home() / f"{base_path}/.experiment_checkpoints/cache/{self.algorithm_name}"),
        ).value
        
        self.checkpoint_path = Path(str(raw_checkpoint_path)).expanduser()
        
        self.get_logger().info(
            f"Checkpoint path set to: {self.checkpoint_path}"
        )
        self.resume_checkpoint = bool(
            self.declare_parameter("resume_checkpoint", Parameter.Type.BOOL).value
        )
        self.get_logger().info(f"resume_checkpoint set to: {self.resume_checkpoint}")

        self.gazebo_env_handler = GazeboEnvironmentHandler(self)
        self.evaluation_handler = ExperimentEvaluator(
            self,
            timeout_duration=timeout_duration,
        )
        if self.use_recorder:
            self.bag_recorder = BagRecorder(
                self,
                algorithm=self.algorithm_name, # type: ignore
                base_path=self.base_path, # type: ignore
                record_maps=self.record_maps,
            )
        if self.use_evaluator:
            self.hunav_evaluator_handler = HunavEvaluatorHandler(
                self,
                algorithm=self.algorithm_name, # type: ignore
                base_path=self.base_path, # type: Path
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
        if not self.experiment_outcomes_path.exists():
            self.experiment_outcomes_path.parent.mkdir(parents=True, exist_ok=True)

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
        self._experiment_identifier = Path(yaml_path).stem
        self._checkpoint_data = self._load_checkpoint()
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
        for experiment_tag in self.initial_state_entities.keys():
            self.get_logger().info(f"Starting {experiment_tag}")
            for i in range(self.repetitions):
                run_id = i + 1
                checkpoint_key = self._checkpoint_key(experiment_tag, run_id)
                if (
                    self.resume_checkpoint
                    and (self._checkpoint_data.get(checkpoint_key) is not None)
                ):
                    self.get_logger().info(
                        f"Skipping {experiment_tag} run {run_id} (already marked as Done in checkpoint)"
                    )
                    continue

                result = await self.run_experiment(experiment_tag, run_id)
                self.get_logger().info(
                    f"Run {run_id} for {experiment_tag} completed with result: {result}"
                )
                # Save the result to the DataFrame
                experiment_outcome = pd.DataFrame(
                    {
                        "experiment_tag": [experiment_tag],
                        "run_id": [run_id],
                        "result": [result],
                    }
                )
                if self.use_evaluator:
                    # Add additional columns for hunav evaluator results
                    hunav_results = self.hunav_evaluator_handler.get_result_df()
                    # Merge hunav results into the DataFrame
                    experiment_outcome = pd.merge(
                        experiment_outcome,
                        hunav_results,
                        on=["experiment_tag", "run_id"],
                        how="left",
                    )

                # Append the result to the CSV file
                experiment_outcome.to_csv(
                    self.experiment_outcomes_path,
                    mode="a",
                    header=not self.experiment_outcomes_path.exists(),
                    index=False,
                )
                self._mark_checkpoint(checkpoint_key, str(result))

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

    def _checkpoint_key(self, experiment_tag: str, run_id: int) -> str:
        algorithm = self.algorithm_name or "unknown"
        return f"{algorithm}|{self._experiment_identifier}|{experiment_tag}|{run_id}"

    def _load_checkpoint(self) -> Dict[str, str]:
        if not self.checkpoint_path.exists():
            return {}
        try:
            raw_data = json.loads(self.checkpoint_path.read_text())
        except json.JSONDecodeError:
            self.get_logger().warning(
                f"Checkpoint file {self.checkpoint_path} is corrupted. Starting with an empty checkpoint."
            )
            return {}

        if isinstance(raw_data, dict):
            return {str(k): str(v) for k, v in raw_data.items()}
        if isinstance(raw_data, list):
            return {str(item): "Done" for item in raw_data}

        return {}

    def _mark_checkpoint(self, key: str, result: str) -> None:
        self._checkpoint_data[key] = result
        try:
            self.checkpoint_path.parent.mkdir(parents=True, exist_ok=True)
            self.checkpoint_path.write_text(json.dumps(self._checkpoint_data, indent=2))
        except OSError as exc:
            self.get_logger().error(
                f"Failed to write checkpoint file {self.checkpoint_path}: {exc}"
            )
