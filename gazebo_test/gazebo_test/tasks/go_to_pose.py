from rclpy.node import Node

from gazebo_test.utils.gazebo_env_handler import GazeboEnvironmentHandler
from gazebo_test.utils.evaluation_handler import ExperimentEvaluator, ExperimentResult
from gazebo_test.utils.bag_recorder import BagRecorder
from gazebo_test.utils.common_utils import (
    parse_entity_state_yaml,
    get_posestamped_from_entity,
)
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, PoseStamped
from typing import Optional, List, Dict, Any
from pathlib import Path
import pandas as pd
import time

# from gazebo_test.utils.basic_navigator import BasicNavigator
from gazebo_test.utils.navigation_handler import NavigationHandler

from ament_index_python.packages import get_package_share_directory
from rclpy.parameter import Parameter


class ExperimentManager(Node):
    def __init__(self):
        super().__init__("experiment_manager")

        self.algorithm_name = self.declare_parameter(
            "algorithm_name", Parameter.Type.STRING
        ).value

        self.date = time.strftime("%d_%m_%Y__%H_%M_%S")
        base_path = self.declare_parameter(
            "base_path",
            f"/workspaces/hunavsim_ws/bags/gazebo_test",
        ).value
        self.base_path = Path(base_path).joinpath(
            f"exp_{self.date}/",
        )

        timeout_duration = self.declare_parameter(
            "timeout_duration", Parameter.Type.DOUBLE
        ).value
        self.use_recorder = self.declare_parameter(
            "use_recorder", Parameter.Type.BOOL
        ).value
        self.repetitions = self.declare_parameter(
            "repetitions", Parameter.Type.INTEGER
        ).value
        yaml_path = self.declare_parameter(
            "goals_and_poses_file",
            Parameter.Type.STRING,
        ).value

        self.gazebo_env_handler = GazeboEnvironmentHandler(self)
        self.evaluation_handler = ExperimentEvaluator(
            self,
            timeout_duration=timeout_duration,
        )
        if self.use_recorder:
            self.bag_recorder = BagRecorder(
                self, algorithm=self.algorithm_name, base_path=self.base_path
            )
        self.navigator = NavigationHandler(
            node=self,
            success_callback=self.evaluation_handler.set_success_event,
        )

        self.initial_state_entities: Dict[str, EntityState] = {}
        self.goal_entities: Dict[str, EntityState] = {}

        self.experiment_outcomes: pd.DataFrame = pd.DataFrame(
            columns=["episode", "run", "result"]
        )

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
        self.end = False
        self.get_logger().info("ExperimentManager initialized")

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

    async def run_experiments(self):
        """
        Run the experiments sequentially.
        This method will run the experiments for each episode defined in the YAML file.
        """
        self.get_logger().debug("Running experiments ...")
        for episode in self.initial_state_entities.keys():
            self.get_logger().info(f"Starting {episode}")
            for i in range(self.repetitions):
                result = await self.run_experiment(episode, i + 1)
                self.get_logger().info(
                    f"Run {i + 1} for {episode} completed with result: {result}"
                )
                # Save the result to the DataFrame
                self.experiment_outcomes = pd.concat(
                    [
                        self.experiment_outcomes,
                        pd.DataFrame(
                            {
                                "episode": [episode],
                                "run": [i + 1],
                                "result": [result],
                            }
                        ),
                    ],
                    ignore_index=True,
                )
        await self.navigator.shutdown_navigation()
        self.get_logger().info("All experiments completed")
        # Save the experiment outcomes to a CSV file
        self.experiment_outcomes.to_csv(self.experiment_outcomes_path, index=False)
        self.get_logger().info(
            f"Experiment outcomes saved to {self.experiment_outcomes_path}"
        )
        self.end = True

    async def run_experiment(
        self, episode: str = "episode_1", run_id: int = 1
    ) -> ExperimentResult:
        """
        Run the experiment for a given episode.
        This method will reset the environment, start the experiment, and return the result.
        The experiment is considered successful if the robot reaches the goal
        without any collisions and within the timeout duration.
        The experiment is considered a failure if:
        - A collision is detected with an agent or the environment
        - The timeout duration is reached
        Args:
            episode (str): The episode to run.
            run_id (int): The run ID for the experiment.
        Returns:
            ExperimentResult: The result of the experiment.
        """
        # Placeholder for experiment running logic
        entities_to_reset = [
            self.initial_state_entities[episode],
        ]
        goal_pose = get_posestamped_from_entity(self.goal_entities[episode], "map")
        await self.gazebo_env_handler.reset_environment_for_experiment(
            entities_to_reset,
            goal_entity=self.goal_entities[episode],
            goal_xml=self.goal_box_xml,
        )
        await self.navigator.reset_navigation()
        # start navigation
        # create task to wait for the robot to reach the goal
        self.navigator.start_navigation_task(
            goal_pose=goal_pose,
            success_callback=self.evaluation_handler.set_success_event,
        )
        self.get_logger().debug("Environment reset successfully")

        if self.use_recorder:
            self.get_logger().debug("Starting recording ...")
            self.bag_recorder.start_recording_and_set_goal(
                experiment_name=episode,
                run_id=str(run_id),
                goal=goal_pose,
            )

        experiment_result = await self.evaluation_handler.run_experiment()
        # cancel the navigation task if it is still running
        self.get_logger().debug("Experiment completed")
        if self.use_recorder:
            self.bag_recorder.set_result_and_stop_recording(
                result=str(experiment_result)
            )
            self.get_logger().debug("Recording stopped")
        self.get_logger().debug(f"Run result: {experiment_result}")
        await self.navigator.cancel_navigation()
        return experiment_result
