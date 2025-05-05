from rclpy.node import Node

from gazebo_test.utils.gazebo_env_handler import GazeboEnvironmentHandler
from gazebo_test.utils.evaluation_handler import ExperimentEvaluator, ExperimentResult
from gazebo_test.utils.bag_recorder import BagRecorder
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose
from typing import Optional, List, Dict, Any
from pathlib import Path
from tf_transformations import quaternion_from_euler
import pandas as pd
import time

from ament_index_python.packages import get_package_share_directory

import yaml


class ExperimentManager(Node):
    def __init__(self):
        super().__init__("experiment_manager")
        self.algorithm_name = "test"  # todo: make it configurable

        self.date = time.strftime("%d_%m_%Y__%H_%M_%S")
        self.base_path = Path(
            f"/workspaces/hunavsim_ws/bags/gazebo_test/exp_{self.date}/"
        )  # todo: make it configurable

        self.gazebo_env_handler = GazeboEnvironmentHandler(self)
        self.evaluation_handler = ExperimentEvaluator(
            self, timeout_duration=5.0
        )  # todo: make it configurable
        self.bag_recorder = BagRecorder(
            self, algorithm=self.algorithm_name, base_path=self.base_path
        )

        self.use_recorder = True  # todo: make it configurable
        self.repetitions = 2  # todo: make it configurable

        self.initial_state_entities: Dict[str, EntityState] = {}
        self.goal_entities: Dict[str, EntityState] = {}
        self.goal_name: str = "goal_box"

        self.experiment_outcomes: pd.DataFrame = pd.DataFrame(
            columns=["episode", "run", "result"]
        )

        self.experiment_outcomes_path = Path(
            f"{self.base_path.resolve()}/{self.algorithm_name}_outcomes.csv"
        )
        if not self.experiment_outcomes_path.exists():
            self.experiment_outcomes_path.parent.mkdir(parents=True, exist_ok=True)

        self.parse_entity_state_yaml(
            Path(
                f"{get_package_share_directory('gazebo_test')}/goals_and_poses/social_nav.yaml"
            )
        )  # todo: make it configurable

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
        await self.gazebo_env_handler.pause_gazebo()
        self.get_logger().debug("All entities are in the world")

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

        await self.gazebo_env_handler.reset_environment_for_experiment(
            entities_to_reset,
            goal_entity=self.goal_entities[episode],
            goal_xml=self.goal_box_xml,
        )

        self.get_logger().debug("Environment reset successfully")

        self.get_logger().debug("Starting recording ...")
        self.bag_recorder.start_recording(
            experiment_name=episode,
            run_id=str(run_id),
        )

        experiment_result = await self.evaluation_handler.run_experiment()
        self.get_logger().debug("Experiment completed")
        self.bag_recorder.set_experiment_result(result=str(experiment_result))
        self.bag_recorder.stop_recording()
        self.get_logger().debug("Recording stopped")
        self.get_logger().debug(f"Run result: {experiment_result}")
        return experiment_result

    def parse_entity_state_yaml(self, yaml_path: Path) -> Dict[str, EntityState]:
        """
        Parse data to create a list of EntityState objects.
        This method reads the yaml file from the provided path and converts it into
        a list of EntityState objects.
        The data should contain the necessary information to create EntityState
        objects, such as name, pose, and other attributes.


        Args:
            yaml_path (Path): Path to the YAML file containing the entity state data.
        Returns:
            List[EntityState]: List of EntityState objects.
        """

        with open(yaml_path, "r") as file:
            data = yaml.safe_load(file)

        episodes = data.get("episodes", [])

        goals = data.get("goals", {})
        poses = data.get("poses", {})

        robot_name = data.get("robot_name", "robot")
        self.goal_name = data.get("goal_name", "goal_box")

        for episode in episodes:
            goal = goals.get(episode, [])
            pose = poses.get(episode, [])

            goal_entity = EntityState()
            goal_entity.name = self.goal_name
            goal_entity.pose = Pose()
            goal_entity.pose.position.x = goal[0]
            goal_entity.pose.position.y = goal[1]

            initial_state_entity = EntityState()
            initial_state_entity.name = robot_name
            initial_state_entity.pose = Pose()

            # Position
            initial_state_entity.pose.position.x = pose[0]
            initial_state_entity.pose.position.y = pose[1]
            initial_state_entity.pose.position.z = 0.07
            # Orientation
            quaternion = quaternion_from_euler(0, 0, pose[2])
            initial_state_entity.pose.orientation.x = quaternion[0]
            initial_state_entity.pose.orientation.y = quaternion[1]
            initial_state_entity.pose.orientation.z = quaternion[2]
            initial_state_entity.pose.orientation.w = quaternion[3]
            # Reference frame

            self.goal_entities[episode] = goal_entity
            self.initial_state_entities[episode] = initial_state_entity
        return {
            "initial_state_entities": self.initial_state_entities,
            "goal_entities": self.goal_entities,
        }
