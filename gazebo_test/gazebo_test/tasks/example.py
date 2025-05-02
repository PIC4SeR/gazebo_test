from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

from gazebo_test.utils.gazebo_env_handler import GazeboEnvironmentHandler
from gazebo_msgs.msg import EntityState
from std_srvs.srv import Empty
from rclpy.task import Future
from geometry_msgs.msg import Pose
from typing import Optional, List, Dict, Any
from pathlib import Path
import asyncio
from tf_transformations import quaternion_from_euler
import time

# import yaw to quaternion


import yaml


class ExperimentManager(Node):
    def __init__(self):
        super().__init__("experiment_manager")
        self.experiment = None
        self.gazebo_env_handler = GazeboEnvironmentHandler(self)
        # Initialize other necessary components

        # wait for the gazebo env handler to be ready

        self.agent_entities: Dict[str, EntityState] = {}
        self.initial_state_entities: Dict[str, EntityState] = {}
        self.goal_entities: Dict[str, EntityState] = {}

        self.parse_entity_state_yaml(
            Path(
                "/workspaces/hunavsim_ws/src/gazebo_test/gazebo_test/goals_and_poses/social_nav.yaml"
            )
        )
        self.end = False
        self.get_logger().info("ExperimentManager initialized")

    async def initialize(self):
        # Placeholder for initialization logic
        self.get_logger().info("Initializing experiment ...")
        # Initialize the experiment manager
        await self.gazebo_env_handler.wait_for_gazebo_ready()
        await self.gazebo_env_handler.pause_gazebo()

    async def run_experiments(self):
        """
        Run the experiments sequentially.
        This method will run the experiments for each episode defined in the YAML file.
        """
        self.get_logger().info("Running experiments ...")
        for episode in self.initial_state_entities.keys():
            await self.run_experiment(episode)

    async def run_experiment(self, episode: str = "episode_1"):
        """
        Run the experiment for a given episode.
        Args:
            episode (str): The episode to run.
        """
        self.get_logger().info(f"Running experiment for {episode} ...")

        # Placeholder for experiment running logic
        self.get_logger().info("Running experiment...")
        # reset the environment

        entities_to_reset = [
            self.initial_state_entities[episode],
            self.goal_entities[episode],
        ] + list(self.agent_entities.values())

        await self.gazebo_env_handler.reset_environment_for_experiment(
            entities_to_reset
        )

        self.get_logger().info("Environment reset successfully")

        time.sleep(10)  # Placeholder for waiting logic

        # Syncronously wait for the environment to be ready

        # get the initial state of the environment from goal and poses
        #
        # experiment = ...  # Placeholder for the experiment object
        # # set the initial state of the environment using set_state
        # self.gazebo_env_handler.set_entities_state(experiment.initial_state)
        # # Syncronously wait for the environment to be ready

        # # start the experiment
        # self.gazebo_env_handler.resume_gazebo()
        # Syncronously wait for the environment to be ready and the experiment to start
        # Placeholder for waiting logic
        # Placeholder for experiment running logic
        # set a timeout for the experiment

    def stop_experiment(self):
        # Placeholder for stopping the experiment
        print("Stopping experiment...")

    # def json_to_entity_state(self, yaml_path: Path) -> List[EntityState]:
    def parse_entity_state_yaml(self, yaml_path: Path) -> List[EntityState]:
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
        agents = data.get("agents", {})

        reference_frame = data.get("reference_frame", "map")
        robot_name = data.get("robot_name", "robot")

        for episode in episodes:
            goal = goals.get(episode, [])
            pose = poses.get(episode, [])

            goal_entity = EntityState()
            goal_entity.name = f"goal"
            goal_entity.pose = Pose()
            goal_entity.pose.position.x = goal[0]
            goal_entity.pose.position.y = goal[1]
            # goal_entity.reference_frame = reference_frame

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
            # initial_state_entity.reference_frame = reference_frame

            self.goal_entities[episode] = goal_entity
            self.initial_state_entities[episode] = initial_state_entity

        for key, agent in agents.items():
            agent_entity = EntityState()
            agent_entity.name = key
            agent_entity.pose = Pose()
            # Position
            agent_entity.pose.position.x = agent[0]
            agent_entity.pose.position.y = agent[1]
            agent_entity.pose.position.z = 1.5
            # Orientation
            quaternion = quaternion_from_euler(0, 0, agent[2])
            agent_entity.pose.orientation.x = quaternion[0]
            agent_entity.pose.orientation.y = quaternion[1]
            agent_entity.pose.orientation.z = quaternion[2]
            agent_entity.pose.orientation.w = quaternion[3]
            # Reference frame
            # agent_entity.reference_frame = reference_frame

            self.agent_entities[key] = agent_entity

            self.get_logger().info(f"Agent {key} name: {agent_entity.name}")
