# use the collision sensor to determine the collision
# if a collision is detected, the navigation is considered a failure
# distinguish between collision with the environment and collision with agents


# use the pose and the goal to determine success of the navigation
# if a timeout is reached, the navigation is considered a failure

import os
from rclpy.node import Node
from enum import Enum
import rclpy
from rclpy.time import Time
import asyncio
from gazebo_collision_msgs.msg import Collision


class ExperimentResult(Enum):
    SUCCESS = 1
    FAILURE_COLLISION_ENVIRONMENT = 2
    FAILURE_COLLISION_AGENT = 3
    FAILURE_TIMEOUT = 4
    FAILURE_UNKNOWN = 5

    def __str__(self):
        match self:
            case ExperimentResult.SUCCESS:
                return "Success"
            case ExperimentResult.FAILURE_COLLISION_ENVIRONMENT:
                return "Failure: Collision with environment"
            case ExperimentResult.FAILURE_COLLISION_AGENT:
                return "Failure: Collision with agent"
            case ExperimentResult.FAILURE_TIMEOUT:
                return "Failure: Timeout"
            case ExperimentResult.FAILURE_UNKNOWN:
                return "Failure: Unknown error"
            case _:
                return "Unknown result"


class ExperimentEvaluator:
    def __init__(self, node: Node, timeout_duration: float = 40.0):
        self.timeout_duration = timeout_duration
        self.start_time = None
        self.node = node
        self._loop = None
        # use asyncio event to notify the experiment that a collision has been detected

        self.success_event = asyncio.Event()
        self.collision_event = asyncio.Event()
        self.timeout_event = asyncio.Event()
        self.collision_with_agent = False

        self.collision_with_environment = False
        self.is_timeout = False

        node.create_subscription(
            Collision, "/jackal/collision", self.collision_callback, 10
        )
        self.logger = rclpy.logging.get_logger("experiment_evaluator")
        self.get_clock = node.get_clock
        self.timeout_timer = None
        self.is_running = False

    async def run_experiment(self) -> ExperimentResult:
        """
        Run the experiment and return the result.
        The experiment is considered successful if the robot reaches the goal
        without any collisions and within the timeout duration.

        The experiment is considered a failure if:
        - A collision is detected with an agent or the environment
        - The timeout duration is reached
        Returns:
        - ExperimentResult: The result of the experiment
        """
        self.is_running = True
        loop = asyncio.get_running_loop()
        self._loop = loop
        self.start_time = self.get_clock().now()
        self.collision_with_agent = False
        self.collision_with_environment = False
        self.is_timeout = False
        self.logger.info("Experiment started")
        self.logger.info(f"Timeout set to {self.timeout_duration} seconds", once=True)
        self.logger.info(f"Start time: {self.start_time}")
        self.create_timeout_timer()

        # Wait for one of: collision_event, success_event, or timeout
        done, pending = await asyncio.wait(
            [
                self.collision_event.wait(),
                self.success_event.wait(),
                self.timeout_event.wait(),
            ],
            return_when=asyncio.FIRST_COMPLETED,
        )
        self.is_running = False
        self.logger.info(f"Finish time:{self.get_clock().now()}")
        self.logger.info(f"Elapsed time: {self.get_clock().now() - self.start_time}")
        # Cancel any pending tasks
        for task in pending:
            task.cancel()

        if self.timeout_timer:
            self.timeout_timer.cancel()

        # unset all events
        self.collision_event.clear()
        self.success_event.clear()
        self.timeout_event.clear()

        experiment_evaluation = self.evaluate_experiment()
        return experiment_evaluation

    def _on_timeout(self):
        """ROS Timer callback: timeout reached based on sim time."""
        self.is_timeout = True
        # Wake up experiment coroutine
        self._loop.call_soon_threadsafe(self.timeout_event.set)
        self.logger.info("Timeout reached")

    def collision_callback(self, msg: Collision):
        """Callback for the collision sensor.
        If a collision is detected, set the collision_event.
        Then verify if the collision is with an agent or the environment.
        If the collision is with an agent, set the collision_with_agent flag to True.
        If the collision is with the environment, set the collision_with_environment flag to True.
        Finally, stop the experiment and notify the experiment that a collision has been detected using asyncio event.
        Args:
            msg (Collision): The collision message from the collision sensor.
        """
        # assure that the message is arrived after the start time
        self.logger.debug(
            f"Collision message arrived at: {Time.from_msg(msg.header.stamp) - self.start_time}"
        )
        if Time.from_msg(msg.header.stamp) <= self.start_time:
            self.logger.debug("Collision message arrived before start time")
            return

        if self.collision_event.is_set():
            return
        # make sure this event is only called once
        self.logger.debug("Collision detected")
        self.logger.debug(f"Collision with: {msg.objects_hit}")
        self.logger.debug(f"Collision time: {msg.header.stamp}")
        if "agent" in msg.objects_hit[0]:
            self.collision_with_agent = True
            self.logger.debug("Collision with agent detected")
        else:
            self.collision_with_environment = True
            self.logger.info("Collision with environment detected")
        # Stop the experiment
        self.logger.debug("Stopping the experiment")
        # notify the experiment that a collision has been detected using asyncio event
        self._loop.call_soon_threadsafe(self.collision_event.set)

    def create_timeout_timer(self):
        """Create a timer to check for timeout."""
        if self.timeout_timer:
            self.timeout_timer.reset()
            return
        self.timeout_timer = self.node.create_timer(
            self.timeout_duration,
            self._on_timeout,
        )

    def evaluate_experiment(self) -> ExperimentResult:
        """Evaluate the experiment and return the result."""
        if self.is_timeout:
            return ExperimentResult.FAILURE_TIMEOUT
        if self.collision_with_agent:
            return ExperimentResult.FAILURE_COLLISION_AGENT
        elif self.collision_with_environment:
            return ExperimentResult.FAILURE_COLLISION_ENVIRONMENT
        return ExperimentResult.SUCCESS
