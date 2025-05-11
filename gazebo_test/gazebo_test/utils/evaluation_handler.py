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
from rclpy.logging import LoggingSeverity


class ExperimentResult(Enum):
    SUCCESS = 1
    FAILURE_COLLISION_ENVIRONMENT = 2
    FAILURE_COLLISION_AGENT = 3
    FAILURE_TIMEOUT = 4
    RESULT_NOT_SET = 5

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
            case ExperimentResult.RESULT_NOT_SET:
                return "Result not set"


class ExperimentEvaluator:
    def __init__(self, node: Node, timeout_duration: float = 40.0):
        self.timeout_duration = timeout_duration
        self.start_time = None
        self.node = node
        self._loop = None

        node.create_subscription(
            Collision, "/jackal/collision", self._collision_callback, 10
        )
        self.logger = rclpy.logging.get_logger("experiment_evaluator")
        self.get_clock = node.get_clock
        self.timeout_timer = None
        # self.logger.set_level(LoggingSeverity.DEBUG)

    def initialize(self):
        """
        Initialize the experiment evaluator.
        This method is called to set up the evaluator before running the experiment.
        """

        # use asyncio event to notify the experiment that a collision has been detected
        self.success_event = asyncio.Event()
        self.collision_event = asyncio.Event()
        self.timeout_event = asyncio.Event()

        self.experiment_result = ExperimentResult.RESULT_NOT_SET
        self._loop = asyncio.get_running_loop()
        self.logger.debug("Experiment evaluator initialized")

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
        # loop = asyncio.get_running_loop()
        # self._loop = loop
        # unset all events
        self.collision_event.clear()
        self.success_event.clear()
        self.timeout_event.clear()

        self.start_time = self.get_clock().now()
        self.experiment_result = ExperimentResult.RESULT_NOT_SET
        self.logger.debug("Experiment started")
        self.logger.debug(f"Timeout set to {self.timeout_duration} seconds", once=True)
        self.logger.debug(f"Start time: {self.start_time}")
        self._create_timeout_timer()

        # Wait for one of: collision_event, success_event, or timeout
        done, pending = await asyncio.wait(
            [
                self.collision_event.wait(),
                self.success_event.wait(),
                self.timeout_event.wait(),
            ],
            return_when=asyncio.FIRST_COMPLETED,
        )
        self.logger.debug(f"Finish time:{self.get_clock().now()}")
        self.logger.debug(f"Elapsed time: {self.get_clock().now() - self.start_time}")
        # Cancel any pending tasks
        for task in pending:
            task.cancel()

        if self.timeout_timer:
            self.timeout_timer.cancel()

        return self.experiment_result

    def _on_timeout(self):
        """ROS Timer callback: timeout reached based on sim time."""
        self.experiment_result = ExperimentResult.FAILURE_TIMEOUT
        self._loop.call_soon_threadsafe(self.timeout_event.set)
        self.logger.info("Timeout reached")

    def _collision_callback(self, msg: Collision):
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
        if not self.start_time:
            self.logger.debug("Collision message arrived before start time")
            return
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
            self.experiment_result = ExperimentResult.FAILURE_COLLISION_AGENT
            self.logger.info("Collision with agent detected")
        else:
            self.experiment_result = ExperimentResult.FAILURE_COLLISION_ENVIRONMENT
            self.logger.info("Collision with environment detected")
        # Stop the experiment
        self.logger.debug("Stopping the experiment")
        # notify the experiment that a collision has been detected using asyncio event
        self._loop.call_soon_threadsafe(self.collision_event.set)

    def _create_timeout_timer(self):
        """Create a timer to check for timeout."""
        if self.timeout_timer:
            self.timeout_timer.reset()
            return
        self.timeout_timer = self.node.create_timer(
            self.timeout_duration,
            self._on_timeout,
        )

    def set_success_event(self):
        """Set the success event."""
        self.experiment_result = ExperimentResult.SUCCESS
        self._loop.call_soon_threadsafe(self.success_event.set)
        self.logger.debug("Success event set")
