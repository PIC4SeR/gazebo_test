from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from gazebo_test.utils.basic_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg import PoseStamped, Pose
from typing import Optional
from rclpy.logging import LoggingSeverity
import asyncio


class NavigationHandler:
    """
    A class to handle the navigation stack in a ROS2 simulation environment.
    It provides methods to reset the navigation stack and manage its lifecycle.
    """

    def __init__(self, node: Node, success_callback=None) -> None:
        self.node = node
        self._loop = None
        self.navigator = BasicNavigator(
            node=self.node,
            # success_callback=success_callback,
        )
        self.logger = rclpy.logging.get_logger("navigation_handler")
        self.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.navigation_task = None
        self._default_success_callback = success_callback

    async def initialize_navigation(self) -> None:
        """
        Initializes the navigation stack and verifies its status.
        """
        # add get status of the nodes
        loop = asyncio.get_running_loop()
        self._loop = loop
        self.navigator.setLoop(loop)
        node_state = await self.navigator.checkNodeState("bt_navigator")
        self.logger.debug(f"Node state: {node_state}")
        if node_state != "active":
            self.logger.debug("Nav2 is not active, starting it ...")
            await self.navigator.lifecycleStartup()
            self.logger.debug("Waiting for Nav2 to be active ...")
        await self.navigator.waitUntilNav2Active()

    def pause_navigation(self) -> None:
        """
        Pauses the navigation stack by sending a request to the lifecycle manager.

        This method creates a client for the ManageLifecycleNodes service and sends
        a request to pause the navigation stack. It waits for the response and logs
        the result.
        """
        pass

    async def reset_navigation(self) -> None:
        """
        Resets the navigation stack by sending a request to the lifecycle manager.

        This method creates a client for the ManageLifecycleNodes service and sends
        a request to reset the navigation stack. It waits for the response and logs
        the result.
        """
        self.logger.debug("Resetting navigation stack ...")
        # clear all the navigation tasks
        if self.navigation_task:
            self.navigation_task.cancel()
            self.wait_for_events_task.cancel()
            self.navigator.clearEvents()
        # clear the navigator event
        self.navigator.clearEvents()
        await self.navigator.lifecycleReset()
        await self.navigator.lifecycleStartup()
        await self.navigator.waitUntilNav2Active()
        self.logger.debug("Resetting navigation stack ...")

    def start_navigation_task(
        self,
        goal_pose: Pose,
        frame_id: str = "map",
        success_callback: Optional[callable] = None,
    ) -> None:
        """
        Starts the navigation task by sending a goal to the navigator.
        This method creates a PoseStamped message with the goal pose and sends it to the navigator.
        It also creates a task to handle the navigation process.

        Parameters:
        - goal_pose (Pose): The goal pose to navigate to.
        - frame_id (str): The frame ID for the goal pose. Default is "map".
        - success_callback (callable): Optional callback function to be called on success.
        """

        nav_goal_pose = PoseStamped()
        nav_goal_pose.header.frame_id = frame_id
        nav_goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        nav_goal_pose.pose = goal_pose

        # launch the navigation task
        self.navigation_task = asyncio.create_task(
            self.navigator.goToPose(nav_goal_pose)
        )
        self.logger.debug("Navigation task started")

        # add a callback whenever one of the events is triggered
        self.wait_for_events_task = asyncio.create_task(
            self.wait_for_events(success_callback)
        )

        # if success_callback:
        #     self.navigation_task.add_done_callback(success_callback)
        # else:
        #     self.navigation_task.add_done_callback(self._default_success_callback)

    async def wait_for_events(self, callback: Optional[callable] = None) -> None:
        """
        Waits for navigation events to be triggered.
        This method subscribes to the navigation events and waits for them to be triggered.
        """

        # Wrap all event.wait() calls into tasks

        # Wait until any one of them is triggered
        done, pending = await asyncio.wait(
            [event.wait() for event in self.navigator.getEvents()],
            return_when=asyncio.FIRST_COMPLETED,
        )

        # # Cancel the remaining tasks
        for task in pending:
            task.cancel()
        self.logger.debug("Event triggered")
        # Call the callback function with the triggered event
        if callback:
            callback()
        else:
            self._default_success_callback()

    async def shutdown_navigation(self) -> None:
        """
        Shuts down the navigation stack by sending a request to the lifecycle manager.

        This method creates a client for the ManageLifecycleNodes service and sends
        a request to shut down the navigation stack. It waits for the response and logs
        the result.
        """
        self.logger.debug("Shutting down navigation stack ...")
        # clear all the navigation tasks

        # clear the navigator event
        self.navigator.clearEvents()
        await self.navigator.lifecycleShutdown()

    async def cancel_navigation(self) -> None:
        """
        Cancels the navigation task if it is running.
        This method checks if the navigation task is running and cancels it.
        """
        if self.navigation_task:
            self.logger.warn("Cancelling navigation task ...")
            await self.navigator.cancelGoToPose()
            self.navigation_task.cancel()
            self.wait_for_events_task.cancel()
            self.logger.warn("Navigation task cancelled")

    def resume_navigation(self) -> None:
        """
        Resumes the navigation stack by sending a request to the lifecycle manager.

        This method creates a client for the ManageLifecycleNodes service and sends
        a request to resume the navigation stack. It waits for the response and logs
        the result.
        """
        pass
