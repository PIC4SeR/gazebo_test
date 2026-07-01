from enum import Enum

from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, ManageLifecycleNodes

from rclpy.action import ActionClient
from rclpy.node import Node
import asyncio
from rclpy.logging import get_logger


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class BasicNavigator:

    def __init__(self, node: Node, namespace: str = ""):

        self.node = node
        self._ns = namespace.strip("/")
        self._loop = None

        self.go_to_pose_goal_handle = None
        self.go_to_pose_result = None
        self.feedback = None
        self.go_to_pose_status = GoalStatus.STATUS_UNKNOWN

        self.go_to_pose_event = asyncio.Event()

        # only the nav_to_pose action is used
        self.nav_to_pose_client = ActionClient(
            self.node, NavigateToPose, self._n("navigate_to_pose")
        )

        self.clear_costmap_global_srv = node.create_client(
            ClearEntireCostmap, self._n("global_costmap/clear_entirely_global_costmap")
        )
        self.clear_costmap_local_srv = node.create_client(
            ClearEntireCostmap, self._n("local_costmap/clear_entirely_local_costmap")
        )
        self.logger = get_logger("basic_navigator")
        # self.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

    def _n(self, name: str) -> str:
        """Prefix a topic/service/node name with this navigator's namespace."""
        if not self._ns:
            return name
        return f"/{self._ns}/{name.lstrip('/')}"

    def _owned_lifecycle_service(self, srv_name: str) -> bool:
        """True if a ManageLifecycleNodes service belongs to this namespace."""
        if not self._ns:
            return True
        return srv_name.startswith(f"/{self._ns}/")

    def destroy_node(self):
        """Destroy the node and all action clients."""
        self.nav_to_pose_client.destroy()
        super().destroy_node()

    def clearEvents(self):
        """Clear the events."""
        self.go_to_pose_event.clear()

    def getEvents(self):
        """Get the events."""
        return (self.go_to_pose_event,)

    async def goToPose(self, pose: PoseStamped, behavior_tree: str = "") -> bool:
        """Send a `NavToPose` action request.
        Args:
            pose (PoseStamped): The target pose to navigate to.
            behavior_tree (str): The behavior tree to use for navigation.
        Returns:
            bool: True if the goal was accepted, False otherwise."""

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.logger.debug(
            "Navigating to goal: "
            + str(pose.pose.position.x)
            + " "
            + str(pose.pose.position.y)
            + "..."
        )
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        self.go_to_pose_goal_handle = await send_goal_future
        if not self.go_to_pose_goal_handle.accepted:
            self.logger.error(
                "Goal to "
                + str(pose.pose.position.x)
                + " "
                + str(pose.pose.position.y)
                + " was rejected!"
            )
            return False

        self.go_to_pose_result = self.go_to_pose_goal_handle.get_result_async()
        self.go_to_pose_result.add_done_callback(self._go_to_pose_result_callback)
        return True

    async def cancelGoToPose(self):
        """Cancel pending task of the `NavToPose` action.
        This will cancel the current task and set the status to CANCELED.
        Returns:
            None
        """
        self.logger.debug("Canceling current task.")
        if self.go_to_pose_result:
            future = await self.go_to_pose_goal_handle.cancel_goal_async()

    def getResult(self, goal_result: GoalStatus) -> TaskResult:
        """Get the pending action result message
        Returns:
            TaskResult: The result of the task.
        """
        match goal_result.status:
            case GoalStatus.STATUS_SUCCEEDED:
                return TaskResult.SUCCEEDED
            case GoalStatus.STATUS_ABORTED:
                return TaskResult.FAILED
            case GoalStatus.STATUS_CANCELED:
                return TaskResult.CANCELED
            case _:
                return TaskResult.UNKNOWN

    async def waitUntilNav2Active(self, navigator="bt_navigator"):
        """Block until the full navigation system is up and running."""
        await self._waitForNodeToActivate(navigator)
        await self._waitForServer()
        self.logger.info("Nav2 is ready for use!", once=True)
        return

    async def waitUntilNavigateToPoseActive(self):
        """Block until a NavigateToPose-compatible action server is available."""
        await self._waitForActionServers([self.nav_to_pose_client])
        self.logger.info("NavigateToPose action server is ready for use!", once=True)
        return

    def setLoop(self, loop):
        """Set the loop for the navigator."""
        self._loop = loop
        self.logger.debug("Set loop for navigator")
        return

    async def clearAllCostmaps(self):
        """Clear all costmaps."""

        clear_local_future = asyncio.create_task(self.clearLocalCostmap())
        clear_global_future = asyncio.create_task(self.clearGlobalCostmap())
        await asyncio.gather(clear_local_future, clear_global_future)
        self.logger.info("Cleared all costmaps")
        return

    async def clearLocalCostmap(self):
        """Clear local costmap."""
        req = ClearEntireCostmap.Request()
        result = await self.clear_costmap_local_srv.call_async(req)
        if not result:
            self.logger.error("Failed to clear local costmap")
            return
        self.logger.debug("Cleared local costmap")
        return

    async def clearGlobalCostmap(self):
        """Clear global costmap."""
        req = ClearEntireCostmap.Request()
        result = await self.clear_costmap_global_srv.call_async(req)
        if not result:
            self.logger.error("Failed to clear global costmap")
            return
        self.logger.debug("Cleared global costmap")
        return

    async def _manageLifecycleNodes(self, command: int, verb: str) -> None:
        """Send `command` to every owned ManageLifecycleNodes service."""
        self.logger.debug(f"{verb} lifecycle nodes based on lifecycle_manager.")
        for srv_name, srv_type in self.node.get_service_names_and_types():
            if srv_type[0] == "nav2_msgs/srv/ManageLifecycleNodes" and self._owned_lifecycle_service(srv_name):
                self.logger.debug(f"{verb} {srv_name}")
                mgr_client = self.node.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.logger.debug(f"{srv_name} service not available, waiting...")
                req = ManageLifecycleNodes.Request()
                req.command = command
                result = await mgr_client.call_async(req)
                if not result:
                    self.logger.error(f"Failed to {verb.lower()} {srv_name}")
                    return
                self.logger.debug(f"{verb} {srv_name} done")

    async def lifecycleStartup(self):
        """Startup nav2 lifecycle system."""
        await self._manageLifecycleNodes(ManageLifecycleNodes.Request().STARTUP, "Starting up")
        self.logger.debug("Nav2 is ready for use!")

    async def lifecycleReset(self):
        """Reset nav2 lifecycle system."""
        await self._manageLifecycleNodes(ManageLifecycleNodes.Request().RESET, "Resetting")

    async def lifecycleShutdown(self):
        """Shutdown nav2 lifecycle system."""
        await self._manageLifecycleNodes(ManageLifecycleNodes.Request().SHUTDOWN, "Shutting down")
        self.logger.info("Nav2 is shut down!")

    async def checkNodeState(self, node_name):
        """Check the state of a node.
        Args:
            node_name (str): The name of the node to check.
        """
        self.logger.debug(f"Checking state of {node_name}...")
        node_service = self._n(f"{node_name}/get_state")
        state_client = self.node.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.logger.debug(f"{node_service} service not available, waiting...")

        req = GetState.Request()
        future = await state_client.call_async(req)
        return future.current_state.label

    async def _waitForNodeToActivate(self, node_name):
        """Wait for a node to become active.
        Args:
            node_name (str): The name of the node to wait for.
        """
        self.logger.debug(f"Waiting for {node_name} to become active..")
        node_service = self._n(f"{node_name}/get_state")
        state_client = self.node.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.logger.debug(f"{node_service} service not available, waiting...")

        req = GetState.Request()
        state = "unknown"
        while state != "active":
            self.logger.debug(f"Waiting for {node_name} to become active...")
            future = await state_client.call_async(req)
            state = future.current_state.label
            self.logger.debug(f"Result of get_state: {state}")
        return

    def _feedbackCallback(self, msg):
        # self.logger.debug("Received action feedback message")
        self.feedback = msg.feedback
        return

    async def _waitForServer(self):
        await self._waitForActionServers([self.nav_to_pose_client])
        await self._waitForCostmapServices()
        return

    async def _waitForActionServers(self, clients):
        for client in clients:
            while not client.wait_for_server(timeout_sec=1.0):
                self.logger.debug(
                    f"{client._action_name} action server not available, waiting..."
                )
        self.logger.debug("All action servers are available.")
        return

    async def _waitForCostmapServices(self):
        for srv in [
            self.clear_costmap_global_srv,
            self.clear_costmap_local_srv,
        ]:
            while not srv.wait_for_service(timeout_sec=1.0):
                self.logger.debug(f"{srv.srv_name} service not available, waiting...")
        self.logger.debug("All services are available.")
        return

    def _go_to_pose_result_callback(self, future):
        self.logger.debug("Received action result message")
        self.go_to_pose_result = future.result()
        self.go_to_pose_status = self.getResult(self.go_to_pose_result)
        self.logger.debug(f"Go to pose status: {self.go_to_pose_status}")
        self._loop.call_soon_threadsafe(self.go_to_pose_event.set)
