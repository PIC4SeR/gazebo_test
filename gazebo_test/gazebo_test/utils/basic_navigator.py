from enum import Enum
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action import (
    FollowPath,
    NavigateToPose,
)
from nav2_msgs.action import SmoothPath
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap, ManageLifecycleNodes

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class BasicNavigator:

    def __init__(self, node: Node):
        self.node = node

        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        # for now allow only the nav_to_pose action
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")
        self.compute_path_to_pose_client = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose"
        )
        self.smoother_client = ActionClient(self, SmoothPath, "smooth_path")

        # can be useful in the future
        # self.change_maps_srv = self.create_client(LoadMap, 'map_server/load_map')

        self.clear_costmap_global_srv = self.create_client(
            ClearEntireCostmap, "global_costmap/clear_entirely_global_costmap"
        )
        self.clear_costmap_local_srv = self.create_client(
            ClearEntireCostmap, "local_costmap/clear_entirely_local_costmap"
        )
        self.get_costmap_global_srv = self.create_client(
            GetCostmap, "global_costmap/get_costmap"
        )
        self.get_costmap_local_srv = self.create_client(
            GetCostmap, "local_costmap/get_costmap"
        )

    def destroyNode(self):
        """Destroy the node and all action clients."""
        self.destroy_node()

    def destroy_node(self):
        """Destroy the node and all action clients."""
        self.nav_to_pose_client.destroy()
        self.follow_path_client.destroy()
        self.compute_path_to_pose_client.destroy()
        self.smoother_client.destroy()
        super().destroy_node()

    def goToPose(self, pose: PoseStamped, behavior_tree: str = "") -> bool:
        """Send a `NavToPose` action request.
        Args:
            pose (PoseStamped): The target pose to navigate to.
            behavior_tree (str): The behavior tree to use for navigation.
        Returns:
            bool: True if the goal was accepted, False otherwise."""
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info(
            "Navigating to goal: "
            + str(pose.pose.position.x)
            + " "
            + str(pose.pose.position.y)
            + "..."
        )
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error(
                "Goal to "
                + str(pose.pose.position.x)
                + " "
                + str(pose.pose.position.y)
                + " was rejected!"
            )
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def followPath(
        self, path: Path, controller_id: str = "", goal_checker_id: str = ""
    ) -> bool:
        """Send a `FollowPath` action request.
        Args:
            path (Path): The path to follow.
            controller_id (str): The controller ID to use.
            goal_checker_id (str): The goal checker ID to use.
        Returns:
            bool: True if the goal was accepted, False otherwise."""

        self.debug("Waiting for 'FollowPath' action server")
        while not self.follow_path_client.wait_for_server(timeout_sec=1.0):
            self.info("'FollowPath' action server not available, waiting...")

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = controller_id
        goal_msg.goal_checker_id = goal_checker_id

        self.info("Executing path...")
        send_goal_future = self.follow_path_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error("Follow path was rejected!")
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelTask(self):
        """Cancel pending task request of any type.
        This will cancel the current task and set the status to CANCELED.
        Returns:
            None
        """
        self.info("Canceling current task.")
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isTaskComplete(self) -> bool:
        """Check if the task request of any type is complete yet.
        This will check the status of the current task and set the status to
        CANCELED if the task was canceled.
        Returns:
            bool: True if the task is complete, False otherwise.
        """
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f"Task with failed with status code: {self.status}")
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug("Task succeeded!")
        return True

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self) -> TaskResult:
        """Get the pending action result message
        Returns:
            TaskResult: The result of the task.
        """
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def waitUntilNav2Active(self, navigator="bt_navigator"):
        """Block until the full navigation system is up and running."""
        self._waitForNodeToActivate(navigator)
        self.info("Nav2 is ready for use!")
        return

    def _getPathImpl(self, start, goal, planner_id="", use_start=False):
        """
        Send a `ComputePathToPose` action request.

        Internal implementation to get the full result, not just the path.
        """
        self.debug("Waiting for 'ComputePathToPose' action server")
        while not self.compute_path_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'ComputePathToPose' action server not available, waiting...")

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = goal
        goal_msg.planner_id = planner_id
        goal_msg.use_start = use_start

        self.info("Getting path...")
        send_goal_future = self.compute_path_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error("Get path was rejected!")
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.warn(f"Getting path failed with status code: {self.status}")
            return None

        return self.result_future.result().result

    def getPath(self, start, goal, planner_id="", use_start=False):
        """Send a `ComputePathToPose` action request."""
        rtn = self._getPathImpl(start, goal, planner_id, use_start)
        if not rtn:
            return None
        else:
            return rtn.path

    def _smoothPathImpl(
        self, path, smoother_id="", max_duration=2.0, check_for_collision=False
    ):
        """
        Send a `SmoothPath` action request.

        Internal implementation to get the full result, not just the path.
        """
        self.debug("Waiting for 'SmoothPath' action server")
        while not self.smoother_client.wait_for_server(timeout_sec=1.0):
            self.info("'SmoothPath' action server not available, waiting...")

        goal_msg = SmoothPath.Goal()
        goal_msg.path = path
        goal_msg.max_smoothing_duration = rclpyDuration(seconds=max_duration).to_msg()
        goal_msg.smoother_id = smoother_id
        goal_msg.check_for_collisions = check_for_collision

        self.info("Smoothing path...")
        send_goal_future = self.smoother_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error("Smooth path was rejected!")
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.warn(f"Getting path failed with status code: {self.status}")
            return None

        return self.result_future.result().result

    def smoothPath(
        self, path, smoother_id="", max_duration=2.0, check_for_collision=False
    ):
        """Send a `SmoothPath` action request."""
        rtn = self._smoothPathImpl(path, smoother_id, max_duration, check_for_collision)
        if not rtn:
            return None
        else:
            return rtn.path

    # def changeMap(self, map_filepath):
    #     """Change the current static map in the map server."""
    #     while not self.change_maps_srv.wait_for_service(timeout_sec=1.0):
    #         self.info("change map service not available, waiting...")
    #     req = LoadMap.Request()
    #     req.map_url = map_filepath
    #     future = self.change_maps_srv.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     status = future.result().result
    #     if status != LoadMap.Response().RESULT_SUCCESS:
    #         self.error("Change map request failed!")
    #     else:
    #         self.info("Change map request was successful!")
    #     return

    def clearAllCostmaps(self):
        """Clear all costmaps."""
        self.clearLocalCostmap()
        self.clearGlobalCostmap()
        return

    def clearLocalCostmap(self):
        """Clear local costmap."""
        while not self.clear_costmap_local_srv.wait_for_service(timeout_sec=1.0):
            self.info("Clear local costmaps service not available, waiting...")
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_local_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return

    def clearGlobalCostmap(self):
        """Clear global costmap."""
        while not self.clear_costmap_global_srv.wait_for_service(timeout_sec=1.0):
            self.info("Clear global costmaps service not available, waiting...")
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return

    def lifecycleStartup(self):
        """Startup nav2 lifecycle system."""
        self.info("Starting up lifecycle nodes based on lifecycle_manager.")
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == "nav2_msgs/srv/ManageLifecycleNodes":
                self.info(f"Starting up {srv_name}")
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(f"{srv_name} service not available, waiting...")
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().STARTUP
                future = mgr_client.call_async(req)

                # starting up requires a full map->odom->base_link TF tree
                # so if we're not successful, try forwarding the initial pose
                while True:
                    rclpy.spin_until_future_complete(self, future, timeout_sec=0.10)
                    if not future:
                        self._waitForInitialPose()
                    else:
                        break
        self.info("Nav2 is ready for use!")
        return

    def lifecycleShutdown(self):
        """Shutdown nav2 lifecycle system."""
        self.info("Shutting down lifecycle nodes based on lifecycle_manager.")
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == "nav2_msgs/srv/ManageLifecycleNodes":
                self.info(f"Shutting down {srv_name}")
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(f"{srv_name} service not available, waiting...")
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().SHUTDOWN
                future = mgr_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                future.result()
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug(f"Waiting for {node_name} to become active..")
        node_service = f"{node_name}/get_state"
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f"{node_service} service not available, waiting...")

        req = GetState.Request()
        state = "unknown"
        while state != "active":
            self.debug(f"Getting {node_name} state...")
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug(f"Result of get_state: {state}")
            time.sleep(2)
        return

    def _feedbackCallback(self, msg):
        self.debug("Received action feedback message")
        self.feedback = msg.feedback
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return
