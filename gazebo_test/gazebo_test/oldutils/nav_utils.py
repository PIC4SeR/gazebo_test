#!/usr/bin/env python3
import time
import subprocess
import math
import numpy as np
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy


def check_navigation(navigator):
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Goal succeeded!")
    elif result == TaskResult.CANCELED:
        print("Goal was canceled!")
    elif result == TaskResult.FAILED:
        print("Goal failed!")
    elif result == TaskResult.UNKNOWN:
        print("Navigation Result UNKNOWN!")
    return result


def lifecyclePause(navigator: BasicNavigator):
    """Pause nav2 lifecycle system."""
    navigator.info("Pause lifecycle nodes based on lifecycle_manager.")
    for srv_name, srv_type in navigator.get_service_names_and_types():
        if srv_type[0] == "nav2_msgs/srv/ManageLifecycleNodes":
            navigator.info(f"Pause {srv_name}")
            mgr_client = navigator.create_client(ManageLifecycleNodes, srv_name)
            while not mgr_client.wait_for_service(timeout_sec=1.0):
                navigator.info(f"{srv_name} service not available, waiting...")
            req = ManageLifecycleNodes.Request()
            req.command = ManageLifecycleNodes.Request().PAUSE
            future = mgr_client.call_async(req)
            rclpy.spin_until_future_complete(navigator, future)
            future.result()
    return


def lifecycleResume(navigator: BasicNavigator):
    """Resume nav2 lifecycle system."""
    navigator.info("Resume lifecycle nodes based on lifecycle_manager.")
    for srv_name, srv_type in navigator.get_service_names_and_types():
        if srv_type[0] == "nav2_msgs/srv/ManageLifecycleNodes":
            navigator.info(f"Resume {srv_name}")
            mgr_client = navigator.create_client(ManageLifecycleNodes, srv_name)
            while not mgr_client.wait_for_service(timeout_sec=1.0):
                navigator.info(f"{srv_name} service not available, waiting...")
            req = ManageLifecycleNodes.Request()
            req.command = ManageLifecycleNodes.Request().RESUME
            future = mgr_client.call_async(req)
            rclpy.spin_until_future_complete(navigator, future)
            future.result()
    return


def lifecycleReset(navigator: BasicNavigator):
    """Reset nav2 lifecycle system."""
    navigator.info("Reset lifecycle nodes based on lifecycle_manager.")
    for srv_name, srv_type in navigator.get_service_names_and_types():
        if srv_type[0] == "nav2_msgs/srv/ManageLifecycleNodes":
            navigator.info(f"Reset {srv_name}")
            mgr_client = navigator.create_client(ManageLifecycleNodes, srv_name)
            while not mgr_client.wait_for_service(timeout_sec=1.0):
                navigator.info(f"{srv_name} service not available, waiting...")
            req = ManageLifecycleNodes.Request()
            req.command = ManageLifecycleNodes.Request().RESET
            future = mgr_client.call_async(req)
            rclpy.spin_until_future_complete(navigator, future)
            future.result()
    return
