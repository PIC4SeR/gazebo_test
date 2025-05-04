from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from gazebo_test.utils.basic_navigator import BasicNavigator
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy


class NavigationHandler(Node):
    """
    A class to handle the navigation stack in a ROS2 simulation environment.
    It provides methods to reset the navigation stack and manage its lifecycle.
    """

    def __init__(self):
        super().__init__("navigation_handler")
        self._navigator = BasicNavigator()
        self._navigator.waitUntilNav2Active()
        self._client = self.create_client(
            ManageLifecycleNodes, "/navigate_to_pose/manage_lifecycle_nodes"
        )

    def pause_navigation(self) -> None:
        """
        Pauses the navigation stack by sending a request to the lifecycle manager.

        This method creates a client for the ManageLifecycleNodes service and sends
        a request to pause the navigation stack. It waits for the response and logs
        the result.
        """
        pass

    def reset_navigation(self) -> None:
        """
        Resets the navigation stack by sending a request to the lifecycle manager.

        This method creates a client for the ManageLifecycleNodes service and sends
        a request to reset the navigation stack. It waits for the response and logs
        the result.
        """
        pass

    def shutdown_navigation(self) -> None:
        """
        Shuts down the navigation stack by sending a request to the lifecycle manager.

        This method creates a client for the ManageLifecycleNodes service and sends
        a request to shut down the navigation stack. It waits for the response and logs
        the result.
        """
        pass

    def resume_navigation(self) -> None:
        """
        Resumes the navigation stack by sending a request to the lifecycle manager.

        This method creates a client for the ManageLifecycleNodes service and sends
        a request to resume the navigation stack. It waits for the response and logs
        the result.
        """
        pass
