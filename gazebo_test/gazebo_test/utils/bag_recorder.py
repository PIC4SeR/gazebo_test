import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String
from gazebo_collision_msgs.msg import Collision
from geometry_msgs.msg import Pose, PoseStamped, Twist
from people_msgs.msg import People
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from hunav_msgs.msg import Agents
from tf2_msgs.msg import TFMessage

import os
from pathlib import Path
import time

import rosbag2_py

topic_dict = {
    "/cmd_vel": Twist,
    "/front/scan": LaserScan,
    "/goal_pose": PoseStamped,
    "/human_states": Agents,
    "/hunav_goal_pose": PoseStamped,
    "/jackal/ground_truth": Odometry,
    "/jackal_velocity_controller/cmd_vel_unstamped": Twist,
    "/jackal_velocity_controller/odom": Odometry,
    "/jackal/collision": Collision,
    "/people": People,
    "/plan": NavPath,
    "/tf": TFMessage,
    "/tf_static": TFMessage,
}


class BagRecorder:
    def __init__(self, node: Node, algorithm: str = "test", base_path: str = ""):
        """
        Initialize the BagRecorder class.
        Args:
            node (Node): The ROS2 node to which the BagRecorder is attached.
            algorithm (str): The name of the algorithm being tested.
            base_path (str): The base path for storing bag files.
        """
        self.base_path = base_path
        self.date = time.strftime("%d_%m_%Y__%H_%M_%S")
        if not self.base_path:
            self.base_path = Path("bags/gazebo_test/exp_{self.date}/")
        self.node = node
        self.writer = rosbag2_py.SequentialWriter()
        self.recording = False
        self.logger = rclpy.logging.get_logger("bag_recorder")
        self.get_clock = node.get_clock
        self.algorithm = algorithm
        self.experiment_name = ""
        self.topics_metadata = []
        for topic_name, msg_type in topic_dict.items():
            # Create a TopicMetadata object for each topic
            self.topics_metadata.append(
                rosbag2_py.TopicMetadata(
                    name=topic_name,
                    type=f"{msg_type.__module__.split('.')[0]}/msg/{msg_type.__name__}",
                    serialization_format="cdr",
                )
            )
            self.node.create_subscription(
                msg_type=msg_type,
                topic=topic_name,
                callback=lambda msg, topic_name=topic_name: self.topic_callback(
                    msg, topic_name
                ),
                qos_profile=10,
            )
        self.topics_metadata.append(
            rosbag2_py.TopicMetadata(
                name="/experiment_result",
                type="std_msgs/msg/String",
                serialization_format="cdr",
            )
        )
        # set the logger level of rosbag2_storage to warn
        rosbag2_logger = rclpy.logging.get_logger("rosbag2_storage")
        rosbag2_logger.set_level(rclpy.logging.LoggingSeverity.WARN)

    def start_recording(self, experiment_name: str, run_id: str):
        if self.recording:
            self.logger.warn("Recording is already in progress.")
            return
        self.logger.debug("Starting recording...")
        self.bag_path = Path(
            f"{self.base_path}/{experiment_name}/{self.algorithm}/experiment_{run_id}/"
        )
        self.logger.debug(f"Bag path: {self.bag_path}")

        storage_options = rosbag2_py.StorageOptions(
            uri=str(self.bag_path), storage_id="sqlite3"
        )
        converter_options = rosbag2_py.ConverterOptions("", "")
        self.writer.open(storage_options, converter_options)

        # Create topics
        for topic_info in self.topics_metadata:
            self.writer.create_topic(topic_info)

        self.recording = True
        self.logger.debug(f"Started recording to {self.bag_path}")

    def stop_recording(self):
        if not self.recording:
            self.logger.warn("No recording is in progress.")
            return

        self.writer.close()
        self.recording = False
        self.logger.debug(f"Stopped recording to {self.bag_path}")

    def topic_callback(self, msg, topic_name):
        """
        Callback function for the topic subscription.
        This function is called whenever a message is received on the subscribed topic.
        It serializes the message and writes it to the bag file.
        Args:
            msg: The message received on the topic.
            topic_name: The name of the topic.
        """
        # Check if recording is in progress
        # and if the topic is in the topic_dict
        if topic_name not in topic_dict:
            self.logger.warn(f"Topic {topic_name} is not in the topic_dict.")
            return
        if not self.recording:
            return
        # Serialize the message
        serialized_msg = serialize_message(msg)
        # Write the message to the bag file
        self.writer.write(
            topic_name, serialized_msg, self.get_clock().now().nanoseconds
        )

    def set_experiment_result(self, result: str):
        """
        Set the result of the experiment.
        Args:
            result: The result of the experiment.
        """
        self.writer.write(
            "/experiment_result",
            serialize_message(String(data=result)),
            self.get_clock().now().nanoseconds,
        )
        self.logger.debug(f"Experiment result: {result}")

    def set_result_and_stop_recording(self, result: str):
        """
        Set the result of the experiment and stop recording.
        Args:
            result: The result of the experiment.
        """
        self.set_experiment_result(result)
        self.stop_recording()
