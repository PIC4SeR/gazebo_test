"""Collision labels: which body a robot's contact sensor hit.

The cases at the top are verbatim from the 2026-09-10 sweep's bags, where the
old classifier scored 5 of 8 robot-robot contacts as environment collisions:
a sensor sometimes reports its OWN model, and "turtlebot2" is a prefix of
"turtlebot2_1". Needs a sourced ROS environment (gazebo_collision_msgs).
"""

import asyncio
import pathlib
import sys
import types

from builtin_interfaces.msg import Time as TimeMsg
from gazebo_collision_msgs.msg import Collision

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from gazebo_test.utils.evaluation_handler import (  # noqa: E402
    ExperimentEvaluator,
    ExperimentResult,
    classify_contact,
)

AGENT = ExperimentResult.FAILURE_COLLISION_AGENT
ROBOT = ExperimentResult.FAILURE_COLLISION_ROBOT
ENVIRONMENT = ExperimentResult.FAILURE_COLLISION_ENVIRONMENT
MIXED_FLEET = {"jackal", "turtlebot2", "turtlebot2_1"}
TB2_BASE = "turtlebot2::base_footprint::base_footprint_fixed_joint_lump__base_collision"
JACKAL_BASE = ("jackal::base_link::"
               "base_link_fixed_joint_lump__collision_link_collision_1")


def test_a_sensor_reporting_its_own_body_is_not_an_environment_collision():
    # /turtlebot2/collision during a TB2-TB2 contact: only its own body.
    assert classify_contact("turtlebot2", [TB2_BASE], MIXED_FLEET) == ENVIRONMENT
    # The partner's sensor names it, and that is a robot contact.
    assert classify_contact("turtlebot2_1", [TB2_BASE], MIXED_FLEET) == ROBOT
    assert classify_contact("turtlebot2_1", [JACKAL_BASE], MIXED_FLEET) == ROBOT
    assert classify_contact("jackal", [JACKAL_BASE], MIXED_FLEET) == ENVIRONMENT


def test_robot_names_match_exactly_not_by_prefix():
    hit = "turtlebot2_1::base_footprint::collision"
    # The old substring rule saw "turtlebot2" in this and skipped nothing.
    assert classify_contact("turtlebot2_1", [hit], MIXED_FLEET) == ENVIRONMENT
    assert classify_contact("turtlebot2", [hit], MIXED_FLEET) == ROBOT
    assert classify_contact("turtlebot2", ["turtlebot2_10::base::c"],
                            MIXED_FLEET) == ENVIRONMENT


def test_pedestrians_walls_and_mixed_contacts():
    assert classify_contact("jackal", ["agent3_body::base_link::collision"],
                            MIXED_FLEET) == AGENT
    assert classify_contact("jackal", ["agent2::link::collision"],
                            MIXED_FLEET) == AGENT
    assert classify_contact("jackal", ["corridor::Wall_1::Wall_1_Collision"],
                            MIXED_FLEET) == ENVIRONMENT
    # Every entry counts, not just the first; the most severe wins.
    assert classify_contact(
        "jackal",
        ["corridor::Wall_1::c", TB2_BASE, "agent1_body::base_link::collision"],
        MIXED_FLEET) == AGENT
    assert classify_contact("jackal", [JACKAL_BASE, "corridor::Wall_1::c", TB2_BASE],
                            MIXED_FLEET) == ROBOT
    assert classify_contact("jackal", [], MIXED_FLEET) == ENVIRONMENT


class _FakeNode:
    def __init__(self):
        self.subscriptions = {}

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions[topic] = callback
        return topic

    def destroy_subscription(self, sub):
        self.subscriptions.pop(sub)

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: _time(0))


def _time(sec):
    # ROS time, as the node's clock and Time.from_msg both are.
    from rclpy.clock import ClockType
    from rclpy.time import Time
    return Time(seconds=sec, clock_type=ClockType.ROS_TIME)


def _collision(sec, *objects):
    msg = Collision()
    msg.header.stamp = TimeMsg(sec=sec)
    msg.objects_hit = list(objects)
    return msg


def test_any_watched_robot_can_end_the_episode():
    """The evaluator watches a list of robots; a contact is judged against the
    robot that reported it, with the others as its possible partners."""
    node = _FakeNode()
    evaluator = ExperimentEvaluator(node)
    assert node.subscriptions == {}              # nothing until robots are named
    evaluator.watch_robots(["turtlebot2", "turtlebot2_1"])
    assert set(node.subscriptions) == {"/turtlebot2/collision",
                                       "/turtlebot2_1/collision"}

    async def contact():
        evaluator.initialize()
        evaluator.start_time = _time(1)
        node.subscriptions["/turtlebot2_1/collision"](_collision(5, TB2_BASE))
        await asyncio.wait_for(evaluator.collision_event.wait(), 1.0)
        return evaluator.experiment_result

    assert asyncio.run(contact()) == ROBOT
    # Re-pointing replaces the old subscriptions.
    evaluator.watch_robots(["jackal"])
    assert set(node.subscriptions) == {"/jackal/collision"}


def test_contacts_from_before_the_episode_are_ignored():
    node = _FakeNode()
    evaluator = ExperimentEvaluator(node)
    evaluator.watch_robots(["jackal"])

    async def stale():
        evaluator.initialize()
        evaluator.start_time = _time(10)
        node.subscriptions["/jackal/collision"](_collision(5, TB2_BASE))
        await asyncio.sleep(0.01)
        return evaluator.collision_event.is_set()

    assert asyncio.run(stale()) is False


if __name__ == "__main__":
    for name, case in sorted(globals().items()):
        if name.startswith("test_"):
            case()
            print(f"ok  {name}")
