"""BasicNavigator: one goal's result must never be read as another's.

In the 2026-09-10 sweep 8 of 90 runs ended ~1 s after they started as a
"Navigation failure": the previous run's goal, cancelled but not yet finished,
delivered its CANCELED result into the next run. These pin the fix with fake
goal handles -- no action server. Needs a sourced ROS environment (nav2_msgs).
"""

import asyncio
import pathlib
import sys
import time
import types

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from gazebo_test.utils.basic_navigator import BasicNavigator, TaskResult  # noqa: E402


class _FakeFuture:
    """The parts of rclpy.task.Future the navigator uses, awaitable like it."""

    def __init__(self, result=None, done=False):
        self._result, self._done, self._callbacks = result, done, []

    def done(self):
        return self._done

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        if self._done:
            callback(self)
        else:
            self._callbacks.append(callback)

    def complete(self, result):
        self._result, self._done = result, True
        for callback in self._callbacks:
            callback(self)

    def __await__(self):
        while not self._done:
            yield
        return self._result


def _response(status):
    return types.SimpleNamespace(status=status)


class _FakeHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.future = _FakeFuture()
        self.cancel_requests = 0
        self.end_on_cancel_after = None      # seconds, or None: never ends

    def get_result_async(self):
        return self.future

    def cancel_goal_async(self):
        self.cancel_requests += 1
        if self.end_on_cancel_after is not None:
            asyncio.get_running_loop().call_later(
                self.end_on_cancel_after, self.future.complete,
                _response(GoalStatus.STATUS_CANCELED))
        return _FakeFuture(result=None, done=True)


class _FakeClient:
    def __init__(self):
        self.handles = []

    def send_goal_async(self, goal, feedback_callback=None):
        return _FakeFuture(result=self.handles.pop(0), done=True)


class _Logger:
    def __init__(self):
        self.warnings = []

    def debug(self, *args, **kwargs):
        pass

    def info(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass

    def warning(self, message, *args, **kwargs):
        self.warnings.append(message)


def _navigator():
    nav = BasicNavigator.__new__(BasicNavigator)      # skip the ROS __init__
    nav.nav_to_pose_client = _FakeClient()
    nav.logger = _Logger()
    nav.go_to_pose_goal_handle = None
    nav.go_to_pose_future = None
    nav.go_to_pose_result = None
    nav.go_to_pose_status = TaskResult.UNKNOWN
    nav.go_to_pose_event = asyncio.Event()
    nav.feedback = None
    nav._loop = asyncio.get_running_loop()
    return nav


async def _goal(nav, handle):
    nav.nav_to_pose_client.handles.append(handle)
    assert await nav.goToPose(PoseStamped())


def test_a_previous_goals_late_result_is_ignored():
    async def scenario():
        nav = _navigator()
        first, second = _FakeHandle(), _FakeHandle()
        await _goal(nav, first)
        await _goal(nav, second)
        first.future.complete(_response(GoalStatus.STATUS_CANCELED))   # late
        await asyncio.sleep(0.01)
        assert not nav.go_to_pose_event.is_set()
        assert nav.go_to_pose_status == TaskResult.UNKNOWN
        second.future.complete(_response(GoalStatus.STATUS_SUCCEEDED))
        await asyncio.wait_for(nav.go_to_pose_event.wait(), 1.0)
        assert nav.go_to_pose_status == TaskResult.SUCCEEDED
    asyncio.run(scenario())


def test_a_new_goal_starts_with_no_result():
    async def scenario():
        nav = _navigator()
        nav.go_to_pose_status = TaskResult.CANCELED
        nav.go_to_pose_event.set()
        await _goal(nav, _FakeHandle())
        assert nav.go_to_pose_status == TaskResult.UNKNOWN
        assert not nav.go_to_pose_event.is_set()
    asyncio.run(scenario())


def test_cancel_waits_until_the_goal_has_ended():
    async def scenario():
        nav = _navigator()
        handle = _FakeHandle()
        handle.end_on_cancel_after = 0.1
        await _goal(nav, handle)
        started = time.monotonic()
        await nav.cancelGoToPose()
        assert handle.cancel_requests == 1
        assert handle.future.done()
        assert time.monotonic() - started >= 0.09
        # Its own CANCELED result belongs to it and is recorded.
        await asyncio.wait_for(nav.go_to_pose_event.wait(), 1.0)
        assert nav.go_to_pose_status == TaskResult.CANCELED
    asyncio.run(scenario())


def test_cancel_gives_up_on_a_goal_that_never_ends():
    async def scenario():
        nav = _navigator()
        await _goal(nav, _FakeHandle())
        started = time.monotonic()
        await nav.cancelGoToPose(timeout=0.1)
        assert time.monotonic() - started < 1.0
        assert nav.logger.warnings
    asyncio.run(scenario())


def test_cancelling_a_finished_goal_does_nothing():
    """The old callback replaced the result future with the response, so this
    raised AttributeError -- swallowed, leaving the next goal uncancelled."""
    async def scenario():
        nav = _navigator()
        handle = _FakeHandle()
        await _goal(nav, handle)
        handle.future.complete(_response(GoalStatus.STATUS_SUCCEEDED))
        await nav.cancelGoToPose()
        assert handle.cancel_requests == 0
        nav.go_to_pose_future = None
        await nav.cancelGoToPose()           # no goal at all
    asyncio.run(scenario())


def test_a_rejected_goal_leaves_nothing_to_cancel():
    async def scenario():
        nav = _navigator()
        nav.nav_to_pose_client.handles.append(_FakeHandle(accepted=False))
        assert not await nav.goToPose(PoseStamped())
        assert nav.go_to_pose_future is None
        await nav.cancelGoToPose()
    asyncio.run(scenario())


if __name__ == "__main__":
    for name, case in sorted(globals().items()):
        if name.startswith("test_"):
            case()
            print(f"ok  {name}")
