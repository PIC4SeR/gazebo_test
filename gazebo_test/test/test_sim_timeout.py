"""The fleet episode deadline runs on sim time, with a wall-clock stall guard.

Drives FormationTask._drive_swarm with a fake navigator and a fake node whose
ROS timer the test fires by hand -- so "the sim-time deadline passed" is an
explicit event, independent of how fast the wall clock runs. Needs a sourced
ROS environment (gazebo_collision_msgs, nav2_msgs).
"""

import asyncio
import pathlib
import sys
import time
import types

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from gazebo_test.tasks.formation import FormationTask  # noqa: E402
from gazebo_test.utils.basic_navigator import TaskResult  # noqa: E402
from gazebo_test.utils.evaluation_handler import ExperimentResult  # noqa: E402
from gazebo_test.utils.sim_timeout import SimTimeout  # noqa: E402


class _Timer:
    def __init__(self, seconds, callback):
        self.seconds, self.callback, self.cancelled = seconds, callback, False

    def cancel(self):
        self.cancelled = True


class _Clock:
    def now(self):
        return _Stamp(time.monotonic())


class _Stamp:
    def __init__(self, t):
        self.t = t

    def __sub__(self, other):
        return types.SimpleNamespace(nanoseconds=int((self.t - other.t) * 1e9))


class _FakeManager:
    def __init__(self, timeout, stall_factor):
        self.evaluation_handler = types.SimpleNamespace(timeout_duration=timeout)
        self.stall_factor = stall_factor
        self.timers, self.destroyed = [], []

    def create_timer(self, seconds, callback):
        self.timers.append(_Timer(seconds, callback))
        return self.timers[-1]

    def destroy_timer(self, timer):
        self.destroyed.append(timer)

    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *a: None, warning=lambda *a: None,
                                     error=lambda *a: None, debug=lambda *a: None)


class _FakeNavigator:
    def __init__(self):
        self.go_to_pose_event = asyncio.Event()
        self.go_to_pose_status = TaskResult.UNKNOWN

    def clearEvents(self):
        self.go_to_pose_event.clear()

    async def goToPose(self, pose):
        return True


def _task(timeout=60.0, stall_factor=5.0, station_keeping=False):
    task = FormationTask.__new__(FormationTask)          # skip the ROS __init__
    task.manager = _FakeManager(timeout, stall_factor)
    task._swarm_nav = types.SimpleNamespace(navigator=_FakeNavigator())
    task._robot_names = {"r1", "r2"}
    task._collision_event = {ns: asyncio.Event() for ns in task._robot_names}
    task._collision_result = {ns: None for ns in task._robot_names}
    task._loop = asyncio.get_running_loop()
    task.station_keeping = station_keeping
    return task


async def _episode(task, after, action):
    """Run one episode, doing ``action(task)`` ``after`` wall seconds in."""
    asyncio.get_running_loop().call_later(after, action, task)
    return await asyncio.wait_for(task._drive_swarm(None, "episode_1"), 5.0)


def _deadline(task):
    task.manager.timers[-1].callback()           # the ROS timer fires


def _converge(status):
    def action(task):
        nav = task._swarm_nav.navigator
        nav.go_to_pose_status = status
        nav.go_to_pose_event.set()
    return action


def _collide(ns, result):
    def action(task):
        task._collision_result[ns] = result
        task._collision_event[ns].set()
    return action


def test_the_deadline_is_a_sim_time_timer_and_ends_the_episode():
    async def scenario():
        task = _task(timeout=60.0)
        result = await _episode(task, 0.01, _deadline)
        timer = task.manager.timers[0]
        assert timer.seconds == 60.0                     # on the node's clock
        assert timer in task.manager.destroyed           # never outlives it
        return result
    assert asyncio.run(scenario()) == ExperimentResult.FAILURE_TIMEOUT


def test_a_slow_simulator_no_longer_shortens_the_episode():
    """60 s of wall time used to end the episode whatever the sim had done.
    Now only the timer (sim time) or the stall guard (5x) does: 0.2 s of wall
    time at timeout 0.1 x factor 5 is still inside the guard."""
    async def scenario():
        task = _task(timeout=0.1, stall_factor=5.0)
        return await _episode(task, 0.2, _converge(TaskResult.SUCCEEDED))
    assert asyncio.run(scenario()) == ExperimentResult.SUCCESS


def test_a_stalled_simulator_is_its_own_result():
    async def scenario():
        task = _task(timeout=0.05, stall_factor=2.0)     # guard: 0.1 s wall
        return await _episode(task, 10.0, _deadline)     # the timer never comes
    assert asyncio.run(scenario()) == ExperimentResult.FAILURE_SIM_STALLED


def test_convergence_and_navigation_failure():
    assert asyncio.run(_episode_with(_converge(TaskResult.SUCCEEDED))) \
        == ExperimentResult.SUCCESS
    assert asyncio.run(_episode_with(_converge(TaskResult.CANCELED))) \
        == ExperimentResult.FAILURE_NAVIGATION


def test_a_collision_of_any_robot_ends_the_episode():
    result = asyncio.run(_episode_with(
        _collide("r2", ExperimentResult.FAILURE_COLLISION_ROBOT)))
    assert result == ExperimentResult.FAILURE_COLLISION_ROBOT


def test_station_keeping_succeeds_at_the_deadline():
    async def scenario():
        return await _episode(_task(station_keeping=True), 0.01, _deadline)
    assert asyncio.run(scenario()) == ExperimentResult.SUCCESS


def test_convergence_beats_a_deadline_that_lands_together():
    def both(task):
        _converge(TaskResult.SUCCEEDED)(task)
        _deadline(task)
    assert asyncio.run(_episode_with(both)) == ExperimentResult.SUCCESS


async def _episode_with(action):
    return await _episode(_task(), 0.01, action)


def test_sim_timeout_reports_elapsed_time_and_its_guard():
    async def scenario():
        manager = _FakeManager(2.0, 3.0)
        with SimTimeout(manager, 2.0) as deadline:
            await asyncio.sleep(0.05)
            sim, wall = deadline.elapsed()
            manager.timers[0].callback()
            await asyncio.wait_for(deadline.event.wait(), 1.0)
        assert manager.timers[0].cancelled
        assert sim >= 0.04 and wall >= 0.04
    asyncio.run(scenario())


if __name__ == "__main__":
    for name, case in sorted(globals().items()):
        if name.startswith("test_"):
            case()
            print(f"ok  {name}")
