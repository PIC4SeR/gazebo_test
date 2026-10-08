"""Self-check for the perturbation task's formation monitor.

Drives :meth:`PerturbationTask._monitor_formation` against scripted Gazebo
states -- no simulator, no ROS graph -- to pin the two things that decide an
episode: it must not call steady state before the agents have moved, and it must
call divergence only when the fleet stays apart.

Run directly (``python3 test_perturbation_monitor.py``) or under pytest. Needs a
sourced ROS environment for ``hunav_msgs`` / ``gazebo_msgs``.
"""

import asyncio
import pathlib
import sys
import types

from geometry_msgs.msg import Pose
from hunav_msgs.msg import Agent, Agents
from people_msgs.msg import People, Person

# Allow running from the source tree before the package is installed.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from gazebo_test.tasks.perturbation import PerturbationTask  # noqa: E402
from gazebo_test.utils.evaluation_handler import ExperimentResult  # noqa: E402


def _state(x, y, speed=0.0):
    """Minimal stand-in for gazebo_msgs EntityState (pose.position + twist)."""
    return types.SimpleNamespace(
        pose=types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y)),
        twist=types.SimpleNamespace(linear=types.SimpleNamespace(x=speed, y=0.0)),
    )


class _FakeManager:
    """Serves one scripted poll per call, then repeats the last one forever."""

    def __init__(self, script):
        self._script = list(script)
        self.polls = 0
        self.gazebo_env_handler = types.SimpleNamespace(
            get_entity_state=self._get_entity_state
        )

    async def _get_entity_state(self, name):
        frame = self._script[min(self.polls, len(self._script) - 1)]
        if name == "r2":  # one poll == one state per robot
            self.polls += 1
        return frame[name]

    def get_logger(self):
        return types.SimpleNamespace(
            info=lambda *a: None, debug=lambda *a: None, warning=lambda *a: None
        )


def _task(script, humans_moved, humans_speed=0.0):
    task = PerturbationTask.__new__(PerturbationTask)  # skip the ROS __init__
    task.manager = _FakeManager(script)
    task._robot_names = ["r1", "r2"]
    task._spread_tolerance = 0.75
    task._spread_poll_period = 0.5
    task._spread_grace = 2.0
    task._settle_speed = 0.05
    task._settle_hold = 1.0
    task._max_spread = 0.0
    task._humans_moved = humans_moved
    task._humans_speed = humans_speed
    task._agents_pending = set()          # every finite-route agent arrived
    task._agents_unknown_warned = False
    return task


async def _run(task):
    return await asyncio.wait_for(task._monitor_formation(), timeout=5.0)


def _held(speed=0.0):
    return {"r1": _state(1.0, 0.0, speed), "r2": _state(-1.0, 0.0, speed)}


def _people(*speeds):
    msg = People()
    for speed in speeds:
        person = Person()
        person.velocity.x = speed
        person.velocity.z = 1.0  # angular rate: must not count as motion
        msg.people.append(person)
    return msg


def test_walking_agent_arms_the_settle_gate():
    # The fleet is already still at t=0, so steady state must not count until a
    # human has actually moved -- and must un-arm while one is still walking.
    task = _task([_held()], humans_moved=False)
    task._on_humans(_people(0.0, 0.0))
    assert not task._humans_moved
    task._on_humans(_people(0.0, 0.8))
    assert task._humans_moved and task._humans_speed == 0.8
    task._on_humans(_people(0.0, 0.0))
    assert task._humans_moved and task._humans_speed == 0.0


def test_settles_once_the_agents_have_parked():
    # Fleet still, humans done: steady state after settle_hold.
    result = asyncio.run(_run(_task([_held()], humans_moved=True)))
    assert result == ExperimentResult.SUCCESS, result


def test_does_not_settle_before_the_agents_move():
    # Same still fleet at t=0, but the perturbation has not happened yet: the
    # monitor must keep waiting rather than declare an instant success.
    try:
        asyncio.run(_run(_task([_held()], humans_moved=False)))
    except asyncio.TimeoutError:
        return
    raise AssertionError("settled before the agents ever moved")


def test_does_not_settle_while_a_human_is_still_walking():
    task = _task([_held()], humans_moved=True, humans_speed=0.8)
    try:
        asyncio.run(_run(task))
    except asyncio.TimeoutError:
        return
    raise AssertionError("settled while a human was still walking")


def test_does_not_settle_while_the_fleet_is_still_moving():
    try:
        asyncio.run(_run(_task([_held(speed=0.4)], humans_moved=True)))
    except asyncio.TimeoutError:
        return
    raise AssertionError("settled while the robots were still moving")


def test_sustained_divergence_fails():
    # Baseline spread 1.0; robots fly apart to 3.0 (+2.0 > tolerance) and stay.
    apart = {"r1": _state(3.0, 0.0), "r2": _state(-3.0, 0.0)}
    result = asyncio.run(_run(_task([_held(speed=0.4), apart], humans_moved=True)))
    assert result == ExperimentResult.FAILURE_DRIFT, result


def test_transient_stretch_is_not_divergence():
    # Out past the tolerance for one poll, then back: that is the perturbation,
    # and the fleet settles instead of failing.
    apart = {"r1": _state(3.0, 0.0), "r2": _state(-3.0, 0.0)}
    script = [_held(speed=0.4), apart, _held()]
    result = asyncio.run(_run(_task(script, humans_moved=True)))
    assert result == ExperimentResult.SUCCESS, result


def _agents(*specs):
    """hunav_msgs/Agents from (name, cyclic, goals left) tuples."""
    msg = Agents()
    for name, cyclic, left in specs:
        agent = Agent()
        agent.name, agent.cyclic_goals = name, cyclic
        agent.goals = [Pose() for _ in range(left)]
        msg.agents.append(agent)
    return msg


def test_a_stalled_pedestrian_is_not_steady_state():
    # Still fleet, still humans -- but agent1 still has its last goal queued
    # (blocked in front of the fleet): the monitor must keep waiting.
    task = _task([_held()], humans_moved=True)
    task._on_agent_states(_agents(("agent1", False, 1), ("agent2", False, 0)))
    try:
        asyncio.run(_run(task))
    except asyncio.TimeoutError:
        return
    raise AssertionError("called steady state with a pedestrian short of its goal")


def test_arrival_ignores_cyclic_agents():
    # A cyclic agent always has goals queued and never arrives: it must not
    # hold the episode open. Finite-route agents done -> steady state.
    task = _task([_held()], humans_moved=True)
    task._on_agent_states(_agents(("pacer", True, 2), ("agent1", False, 0)))
    assert task._humans_arrived()
    assert asyncio.run(_run(task)) == ExperimentResult.SUCCESS


def test_without_human_states_arrival_does_not_block():
    task = _task([_held()], humans_moved=True)
    task._agents_pending = None
    assert task._humans_arrived()
    assert asyncio.run(_run(task)) == ExperimentResult.SUCCESS


if __name__ == "__main__":
    for name, case in sorted(globals().items()):
        if name.startswith("test_"):
            case()
            print(f"ok  {name}")
    print("all perturbation monitor checks passed")
