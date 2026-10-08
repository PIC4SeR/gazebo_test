"""An episode deadline on the node's clock -- simulated time under use_sim_time.

asyncio's own timeouts run on the wall clock, so under a slow simulator the
fleet tasks ended their episodes early: 60 s at a real-time factor of 0.8 is
48 s of simulated time (2026-09-10 sweep, two Gazebo instances in parallel).
The single-robot ExperimentEvaluator already counts sim time with a ROS timer;
this is the same pattern, packaged so a task can race it like any other event.
"""

import asyncio
import time


class SimTimeout:
    """``event`` is set once ``seconds`` of node-clock time have passed.

    Use it as a context manager around one episode's wait, and create it after
    the episode's reset: every episode resets the simulation, and with it the
    clock, so a timer must not outlive the episode that armed it.

    The timer callback runs in the ROS executor thread and crosses into the
    loop with ``call_soon_threadsafe``, as ExperimentEvaluator._on_timeout does.
    """

    def __init__(self, node, seconds: float, loop=None) -> None:
        self.node = node
        self.seconds = float(seconds)
        self.loop = loop or asyncio.get_running_loop()
        self.event = asyncio.Event()
        self._timer = None
        self._sim_start = None
        self._wall_start = None

    def __enter__(self) -> "SimTimeout":
        self._sim_start = self.node.get_clock().now()
        self._wall_start = time.monotonic()
        self._timer = self.node.create_timer(self.seconds, self._fire)
        return self

    def __exit__(self, *exc) -> bool:
        if self._timer is not None:
            self._timer.cancel()
            self.node.destroy_timer(self._timer)
            self._timer = None
        return False

    def _fire(self) -> None:
        if self._timer is not None:
            self._timer.cancel()          # one shot
        self.loop.call_soon_threadsafe(self.event.set)

    def elapsed(self) -> tuple:
        """(sim seconds, wall seconds) since the deadline was armed."""
        sim = (self.node.get_clock().now() - self._sim_start).nanoseconds / 1e9
        return sim, time.monotonic() - self._wall_start
