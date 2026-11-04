"""Which pedestrian motion models an episode ran with, for its outcomes row.

hunav_agent_manager publishes every episode's draw on ``~/motion_models`` (latched,
JSON ``{"episode", "strategy", "seed", "models": {agent: model}}``): the episode
index advances once per ``reset_agents`` call, and with the ``random`` strategy the
models are drawn from the seeded generator at that reset. Recording the draw in the
outcomes CSV makes it the pairing key of repeated-measures tests (benchmark_report
``statistics.block``) instead of assuming that run k of every navigator replayed the
same draw -- which a missed or extra reset within a launch would silently break.
"""

import json
import threading
from typing import Any, Dict, Optional, Tuple


class PedestrianDrawTracker:
    """Keeps the latest draw (fed from the ROS callback thread) and turns it into
    outcome columns at the end of each run (read from the asyncio thread)."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: Optional[Dict[str, Any]] = None
        self._last_recorded: Optional[int] = None

    def on_message(self, data: str) -> None:
        """``data`` of the std_msgs/String message; malformed messages are ignored."""
        try:
            draw = json.loads(data)
            int(draw["episode"])
        except (TypeError, ValueError, KeyError):
            return
        with self._lock:
            self._latest = draw

    def columns_for_run(self) -> Tuple[Dict[str, Any], Optional[str]]:
        """The outcome columns of the run that just ended and a warning, if any.

        Columns (empty when no draw has been received, e.g. no HuNav or an older
        hunav_agent_manager):
          ped_seed        the motion_model_seed (-1: not seeded)
          ped_draw_index  the manager's episode index (resets in this launch)
          ped_draw        "s<seed>_e<index>", the same draw in every launch with that seed
          ped_models      "agent1:sfm;agent2:orca", the pedestrian condition itself
        The warning flags a draw already recorded for the previous run: the reset
        that should have started this run never reached the manager.
        """
        with self._lock:
            draw = self._latest
            if draw is None:
                return {}, None
            index, seed = int(draw["episode"]), int(draw.get("seed", -1))
            warning = None
            if self._last_recorded is not None and index == self._last_recorded:
                warning = (f"pedestrian draw {index} was already recorded for the previous run; "
                           "this run's reset did not reach hunav_agent_manager")
            self._last_recorded = index
        models = draw.get("models") or {}
        return {
            "ped_seed": seed,
            "ped_draw_index": index,
            "ped_draw": f"s{seed}_e{index}",
            "ped_models": ";".join(f"{agent}:{model}" for agent, model in models.items()),
        }, warning
