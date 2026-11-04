"""The pedestrian draw of each run becomes its outcome columns (no ROS needed)."""

import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from gazebo_test.utils.pedestrian_draws import PedestrianDrawTracker  # noqa: E402


def _draw(episode, seed=11, models=None):
    return json.dumps({"episode": episode, "strategy": "random", "seed": seed,
                       "models": models or {"agent1": "sfm", "agent2": "orca"}})


def test_no_draw_no_columns():
    assert PedestrianDrawTracker().columns_for_run() == ({}, None)


def test_columns_of_the_latest_draw():
    tracker = PedestrianDrawTracker()
    tracker.on_message(_draw(1))
    tracker.on_message(_draw(2, models={"agent1": "cv", "agent2": "sfm"}))
    columns, warning = tracker.columns_for_run()
    assert columns == {"ped_seed": 11, "ped_draw_index": 2, "ped_draw": "s11_e2",
                       "ped_models": "agent1:cv;agent2:sfm"}
    assert warning is None


def test_a_draw_recorded_twice_is_flagged():
    tracker = PedestrianDrawTracker()
    tracker.on_message(_draw(3))
    assert tracker.columns_for_run()[1] is None
    columns, warning = tracker.columns_for_run()   # no reset in between
    assert columns["ped_draw_index"] == 3 and "already recorded" in warning
    tracker.on_message(_draw(4))
    assert tracker.columns_for_run()[1] is None


def test_malformed_messages_are_ignored():
    tracker = PedestrianDrawTracker()
    for bad in ("not json", "{}", json.dumps({"episode": "x"})):
        tracker.on_message(bad)
    assert tracker.columns_for_run() == ({}, None)
