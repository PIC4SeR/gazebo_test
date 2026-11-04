"""The tmux lane a ``ros2 gazeboexp run`` builds, compared with a golden record.

The golden file was recorded from ``full_experiment.run`` before the LaneSpec
split, so the refactored CLI must send byte-identical commands to every tmux
window. Regenerate it only for an intended change:

    GAZEBO_TEST_UPDATE_GOLDEN=1 python3 -m pytest gazebo_test_cli/test/test_lane_spec.py

The cases use real registered experiments and navigators (they need the
workspace sourced); only the rendered Nav2 params path, the checkpoint store
and the summary printer are faked.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from fake_tmux import FakeServer  # noqa: E402

from gazebo_test_cli import full_experiment as fe  # noqa: E402

GOLDEN = Path(__file__).parent / "golden_tmux_lanes.json"

CASES = {
    "single_robot_rviz_eval_bag": ["social_env_test", "--hunav-eval", "--bag-record",
                                   "--ros-domain-id", "11"],
    "fleet_navigation_launch_headless": ["perturbation_tb2_staggered_far_adaptive", "--headless",
                                         "--no-rviz", "--episodes", "3", "--ros-domain-id", "12"],
    "fleet_rviz": ["formation_crossing_tb2_adaptive", "--ros-domain-id", "13"],
    "navigator_extras_no_gpu": ["social_nav", "--navigator", "SICNAV", "--no-gpu", "--headless",
                                "--repetitions", "2", "--wait-before-start", "3",
                                "--ros-domain-id", "14"],
    "checkpoint_external_nav": ["social_nav", "--checkpoint-dsn", "postgresql://u:p@db:5432/c",
                                "--resume-checkpoint", "--no-navigation-launch", "--no-rviz",
                                "--ros-domain-id", "15"],
}


class _FakeStore:
    def __init__(self, dsn: str) -> None:
        self.dsn = dsn

    def list_jobs(self, **kwargs):
        return []

    def has_incomplete_jobs(self, **kwargs):
        return True


def _patch(monkeypatch, name, value):
    """Patch ``name`` wherever the CLI looks it up (before and after the split)."""
    import importlib
    for module_name in ("gazebo_test_cli.full_experiment", "gazebo_test_cli.lane_spec"):
        try:
            module = importlib.import_module(module_name)
        except ImportError:
            continue
        if hasattr(module, name):
            monkeypatch.setattr(module, name, value)


def run_case(monkeypatch, argv):
    FakeServer.instances.clear()
    FakeServer.pane_output = ["Experiment outcomes saved to /results/x/outcomes.csv [12.3]"]
    monkeypatch.setattr(fe.libtmux, "Server", FakeServer)
    _patch(monkeypatch, "get_navigator_yaml",
           lambda navigator, render_vars=None: f"/rendered/{navigator}_config.yaml")
    _patch(monkeypatch, "ExperimentCheckpointStore", _FakeStore)
    summaries = []
    _patch(monkeypatch, "print_summary", lambda path: summaries.append(str(path)))
    monkeypatch.setenv("ROS_DOMAIN_ID", "0")
    monkeypatch.delenv("GAZEBO_TEST_CHECKPOINT_DSN", raising=False)

    parser = fe.build_parser(argparse.ArgumentParser())
    args = parser.parse_args(["run", *argv])
    fe.run(args)

    (server,) = FakeServer.instances
    (session,) = server.created
    record = session.record()
    record["server_commands"] = [list(c) for c in server.commands]
    record["closed"] = server.closed
    record["summaries"] = summaries
    return record


def test_tmux_lanes_match_golden(monkeypatch):
    records = {name: run_case(monkeypatch, argv) for name, argv in CASES.items()}
    if os.environ.get("GAZEBO_TEST_UPDATE_GOLDEN"):
        GOLDEN.write_text(json.dumps(records, indent=1, sort_keys=True) + "\n")
        pytest.skip("golden file regenerated")
    golden = json.loads(GOLDEN.read_text())
    assert sorted(records) == sorted(golden)
    for name in CASES:
        assert records[name] == golden[name], name
