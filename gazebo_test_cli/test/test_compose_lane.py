"""``ros2 gazeboexp run --backend compose``: the compose project a lane becomes.

Uses real registered experiments and navigators (workspace sourced). Only the
checkpoint store is faked; the navigator's Nav2 params are really rendered, to
check they land in the lane dir on the shared data root.
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from test_lane_spec import CASES, GOLDEN, _FakeStore, _patch  # noqa: E402

from gazebo_test_cli import full_experiment as fe  # noqa: E402
from gazebo_test_cli.compose_lane import ENTRYPOINT  # noqa: E402


def write(monkeypatch, tmp_path, argv, results=None, store=_FakeStore):
    _patch(monkeypatch, "ExperimentCheckpointStore", store)
    monkeypatch.setenv("ROS_DOMAIN_ID", "0")
    monkeypatch.setattr(tempfile, "tempdir", None)  # write_compose sets it
    data = tmp_path / "data"
    lane = data / "lanes" / "lane"
    results = data / "results" if results is None else results
    parser = fe.build_parser(argparse.ArgumentParser())
    args = parser.parse_args(["run", *argv, "--backend", "compose", "--lane-dir", str(lane),
                              "--data-root", str(data), "--base-path-for-results", str(results),
                              "--image", "gt-lane:test"])
    path = fe.write_compose(args)
    return data, lane, path


def golden_windows(case):
    return dict(json.loads(GOLDEN.read_text())[case]["windows"])


def test_single_robot_lane(monkeypatch, tmp_path):
    data, lane, path = write(monkeypatch, tmp_path, CASES["single_robot_rviz_eval_bag"])
    doc = yaml.safe_load(path.read_text())
    assert doc["name"] == "lane-11"
    services = doc["services"]
    # RViz has no display in a container; the rest keeps the tmux order.
    assert list(services) == ["sim", "nav", "evaluator", "manager"]
    tmux = golden_windows("single_robot_rviz_eval_bag")
    # Always headless in a container (the tmux case ran with a GUI).
    assert services["sim"]["command"] == [
        "bash", "-c", tmux["Gazebo"][0].strip().replace("headless:=False", "headless:=True")]
    assert services["nav"]["command"] == ["bash", "-c", tmux["Navigation"][0].strip()]
    # The manager runs the tmux window's launch (plus the results path on the
    # data root), without the tmux wait-for.
    manager = services["manager"]["command"][2]
    assert manager.startswith("ros2 launch gazebo_test experiment_manager.launch.py ")
    assert f"base_path:={data / 'results'} " in manager
    assert "tmux" not in manager and manager.endswith("task:=go_to_pose")
    for name, body in services.items():
        assert body["image"] == "gt-lane:test"
        assert body["volumes"] == [f"{data.resolve()}:{data.resolve()}"]
        env = body["environment"]
        assert env["ROS_DOMAIN_ID"] == "11"
        assert env["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp"  # compose default
        assert env["GT_DATA"] == str(data.resolve())
        if name == "sim":
            assert "depends_on" not in body
            assert body["healthcheck"]["test"][:2] == ["CMD", ENTRYPOINT]
            assert "clock_ready" in body["healthcheck"]["test"]
            assert env["GT_XVFB"] == "1"
        else:
            assert body["depends_on"] == {"sim": {"condition": "service_healthy"}}
    spec = json.loads((lane / "lane.json").read_text())
    assert spec["domain"] == 11 and spec["manager"]["role"] == "manager"


def test_navigator_params_rendered_into_the_lane(monkeypatch, tmp_path):
    data, lane, path = write(monkeypatch, tmp_path, CASES["navigator_extras_no_gpu"])
    services = yaml.safe_load(path.read_text())["services"]
    assert list(services) == ["sim", "nav", "extra-1", "manager"]
    rendered = list((lane / "config").rglob("*SICNAV*config.yaml"))
    assert len(rendered) == 1
    assert f"params_file:={rendered[0]}" in services["nav"]["command"][2]
    assert f"params_file:={rendered[0]}" in services["manager"]["command"][2]
    assert "costmap_converter" in services["extra-1"]["command"][2]


def test_explicit_rmw_wins(monkeypatch, tmp_path):
    _, _, path = write(monkeypatch, tmp_path, [*CASES["checkpoint_external_nav"], "--rmw", "fastdds"])
    services = yaml.safe_load(path.read_text())["services"]
    assert list(services) == ["sim", "manager"]  # external navigation stack, no RViz
    assert "RMW_IMPLEMENTATION" not in services["sim"]["environment"]


def test_results_must_be_on_the_data_root(monkeypatch, tmp_path):
    with pytest.raises(SystemExit, match="base-path-for-results"):
        write(monkeypatch, tmp_path, CASES["single_robot_rviz_eval_bag"],
              results=tmp_path / "elsewhere")


def test_complete_experiment_leaves_no_compose_file(monkeypatch, tmp_path):
    class Done(_FakeStore):
        def list_jobs(self, **kwargs):
            return [{"status": "completed"}]

        def has_incomplete_jobs(self, **kwargs):
            return False

    stale = tmp_path / "data" / "lanes" / "lane" / "compose.yaml"
    stale.parent.mkdir(parents=True)
    stale.write_text("stale")
    _, lane, path = write(monkeypatch, tmp_path, CASES["checkpoint_external_nav"], store=Done)
    assert path is None and not (lane / "compose.yaml").exists()
