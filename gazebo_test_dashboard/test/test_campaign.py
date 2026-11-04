"""The campaign end to end with a fake `ros2 gazeboexp`: plan, batches, lanes,
requeue, stall teardown scoped to one lane, stagger, occupied lanes, restart."""

import os
import pathlib
import subprocess
import sys
import time

import pytest
import yaml

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from gazebo_test_dashboard import status as S  # noqa: E402
from gazebo_test_dashboard.campaign import Campaign, read_state  # noqa: E402
from gazebo_test_dashboard.lanes import (TmuxLaneBackend, gazebo_uri, lane_pids,  # noqa: E402
                                         parse_gz_stats, session_name)
from gazebo_test_dashboard.plan import expand, load_plan, plan_warnings  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
EXAMPLE = HERE.parent / "gazebo_test_dashboard" / "example_campaign.yaml"
EPISODES = ["episode_1", "episode_2"]
D1, D2 = 93, 94  # test lanes, away from real ones


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def make_plan(tmp_path, **overrides):
    raw = yaml.safe_load(EXAMPLE.read_text())
    raw.update(results_base_path="results", cli=[sys.executable, str(HERE / "fake_gazeboexp.py")],
               common_args=["--headless"])
    raw["lanes"].update(domains=[D1], stagger_s=0, max_attempts=2)
    raw["experiments"] = [{"name": "social_env_test", "repetitions": 2}]
    for key, value in overrides.items():
        if key == "lanes":
            raw["lanes"].update(value)
        else:
            raw[key] = value
    path = tmp_path / "plan.yaml"
    path.write_text(yaml.safe_dump(raw))
    return load_plan(path)


def make_campaign(plan, clock=None):
    return Campaign(plan, TmuxLaneBackend(gz_stats=False, kill_grace_s=2),
                    episode_resolver=lambda name: EPISODES, clock=clock or time.time)


def run_until(campaign, predicate, timeout=20.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        campaign.tick()
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError(f"timeout; batches: {[(r['id'], r['status'], r['reason']) for r in campaign.rows]}")


def statuses(campaign):
    return [r["status"] for r in sorted(campaign.rows, key=lambda r: r["order"])]


@pytest.fixture(autouse=True)
def _env(monkeypatch):
    monkeypatch.delenv("FAKE_MODE", raising=False)
    yield
    for d in (D1, D2):
        subprocess.run(["tmux", "kill-session", "-t", f"={session_name(d)}"], capture_output=True)


def test_plan_expands_and_validates(tmp_path):
    plan = make_plan(tmp_path)
    batches = expand(plan)
    assert [(b["experiment"], b["algorithm"], b["episodes"]) for b in batches] == [
        ("social_env_test", "dwb", ""), ("social_env_test", "mppi", "")]
    split = make_plan(tmp_path, experiments=[{"name": "x", "repetitions": 1, "split_episodes": True}])
    assert [b["episodes"] for b in expand(split, lambda n: EPISODES)] == EPISODES * 2
    with pytest.raises(ValueError, match="--repetitions"):
        make_plan(tmp_path, common_args=["--repetitions", "3"])
    with pytest.raises(ValueError, match="wait-after-finish"):
        make_plan(tmp_path, common_args=["--wait-after-finish"])
    warned = plan_warnings(make_plan(tmp_path, lanes={"domains": [0, 5], "stagger_s": 5}),
                           known_experiments=["other"], known_navigators=["DWB"])
    assert len(warned) == 4  # domain 0, short stagger, unknown experiment, MPPI unknown


def test_checkpoint_namespace_matches_the_manager():
    assert S.checkpoint_namespace("") == "no_checkpoint"
    dsn = "postgresql://gazebo:gazebo@localhost:5432/checkpoints2"
    try:
        from gazebo_test.experiment_manager import ExperimentManager
    except Exception:  # noqa: BLE001 - gazebo_test / rclpy not available
        pytest.skip("gazebo_test not importable")
    assert S.checkpoint_namespace(dsn) == ExperimentManager._derive_checkpoint_namespace(dsn)


def test_a_campaign_runs_to_the_end(tmp_path):
    plan = make_plan(tmp_path)
    campaign = make_campaign(plan)
    command = campaign.command({**campaign.rows[0], "lane": D1})
    assert command[2:4] == ["run", "social_env_test"]
    for flag in ("--algorithm-name", "--repetitions", "--ros-domain-id", "--disable-checkpoint",
                 "--resume-checkpoint", "--headless", "--navigator"):
        assert flag in command
    campaign.set_running(True)
    run_until(campaign, lambda: statuses(campaign) == ["done", "done"])
    snap = campaign.snapshot()
    cell = snap["matrix"]["cells"]["social_env_test|dwb"]
    assert (cell["done"], cell["expected"], cell["success_rate"], cell["status"]) == (4, 4, 0.5, "done")
    assert snap["pairing"]["social_env_test"]["ok"] is True
    assert [r["status"] for r in read_state(pathlib.Path(plan.state_file))] == ["done", "done"]


def test_an_incomplete_batch_is_resumed_then_fails_after_max_attempts(tmp_path, monkeypatch):
    plan = make_plan(tmp_path)
    campaign = make_campaign(plan)
    monkeypatch.setenv("FAKE_MODE_dwb", "partial")
    campaign.set_running(True)
    run_until(campaign, lambda: campaign.rows[0]["status"] == "failed")
    dwb = campaign.rows[0]
    assert dwb["attempts"] == 2 and "runs after 2 attempt(s)" in dwb["reason"]
    assert campaign.progress(dwb)["done"] == 3  # 2 of 4, then 1 of the 2 missing
    monkeypatch.setenv("FAKE_MODE_dwb", "ok")
    campaign.requeue_failed()
    run_until(campaign, lambda: statuses(campaign) == ["done", "done"])


def test_a_stalled_lane_is_torn_down_without_touching_another_lane(tmp_path, monkeypatch):
    clock = Clock()
    plan = make_plan(tmp_path, lanes={"stall_timeout_s": 60, "max_attempts": 1})
    campaign = make_campaign(plan, clock)
    other = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"],
                             env={**os.environ, "GAZEBO_MASTER_URI": gazebo_uri(D2)})
    monkeypatch.setenv("FAKE_MODE", "hang")
    campaign.set_running(True)
    campaign.tick()
    assert campaign.rows[0]["status"] == "running"
    for _ in range(50):  # the fake "gzserver" of the lane is up
        if lane_pids(gazebo_uri(D1)):
            break
        time.sleep(0.05)
    assert lane_pids(gazebo_uri(D1))
    clock.t += 61
    campaign.set_running(False)
    campaign.tick()
    row = campaign.rows[0]
    assert row["status"] == "failed" and row["reason"].startswith("stalled: no new outcome row")
    assert lane_pids(gazebo_uri(D1)) == []
    assert other.poll() is None  # the other lane's process survived
    other.kill()


def test_leftover_processes_are_cleaned_after_a_batch(tmp_path, monkeypatch):
    plan = make_plan(tmp_path, algorithms=[{"name": "dwb", "args": []}])
    campaign = make_campaign(plan)
    monkeypatch.setenv("FAKE_MODE", "leftover")
    campaign.set_running(True)
    run_until(campaign, lambda: statuses(campaign) == ["done"])
    assert lane_pids(gazebo_uri(D1)) == []


def test_stagger_and_parallel_lanes(tmp_path, monkeypatch):
    clock = Clock()
    plan = make_plan(tmp_path, lanes={"domains": [D1, D2], "stagger_s": 30})
    campaign = make_campaign(plan, clock)
    monkeypatch.setenv("FAKE_MODE", "hang")
    campaign.set_running(True)
    campaign.tick()
    assert statuses(campaign) == ["running", "pending"]
    clock.t += 10
    campaign.tick()
    assert statuses(campaign) == ["running", "pending"]
    clock.t += 25
    campaign.tick()
    assert statuses(campaign) == ["running", "running"]
    assert {r["lane"] for r in campaign.rows} == {D1, D2}
    campaign.stop_lane(D1)
    assert campaign.rows[0]["status"] == "pending" and campaign.rows[0]["attempts"] == 0
    assert campaign.lanes[D1].enabled is False
    campaign.stop_lane(D2)


def test_an_occupied_lane_is_left_alone_until_cleaned(tmp_path):
    plan = make_plan(tmp_path)
    campaign = make_campaign(plan)
    subprocess.run(["tmux", "new-session", "-d", "-s", session_name(D1), "sleep 60"], check=True)
    campaign.set_running(True)
    campaign.tick()
    assert statuses(campaign) == ["pending", "pending"]
    assert campaign.snapshot()["lanes"][0]["state"] == "occupied"
    assert campaign.clean_lane(D1)["stopped"] == [f"tmux session {session_name(D1)}"]
    run_until(campaign, lambda: statuses(campaign) == ["done", "done"])


def test_a_restarted_dashboard_requeues_what_was_running(tmp_path):
    plan = make_plan(tmp_path)
    campaign = make_campaign(plan)
    campaign.rows[0]["status"] = "running"
    campaign.rows[1]["notes"] = "check RTF"
    campaign._save()
    again = make_campaign(make_plan(tmp_path))
    assert again.rows[0]["status"] == "pending" and "interrupted" in again.rows[0]["reason"]
    assert again.rows[1]["notes"] == "check RTF"


def test_queue_edits(tmp_path):
    campaign = make_campaign(make_plan(tmp_path))
    campaign.edit(2, "up")
    assert [r["id"] for r in sorted(campaign.rows, key=lambda r: r["order"])] == [2, 1]
    campaign.edit(1, "skip")
    assert campaign.next_pending()["id"] == 2
    campaign.edit(1, "unskip")
    campaign.edit(1, "notes", "rerun with seed 8")
    assert campaign.batch(1)["notes"] == "rerun with seed 8"
    with pytest.raises(ValueError):
        campaign.edit(1, "explode")


def test_pairing_check_and_gz_stats_parsing():
    import pandas as pd
    a = pd.DataFrame({"experiment_tag": ["e"] * 3, "run_id": [1, 2, 3], "ped_draw": ["s1_e1", "s1_e2", "s1_e3"]})
    b = pd.DataFrame({"experiment_tag": ["e"] * 2, "run_id": [1, 2], "ped_draw": ["s1_e1", "s1_e2"]})
    check = S.pairing_check({"a": a, "b": b, "c": None})
    assert check["common_draws"] == 2 and check["algorithms"]["a"]["unpaired"] == 1
    assert check["without_draws"] == ["c"] and check["ok"] is False
    assert S.pairing_check({"a": a.drop(columns="ped_draw")}) is None
    h = parse_gz_stats("# real-time factor (percent), simtime (sec), realtime (sec), paused (T or F)\n"
                       "0.92,         0.184000,         0.477162, F\n0.31,        10.580000,        38.471730, F\n")
    assert (h.rtf, h.sim_time, h.paused) == (0.31, 10.58, False)
    assert parse_gz_stats("An instance of Gazebo is not running.\n").error


def test_more_repetitions_in_the_plan_reopen_a_done_batch(tmp_path):
    campaign = make_campaign(make_plan(tmp_path, algorithms=[{"name": "dwb", "args": []}]))
    campaign.set_running(True)
    run_until(campaign, lambda: statuses(campaign) == ["done"])
    again = make_campaign(make_plan(tmp_path, algorithms=[{"name": "dwb", "args": []}],
                                    experiments=[{"name": "social_env_test", "repetitions": 3}]))
    assert again.rows[0]["status"] == "pending" and again.rows[0]["reason"] == "plan changed: 4/6 runs"


def test_the_running_episode_and_run_are_shown(tmp_path, monkeypatch):
    log = ("[experiment_manager]: Starting hunav evaluator recording ...\n"
           "[experiment_manager-1] [INFO] [1.0] [experiment_manager]: Starting episode_1 run 2\n"
           "[experiment_manager-1] [INFO] [2.0] [experiment_manager]: Starting episode_2 run 1\n"
           "[basic_navigator]: Cleared all costmaps\n")
    assert S.current_run(log) == {"episode": "episode_2", "run_id": 1}
    assert S.current_run("Starting up\n") is None

    class Backend(TmuxLaneBackend):
        def pane_tail(self, domain, lines=40):
            return log

    plan = make_plan(tmp_path, algorithms=[{"name": "dwb", "args": []}])
    campaign = Campaign(plan, Backend(gz_stats=False, kill_grace_s=2), lambda name: EPISODES)
    monkeypatch.setenv("FAKE_MODE", "hang")
    campaign.set_running(True)
    campaign.tick()
    snap = campaign.snapshot()
    current = snap["lanes"][0]["batch"]["current"]
    assert current == {"episode": "episode_2", "run_id": 1, "repetitions": 2, "episode_index": 2,
                       "episodes": 2, "recorded": False}
    assert snap["batches"][0]["current"] == current
    assert snap["lanes"][0]["pane_tail"].endswith("Cleared all costmaps")
    campaign.stop_lane(D1)


def test_a_batch_can_be_run_on_a_chosen_lane(tmp_path, monkeypatch):
    clock = Clock()
    campaign = make_campaign(make_plan(tmp_path, lanes={"domains": [D1, D2], "stagger_s": 30}), clock)
    monkeypatch.setenv("FAKE_MODE", "hang")
    campaign.edit(2, "run", str(D2))  # campaign paused: still starts
    assert campaign.batch(2)["status"] == "running" and campaign.batch(2)["lane"] == D2
    with pytest.raises(PermissionError, match="not free"):
        campaign.edit(1, "run", str(D2))
    with pytest.raises(PermissionError, match="stagger"):
        campaign.edit(1, "run", str(D1))
    clock.t += 31
    campaign.edit(1, "run", str(D1))
    assert campaign.batch(1)["lane"] == D1
    campaign.stop_lane(D1)
    campaign.stop_lane(D2)


def test_window_toggles_set_the_launch_flags(tmp_path):
    campaign = make_campaign(make_plan(tmp_path))  # plan common_args: [--headless]
    assert campaign.windows == {"gazebo_gui": False, "rviz": True, "hunav_eval": False}
    row = {**campaign.rows[0], "lane": D1}
    assert "--headless" in campaign.command(row) and "--no-rviz" not in campaign.command(row)
    campaign.set_windows({"gazebo_gui": True, "rviz": False, "hunav_eval": True})
    command = campaign.command(row)
    assert "--headless" not in command and "--no-rviz" in command and "--hunav-eval" in command
    with pytest.raises(ValueError):
        campaign.set_windows({"gzclient": True})
