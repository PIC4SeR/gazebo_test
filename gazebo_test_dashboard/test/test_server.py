"""The page and its API with a fake `ros2 gazeboexp`."""

import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from gazebo_test_dashboard.lanes import TmuxLaneBackend  # noqa: E402
from gazebo_test_dashboard.server import create_app  # noqa: E402
from test_campaign import EPISODES, make_plan  # noqa: E402


def test_page_api_and_exports(tmp_path):
    plan = make_plan(tmp_path)
    app = create_app(plan, TmuxLaneBackend(gz_stats=False), lambda name: EPISODES,
                     warnings=["a warning"], threads=False)
    campaign = app.config["campaign"]
    client = app.test_client()
    assert b"Gazebo campaign" in client.get("/").data
    state = client.get("/api/state").get_json()
    assert state["warnings"] == ["a warning"] and state["running"] is False
    assert [b["progress"]["expected"] for b in state["batches"]] == [4, 4]

    assert client.post("/api/batches/1/notes", json={"value": "first"}).get_json()["ok"]
    assert client.post("/api/batches/9/skip").status_code == 400
    assert client.post("/api/lanes/5/stop").status_code == 400
    assert client.post("/api/campaign/start").get_json()["running"] is True
    for _ in range(400):
        campaign.tick()
        if all(r["status"] == "done" for r in campaign.rows):
            break
        time.sleep(0.05)
    state = client.get("/api/state").get_json()
    assert state["counts"]["done"] == 2
    assert client.post("/api/batches/1/requeue").get_json()["ok"]
    assert client.post("/api/campaign/pause").get_json()["running"] is False

    exported = client.get("/export/campaign.csv").data.decode()
    assert exported.splitlines()[0].startswith("order,id,experiment,algorithm")
    assert "first" in exported
    matrix = client.get("/export/matrix.csv").data.decode().splitlines()
    assert matrix[1].startswith("social_env_test,dwb,")
