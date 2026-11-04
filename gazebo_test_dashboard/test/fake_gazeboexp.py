"""Stands in for `ros2 gazeboexp`: writes the outcome rows gazebo_test writes.

FAKE_MODE (or FAKE_MODE_<algorithm>): ok | partial (half the missing runs, exit 0)
| hang (never ends) | leftover (ok, but leaves a "gzserver" behind). Every mode
starts a child carrying the lane's GAZEBO_MASTER_URI, as tmux panes do.
"""

import argparse
import os
import pathlib
import subprocess
import sys
import time

import pandas as pd

parser = argparse.ArgumentParser()
parser.add_argument("command")
parser.add_argument("experiment")
parser.add_argument("--algorithm-name")
parser.add_argument("--repetitions", type=int)
parser.add_argument("--base-path-for-results")
parser.add_argument("--ros-domain-id", type=int)
parser.add_argument("--episodes", default="")
args, _ = parser.parse_known_args()

mode = os.environ.get(f"FAKE_MODE_{args.algorithm_name}", os.environ.get("FAKE_MODE", "ok"))
uri = f"http://localhost:{args.ros_domain_id * 10 + 11345}"
sim = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(600)"],
                       env={**os.environ, "GAZEBO_MASTER_URI": uri},
                       start_new_session=(mode == "leftover"))
episodes = [e for e in args.episodes.split(",") if e] or \
    os.environ.get("FAKE_EPISODES", "episode_1,episode_2").split(",")
base = pathlib.Path(args.base_path_for_results) / "no_checkpoint" / args.algorithm_name / args.experiment
base.mkdir(parents=True, exist_ok=True)
outcomes = base / f"{args.algorithm_name}_outcomes.csv"
frame = pd.read_csv(outcomes) if outcomes.exists() else pd.DataFrame()
done = set() if frame.empty else set(zip(frame["experiment_tag"].astype(str), frame["run_id"]))
todo = [(e, r) for e in episodes for r in range(1, args.repetitions + 1) if (e, r) not in done]
if mode == "partial":
    todo = todo[: len(todo) // 2]
if mode == "hang":
    time.sleep(600)
for index, (episode, run) in enumerate(todo, start=1):
    time.sleep(float(os.environ.get("FAKE_SECONDS", "0.05")))
    row = pd.DataFrame({"experiment_tag": [episode], "run_id": [run],
                        "result": ["Success" if run % 2 else "Failure: Collision with agent"],
                        "ped_draw": [f"s7_e{index}"]})
    frame = pd.concat([frame, row], ignore_index=True)
    frame.to_csv(outcomes, index=False)
if mode != "leftover":
    sim.terminate()
print(f"Experiment outcomes saved to {outcomes}")
