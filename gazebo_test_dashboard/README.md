# gazebo_test_dashboard

A local web page that runs a gazebo_test **campaign**: experiments × algorithms,
each pair one **batch** (one `ros2 gazeboexp run` with all its repetitions),
queued over one or more **lanes** (ROS domains). It starts the batches, watches
them, tears down a lane that hangs, requeues what is incomplete, and shows the
progress, success rates and pedestrian-draw pairing it reads from the
gazebo_test outcomes.

Design and the later phases (containers, experiment tracking):
[docs/gazebo_campaign_dashboard_plan.md](../../../docs/gazebo_campaign_dashboard_plan.md).

## Start

```bash
colcon build --symlink-install --packages-select gazebo_test_dashboard
source install/setup.bash
cp $(ros2 pkg prefix gazebo_test_dashboard)/share/gazebo_test_dashboard/example_campaign.yaml campaign.yaml
# edit campaign.yaml, then
gazebo_test_dashboard --plan campaign.yaml            # http://127.0.0.1:8060
```

If Flask is only installed in a virtualenv, run it as a module from there:
`python3 -m gazebo_test_dashboard.server --plan campaign.yaml`.

| Option | Meaning |
| --- | --- |
| `--plan FILE` | the campaign plan (required) |
| `--start` | start the campaign at once (otherwise press **Start campaign**) |
| `--no-gz-stats` | do not sample Gazebo (no real-time factor, no sim-time stall check) |
| `--host`, `--port` | default `127.0.0.1:8060`. The page starts simulations: keep it local |

## The plan

```yaml
results_base_path: results/gazebo_campaign   # --base-path-for-results
checkpoint_dsn: ""                            # "" = --disable-checkpoint
# exp_config_pkg: benchmark_experiments
common_args: [--headless, --hunav-eval, --bag-record]
lanes:
  domains: [11, 12]          # one lane per ROS domain (tmux session gazebo_test_<d>)
  stagger_s: 45              # between two lane starts
  batch_timeout_s: 5400      # wall clock per launch
  stall_timeout_s: 900       # no new outcome row for this long
  sim_stall_s: 300           # Gazebo sim time stuck for this long
  max_attempts: 3            # launches per batch before "failed"
algorithms:
  - {name: dwb,  args: [--navigator, DWB]}
  - {name: mppi, args: [--navigator, MPPI]}
experiments:
  - {name: social_env_test, repetitions: 10}
  - {name: crossing, repetitions: 10, episodes: [episode_1, episode_2], split_episodes: true}
```

- Relative paths are relative to the plan file. The state file defaults to
  `<results_base_path>/dashboard/campaign.csv`; launch logs go to `logs/` next to it.
- The dashboard sets `--algorithm-name`, `--repetitions`, `--episodes`,
  `--ros-domain-id`, `--base-path-for-results`, `--checkpoint-dsn` /
  `--disable-checkpoint` and `--exp-config-pkg` itself, and always adds
  `--resume-checkpoint`. A plan that sets any of them in `args` is refused, and so
  is `--wait-after-finish` (the batch would never end).
- `split_episodes: true` makes one batch per episode: shorter launches, finer
  retries. The episodes come from the plan, else from the experiment's goals file.
- When the plan changes, batches are matched by (experiment, algorithm,
  episodes): status, attempts and notes are kept. New batches join the end of the
  queue. A `done` batch whose plan now asks for more repetitions is reopened.

## How a batch runs

1. A free, enabled lane takes the first pending batch, at least `stagger_s` after
   the previous start on any lane (lanes share `generatedWorld.world`; starting
   two scenes together can load the wrong world).
2. It runs `ros2 gazeboexp run <experiment> … --ros-domain-id <lane>`; the CLI
   builds the tmux session `gazebo_test_<lane>`. The launch's output is in the
   batch log; the page also shows the experiment manager's tmux window.
3. While it runs, the page shows runs done / expected, success rate, the last
   result, and Gazebo's real-time factor and sim time (`gz stats`, every 20 s).
4. When the launch ends, the batch is judged from the outcomes CSV: all runs in →
   **done**; otherwise **pending** again (the next launch only runs what is
   missing), or **failed** after `max_attempts`.
5. A launch past `batch_timeout_s`, without a new outcome row for
   `stall_timeout_s`, or whose sim time has not advanced for `sim_stall_s` is torn
   down and judged the same way, with the reason on the batch.

Teardown of a lane kills its tmux session, the launcher's process group, then
every process whose environment holds the lane's `GAZEBO_MASTER_URI`
(`http://localhost:<11345 + 10 × lane>`): Gazebo can outlive its pane. Other lanes'
processes are never touched. It also runs after every normal batch end, so no
leftover blocks the next batch.

## The page

- **Lanes**: state (running / idle / occupied / disabled), the batch and its
  progress, the run in progress ("now: episode_3 (3/10) · run 2/5", from the
  manager's `Starting <episode> run <n>` log line; "recorded, resetting" once its
  outcome row is in), RTF sparkline, logs, and `tmux attach -t gazebo_test_<lane>`.
  The queue shows the same episode and run next to a running batch.
  - **Stop**: tear the lane down and disable it; the batch goes back to the queue
    and the attempt does not count. **Enable** to use the lane again.
  - **Clean**: kill leftovers on an idle lane.
  - **Occupied**: a `gazebo_test_<lane>` session exists that this dashboard did
    not start (a manual run, a crash, a previous dashboard). The lane is skipped,
    never killed, until you Clean it.
- **Matrix**: experiments × algorithms: done / expected runs, success rate,
  launches, the outcomes file.
- **Pedestrian-draw pairing** (experiments whose outcomes have `ped_draw`): per
  algorithm, how many runs have a draw that every other algorithm also has.
  Paired tests blocked on `ped_draw` drop the others. While the campaign runs,
  the algorithms behind make the others look unpaired.
- **Queue**: order (↑ ↓), Skip / Unskip, Requeue (resets attempts), Notes;
  **Requeue failed**; exports of the campaign and the matrix as CSV.
- **Start / Pause campaign**: paused = no new batches; running ones finish.
- **Windows** (header checkboxes): Gazebo GUI (`--headless` off), RViz
  (`--no-rviz` off), HuNav eval (`--hunav-eval`) for the next launches; running
  lanes keep theirs. Defaults come from `common_args`; a restart goes back to them.

Restarting the dashboard keeps the queue. Batches that were running are set back
to pending with the reason "interrupted"; their launches keep running in their
own process groups, so their lanes show as occupied until they end or you Clean
them.

## Where results go

Exactly where `ros2 gazeboexp run` puts them:
`<results_base_path>/<namespace>/<algorithm>/<experiment>/<algorithm>_outcomes.csv`,
with `<namespace>` = `no_checkpoint` without a DSN. Without a DSN in the plan,
`GAZEBO_TEST_CHECKPOINT_DSN` is removed from the launch environment, so results
cannot silently land in another namespace.

## Limits (for now)

- Progress comes from the outcomes CSVs only; the checkpoint store is not read.
- Parallel lanes on one machine share CPU, GPU and the DDS participant limit:
  watch the RTF per lane before adding lanes.
- Two batches of the same (experiment, algorithm) with split episodes may run at
  the same time on different lanes. The experiment manager locks and merges the
  shared outcomes file.

## Tests

```bash
python3 -m pytest -q src/gazebo_test/gazebo_test_dashboard/test
```

They use `test/fake_gazeboexp.py` in place of the CLI, on lanes 93 and 94.
