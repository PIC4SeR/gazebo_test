# Optuna parameter tuning for `gazebo_test` controllers

This document explains, for the `gazebo_test` benchmark, **which controller parameters
are tunable, how to obtain them, how to drive Optuna over them, whether to optimize
per-scenario or across all experiments, and how to define the objectives.**

It reconciles three pieces of the codebase:

- `gazebo_optimization/scripts/optuna_optimization` — the only existing Optuna driver.
- `nav2_profiles` — the Jinja2
  **template registry** that defines each controller's full parameter surface.
- `gazebo_test_cli` (`ros2 gazeboexp run`) — the runner that executes one full benchmark
  given a navigator/params file.

---

## 1. The two mechanisms (do not conflate them)

| Mechanism | What it does | File(s) |
|---|---|---|
| **Templates** | Define the *full tunable surface* per controller; rendered into a `nav2_params.yaml` at navigator-selection time. | `nav2_profiles/templates/controllers/*.yaml.j2` |
| **Optuna driver** | Samples values, writes a params YAML, runs one benchmark, reads back a cost. | `gazebo_optimization/scripts/optuna_optimization` |

**Critical limitation of the current driver:** `substitute_controller_params()` only
descends into

```
controller_server → ros__parameters → FollowPath → optimizer → weights
```

and only calls `trial.suggest_float(key, min, max)`. Any key that is not a float living
in that exact subtree is silently dropped (`Warning: <key> not found`). That subtree
exists **only for the MPC family** (`mpc_base` / `mpc_sfm_motion_model` /
`mpc_enlarged_state`, and the legacy `social_mpc` `FollowPath`). For MPPI / DWB / DWA /
ORCA / SFM / SICNav / CrowdNav the weights live elsewhere, so the driver must be
generalized first (see §4).

---

## 2. How many parameters are realistically tunable, per controller

"Realistic continuous knobs" = the cost weights / scales / force factors you would
actually sweep, excluding purely structural ints (horizon, batch size) you normally freeze.

| Controller (template) | Where weights live | Realistic knobs | Reachable by current driver |
|---|---|---|---|
| **MPC** base / social_motion / enlarged_state | `optimizer.weights` | **16** weights (≈7 base + 6 social + 3 enlarged), only the enabled critics matter | ✅ yes |
| legacy social_mpc `FollowPath` | `optimizer.weights` | **9** weights (as-shipped default) | ✅ yes |
| **MPPI** | per-critic `cost_weight` | ~**12–20** (8 critic weights + obstacle repulsion/critical + `vx/vy/wz_std`, `temperature`, `gamma`) | ❌ needs §4 |
| **DWB** | per-critic `scale` | ~**6–12** | ❌ needs §4 |
| **DWA / SFW** | flat `*_weight` | ~**8** | ❌ needs §4 |
| **pure_sfm** | flat | ~**4–7** (`force_factor_*`, `force_sigma_obstacle`) | ❌ needs §4 |
| **ORCA** | flat | ~**6–8** (mostly geometric) | ❌ needs §4 |
| **SICNav / CrowdNav** (pybind) | flat / inside policy | ~**5–8** exposed; real policy weights live in the pybind policy and are solve-time constrained | ❌ needs §4 |

**Bottom line:** out of the box ≈ **9–16 params**, MPC family only. Everything else
requires the §4 generalization. In practice, freeze the structural ints and tune
**5–10 cost weights** regardless of controller.

---

## 3. How to obtain the parameter list for a given controller

### 3.1 Read the template directly

Each controller's tunable surface is its Jinja2 template:

```bash
ls src/nav2_profiles/templates/controllers/
# mpc_controller.yaml.j2, mppi_controller.yaml.j2, dwb_controller.yaml.j2, ...
```

The `weights:` / `cost_weight:` / `scale:` / `force_factor_*` lines are your candidates.

### 3.2 Render the actual YAML for the exact navigator you will run

The navigators (`MPC_base`, `social_MPPI`, `SICNAV`, …) are defined in
`nav2_profiles/navigators/navigators.yaml`. Rendering one produces the concrete
`nav2_params.yaml` with all keys and current defaults:

```bash
ros2 gazeboexp run --help          # lists flags
ros2 gazeboexp run social_nav --navigator MPC_base --print-navigators  # list names

# Render only (the runner writes a temp YAML via get_navigator_yaml());
# easiest is to copy what it prints as "Generated navigation parameters file at: <path>"
python3 - <<'PY'
from nav2_profiles.navigator_utils import get_navigator_yaml
print(get_navigator_yaml("MPC_base"))   # path to rendered YAML
PY
```

Open that YAML, find the `weights:` (or critic) block, and copy the keys + current
values. Those keys, and the path to them, are exactly what your search space must target.

### 3.3 Decide ranges

- Use the existing `params_to_optimize.yaml` as the format: `{min, max}` per key.
- Anchor ranges around the rendered default (e.g. `[0, 2–3× default]` for weights).
- The MPC template comments list the *effective* (peak-normalized) weight next to each
  raw weight — use those to keep magnitudes comparable across terms.

---

## 4. Generalizing the Optuna driver to reach any controller

The current `substitute_controller_params` is hard-coded to the MPC weight path and to
floats. Replace it with a recursive path-walk + typed suggestions so the search space
can target arbitrary nested keys.

**Search-space file** (`params_to_optimize.yaml`) — make each entry a fully-qualified
path with a type:

```yaml
parameters:
  # MPC family
  - path: controller_server.ros__parameters.FollowPath.optimizer.weights.social_weight
    type: float
    min: 0.0
    max: 60.0
  - path: controller_server.ros__parameters.FollowPath.optimizer.weights.proxemics_weight
    type: float
    min: 0.0
    max: 30.0
  # MPPI example (different subtree)
  - path: controller_server.ros__parameters.FollowPath.ObstaclesCritic.critical_weight
    type: float
    min: 1.0
    max: 40.0
  - path: controller_server.ros__parameters.FollowPath.vx_std
    type: float
    min: 0.1
    max: 0.6
    log: true
  - path: controller_server.ros__parameters.FollowPath.batch_size
    type: int
    min: 256
    max: 1500
```

**Driver changes (sketch):**

```python
def set_by_path(d, dotted, value):
    keys = dotted.split(".")
    for k in keys[:-1]:
        d = d.setdefault(k, {})
    d[keys[-1]] = value

def suggest(trial, spec):
    name = spec["path"]
    if spec["type"] == "int":
        return trial.suggest_int(name, spec["min"], spec["max"], log=spec.get("log", False))
    if spec["type"] == "categorical":
        return trial.suggest_categorical(name, spec["choices"])
    return trial.suggest_float(name, spec["min"], spec["max"], log=spec.get("log", False))

def build_params(trial, base_yaml, search_space):
    params = copy.deepcopy(base_yaml)
    for spec in search_space["parameters"]:
        set_by_path(params, spec["path"], suggest(trial, spec))
    return params
```

This makes the count in §2 actually reachable for every controller. Keep the rendered
navigator YAML (`get_navigator_yaml`) as the *base*, overwrite the sampled keys, write to
`/tmp/nav2_params.yaml`, and pass `--nav-params /tmp/nav2_params.yaml`.

---

## 5. How one Optuna trial runs

One trial = one full `gazebo_test` benchmark. The runner is:

```bash
ros2 gazeboexp run <scenario> \
  --nav-params /tmp/nav2_params.yaml \   # sampled params (overrides --navigator)
  --hunav-eval \                         # produce social metrics.csv
  --headless --no-rviz \
  --algorithm-name <study_trial_id> \
  --base-path-for-results <results_root>
```

Results land under
`<results_root>/<checkpoint_ns>/<algorithm_name>/<scenario>/...` with:

- `<algorithm>_outcomes.csv` — per `(experiment_tag, run_id)`: the `result`
  (SUCCESS / FAILURE_*) merged with the hunav metrics.
- `metrics.csv` per run — the hunav social metrics.

**Read the cost from `outcomes.csv`**, not from stdout parsing — it already aggregates
all `experiment_tag`s and repetitions and includes the success/failure result, which you
need for constraint handling (§7).

---

## 6. Per-scenario vs. across-all-experiments

A `gazebo_test` "scenario" (`social_nav`, `social_env_test`, `social_nav_obstacles`,
`social_scenarios`, …; see `experiments.yaml`) is a distinct world + agents + goals.
Within a scenario there are several `experiment_tag`s × `repetitions`.

**Recommendation: do both, in two stages.**

1. **Per-scenario studies first.** Optimal social weights are scenario-dependent
   (a crossing rewards yielding/`crossing_weight`; an open corridor rewards
   `goal_proximity`/speed). One global optimum tends to be a mediocre compromise.
   Run one study per scenario; this also keeps trials cheap (one world per run) and the
   objective signal clean.

2. **One robustness study across all scenarios** to pick the single config you will
   actually deploy. Run every trial over **all** scenarios and aggregate (see §7). Seed
   this study's sampler with the per-scenario winners (`study.enqueue_trial(...)`) so it
   starts from known-good points.

Avoid optimizing one set of weights on a single scenario and assuming it transfers — the
template defaults already encode a hand-tuned compromise; beating it requires the
cross-scenario stage.

Always average over the **repetitions** within a scenario (the sim is stochastic — agent
policies, spawn jitter). Tune `repetitions` in `experiment_config.yaml`; ≥3 is a
reasonable floor for a stable mean.

---

## 7. Setting the objectives

### 7.1 Available metrics

`objectives.yaml` already enumerates the intended objectives:
`time_to_reach_goal`, `social_work_step`, `path_length`, `avg_speed`,
`personal_space_violation` (all `minimize`). The hunav evaluator emits these (plus
others) into `metrics.csv`; the merged `outcomes.csv` is the single source of truth.

> Note: `objectives.yaml` is currently *not read* by the driver (objectives are
> hard-coded to 3). Wire it in as part of §4 so the active set + directions come from
> config.

### 7.2 Single- vs multi-objective

- **Multi-objective (recommended):** social navigation is inherently a trade-off
  (fast/short path vs. low social work / personal-space respect). Use
  `optuna.create_study(directions=[...])` and inspect `study.best_trials` (the Pareto
  front). Keep it to **2–3 genuinely conflicting** objectives, e.g.
  `time_to_reach_goal` ↓ vs `social_work` ↓ (optionally `personal_space_violation` ↓).
  More than 3 makes the front sparse and uninterpretable at this trial budget.

- **Single scalar (if you need one number, e.g. for ASHA pruning):** combine a small set
  with **explicit normalization**, since `path_length` (m), `time` (s) and `social_work`
  (J-like) have different scales. Normalize each by the template-default config's value
  (run the default once to get baselines) so each term is ≈1.0 at baseline:

  ```
  cost = w_t * time/time_base + w_s * social_work/social_base + w_p * psv/psv_base
  ```

  Read weights from `objectives.yaml` (`weight`, `active`, `minimize`).

### 7.3 Handle failures explicitly (most important)

A trial that **collides or times out** has meaningless metric values (short path because
it never reached the goal looks "good"). Treat the result column from `outcomes.csv`:

- If any/most runs are `FAILURE_*`, return a large penalty (single-objective) or
  `float("inf")` per objective (multi-objective), or use `optuna.TrialPruned`.
- Or use a **constraint**: success-rate ≥ threshold, exposed via
  `trial.set_user_attr` + a constrained sampler (`NSGAIISampler(constraints_func=...)`).

Without this, Optuna will happily exploit the failure modes.

### 7.4 Aggregation order

Per trial: for each scenario, average each metric over `experiment_tag`s × `repetitions`
→ get per-scenario metric vector. For a cross-scenario study, aggregate across scenarios
with **mean (or worst-case max for robustness)**, after per-scenario normalization.

---

## 8. Recommended recipe

1. Pick the controller; render its navigator YAML (§3.2) and read the weight block.
2. Write `params_to_optimize.yaml` as fully-qualified paths (§4), **5–10 cost weights**,
   structural ints frozen.
3. Generalize the driver (§4): recursive set-by-path, typed suggestions, config-driven
   objectives, read cost from `outcomes.csv`, failure penalty (§7.3).
4. **Stage 1:** one multi-objective study per scenario,
   `directions=["minimize","minimize"]` (time vs social_work), `repetitions ≥ 3`,
   `n_trials ≈ 15–30 × n_params`.
5. **Stage 2:** one cross-scenario robustness study over all scenarios, seeded with
   stage-1 winners; aggregate per §7.4.
6. Use a persistent `storage="sqlite:///..."` + `load_if_exists=True` (already in the
   script) so studies resume — each trial is minutes of sim time.

### Budget reality

Each trial is a full Gazebo run × scenarios × repetitions. With multi-objective TPE/NSGA
you need ~15–30× `n_params` trials for a usable front: 8 params → ~150–250 trials → many
hours to days of sim time. This budget, not Optuna, is the real ceiling — which is why
**5–10 weights and 2–3 objectives** is the practical sweet spot.

---

## 9. Quick reference

- Templates / surface: `nav2_profiles/templates/controllers/*.yaml.j2`
- Navigator definitions: `nav2_profiles/navigators/navigators.yaml`
- Render a navigator: `nav2_profiles.navigator_utils.get_navigator_yaml(<NAME>)`
- Scenarios: `gazebo_experiments/experiments/experiments.yaml`
- Objectives config: `gazebo_optimization/config/objectives.yaml`
- Search space: `gazebo_optimization/config/params_to_optimize.yaml`
- Runner: `ros2 gazeboexp run <scenario> --nav-params <yaml> --hunav-eval --headless --no-rviz --algorithm-name <id> --base-path-for-results <root>`
- Results truth: `<root>/<ns>/<algorithm>/<scenario>/<algorithm>_outcomes.csv`
- Optuna driver: `gazebo_optimization/scripts/optuna_optimization`
</content>
</invoke>
