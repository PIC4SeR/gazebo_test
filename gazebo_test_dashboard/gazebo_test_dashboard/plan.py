"""The campaign plan and the batches built from it.

A plan (YAML) names the algorithms (an ``--algorithm-name`` and its flags), the
experiments (a gazebo_test experiment id with its repetitions, optionally a
subset of episodes) and the lanes the batches run on. Every (experiment,
algorithm) pair is one **batch**: one ``ros2 gazeboexp run`` call that runs all
its repetitions, because Gazebo + HuNav start-up costs far more than a run. An
experiment with ``split_episodes: true`` gives one batch per episode instead, to
keep a single launch short.

Batches never reorder runs: in simulation the pairing between algorithms comes
from the recorded pedestrian draw (``ped_draw``), not from the order.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import yaml

# Flags the dashboard sets per batch: a plan must not set them in common_args.
RESERVED_FLAGS = {
    "--algorithm-name", "--ros-domain-id", "--base-path-for-results", "--repetitions",
    "--episodes", "--checkpoint-dsn", "--disable-checkpoint", "--exp-config-pkg",
}
# Flags that would keep a batch from ever ending.
FORBIDDEN_FLAGS = {"--wait-after-finish"}

EpisodeResolver = Callable[[str], Optional[List[str]]]


@dataclass
class Algorithm:
    name: str  # --algorithm-name: the results folder and the outcomes file name
    args: List[str] = field(default_factory=list)


@dataclass
class Experiment:
    name: str  # gazebo_test experiment id (positional argument of the CLI)
    repetitions: int
    episodes: Optional[List[str]] = None  # subset; None = every episode of the goals file
    split_episodes: bool = False
    args: List[str] = field(default_factory=list)


@dataclass
class Lanes:
    domains: List[int]
    stagger_s: float = 5.0  # between two lane starts: spreads the start-up CPU spike
    batch_timeout_s: float = 5400.0  # wall clock per launch
    stall_timeout_s: float = 900.0  # no new outcome row for this long
    sim_stall_s: float = 300.0  # Gazebo sim time not advancing for this long
    max_attempts: int = 3  # launches per batch before it is marked failed


@dataclass
class Plan:
    path: Path
    algorithms: List[Algorithm]
    experiments: List[Experiment]
    lanes: Lanes
    results_base_path: str = "results/gazebo_campaign"
    state_file: str = "results/gazebo_campaign/dashboard/campaign.csv"
    cli: List[str] = field(default_factory=lambda: ["ros2", "gazeboexp"])
    common_args: List[str] = field(default_factory=list)
    checkpoint_dsn: str = ""
    exp_config_pkg: str = ""

    def algorithm(self, name: str) -> Algorithm:
        return next(a for a in self.algorithms if a.name == name)

    def experiment(self, name: str) -> Experiment:
        return next(e for e in self.experiments if e.name == name)


def _resolve(base: Path, value: str) -> str:
    path = Path(value).expanduser()
    return str(path if path.is_absolute() else (base / path).resolve())


def _str_list(value: Any, what: str) -> List[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [part.strip() for part in value.split(",") if part.strip()]
    if not isinstance(value, list):
        raise ValueError(f"{what} must be a list")
    return [str(v) for v in value]


def load_plan(path: Path) -> Plan:
    """Read and validate a plan; relative paths are relative to the plan file."""
    path = Path(path).resolve()
    raw: Dict[str, Any] = yaml.safe_load(path.read_text()) or {}
    base = path.parent
    algorithms = [Algorithm(name=str(a["name"]), args=_str_list(a.get("args"), "args"))
                  for a in raw.get("algorithms") or []]
    experiments = []
    for e in raw.get("experiments") or []:
        if isinstance(e, str):
            e = {"name": e}
        reps = e.get("repetitions", raw.get("repetitions"))
        if reps is None:
            raise ValueError(f"experiment {e.get('name')}: repetitions missing "
                             "(per experiment or top-level)")
        episodes = _str_list(e.get("episodes"), "episodes") or None
        experiments.append(Experiment(name=str(e["name"]), repetitions=int(reps),
                                      episodes=episodes,
                                      split_episodes=bool(e.get("split_episodes", False)),
                                      args=_str_list(e.get("args"), "args")))
    lanes_raw = raw.get("lanes") or {}
    domains = [int(d) for d in lanes_raw.get("domains") or []]
    lanes = Lanes(domains=domains, **{k: type(getattr(Lanes, k))(lanes_raw[k])
                                      for k in ("stagger_s", "batch_timeout_s", "stall_timeout_s",
                                                "sim_stall_s", "max_attempts") if k in lanes_raw})
    results = _resolve(base, raw.get("results_base_path", "results/gazebo_campaign"))
    state_file = raw.get("state_file") or str(Path(results) / "dashboard" / "campaign.csv")
    plan = Plan(path=path, algorithms=algorithms, experiments=experiments, lanes=lanes,
                results_base_path=results, state_file=_resolve(base, state_file),
                cli=_str_list(raw.get("cli"), "cli") or ["ros2", "gazeboexp"],
                common_args=_str_list(raw.get("common_args"), "common_args"),
                checkpoint_dsn=str(raw.get("checkpoint_dsn") or "").strip(),
                exp_config_pkg=str(raw.get("exp_config_pkg") or "").strip())
    validate(plan)
    return plan


def validate(plan: Plan) -> None:
    """Errors that make the plan unusable."""
    if not plan.algorithms:
        raise ValueError("the plan has no algorithms")
    if not plan.experiments:
        raise ValueError("the plan has no experiments")
    if not plan.lanes.domains:
        raise ValueError("lanes.domains is empty: give at least one ROS domain id")
    if len(set(plan.lanes.domains)) != len(plan.lanes.domains):
        raise ValueError("lanes.domains has duplicates")
    for domain in plan.lanes.domains:
        if not 0 <= domain <= 101:  # Gazebo port 11345 + 10 x domain must stay valid
            raise ValueError(f"lane domain {domain} out of range 0..101")
    for kind, items in (("algorithm", plan.algorithms), ("experiment", plan.experiments)):
        names = [i.name for i in items]
        duplicates = sorted({n for n in names if names.count(n) > 1})
        if duplicates:
            raise ValueError(f"duplicate {kind} names: {duplicates}")
    for exp in plan.experiments:
        if exp.repetitions < 1:
            raise ValueError(f"experiment {exp.name}: repetitions must be >= 1")
    flags = list(plan.common_args)
    for item in [*plan.algorithms, *plan.experiments]:
        flags += item.args
    reserved = sorted({f.split("=")[0] for f in flags} & RESERVED_FLAGS)
    if reserved:
        raise ValueError(f"set by the dashboard per batch, remove from the plan args: {reserved}"
                         " (use results_base_path, repetitions, episodes, checkpoint_dsn,"
                         " exp_config_pkg, lanes.domains)")
    forbidden = sorted({f.split("=")[0] for f in flags} & FORBIDDEN_FLAGS)
    if forbidden:
        raise ValueError(f"{forbidden} would keep a batch from ending")
    if plan.lanes.max_attempts < 1:
        raise ValueError("lanes.max_attempts must be >= 1")


def plan_warnings(plan: Plan, known_experiments: Optional[List[str]] = None,
                  known_navigators: Optional[List[str]] = None) -> List[str]:
    """Problems worth showing that do not stop the dashboard."""
    warnings: List[str] = []
    if 0 in plan.lanes.domains:
        warnings.append("lane domain 0 is the one the Optuna driver tears down "
                        "(tmux session gazebo_test_0); prefer other domains")
    if known_experiments is not None:
        unknown = [e.name for e in plan.experiments if e.name not in known_experiments]
        if unknown:
            warnings.append(f"experiments not registered by any package: {unknown}")
    if known_navigators is not None:
        for alg in plan.algorithms:
            if "--navigator" in alg.args:
                i = alg.args.index("--navigator")
                if i + 1 < len(alg.args) and alg.args[i + 1] not in known_navigators:
                    warnings.append(f"algorithm {alg.name}: navigator {alg.args[i + 1]} "
                                    "is not a nav2_profiles profile")
    return warnings


def batch_key(experiment: str, algorithm: str, episodes: Optional[List[str]]) -> str:
    return f"{experiment}|{algorithm}|{','.join(episodes) if episodes else '*'}"


def expand(plan: Plan, episode_resolver: Optional[EpisodeResolver] = None) -> List[Dict[str, Any]]:
    """The batches of the plan, experiment by experiment, algorithm by algorithm."""
    batches: List[Dict[str, Any]] = []
    for exp in plan.experiments:
        groups: List[Optional[List[str]]] = [exp.episodes]
        if exp.split_episodes:
            episodes = exp.episodes or (episode_resolver(exp.name) if episode_resolver else None)
            if not episodes:
                raise ValueError(f"experiment {exp.name}: split_episodes needs its episodes "
                                 "(list them in the plan, or install gazebo_test)")
            groups = [[e] for e in episodes]
        for alg in plan.algorithms:
            for episodes in groups:
                batches.append({
                    "key": batch_key(exp.name, alg.name, episodes),
                    "experiment": exp.name,
                    "algorithm": alg.name,
                    "episodes": ",".join(episodes) if episodes else "",
                    "repetitions": exp.repetitions,
                })
    return batches


def gazebo_episode_resolver() -> Optional[EpisodeResolver]:
    """Episodes of an experiment from its goals_and_poses file, through the
    gazebo_test experiment registry (None when gazebo_test is not installed)."""
    try:
        from gazebo_test.script_utils.parsing_utils import get_experiment_by_id
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return None
    cache: Dict[str, Optional[List[str]]] = {}

    def resolve(name: str) -> Optional[List[str]]:
        if name not in cache:
            try:
                experiment = get_experiment_by_id(name) or {}
                goals = Path(experiment["goals_and_poses"])
                if not goals.is_absolute():
                    goals = Path(get_package_share_directory(experiment["pkg_name"])) / goals
                data = yaml.safe_load(goals.read_text()) or {}
                cache[name] = [str(e) for e in data.get("episodes") or []] or None
            except Exception:  # noqa: BLE001 - unknown experiment or unreadable file
                cache[name] = None
        return cache[name]

    return resolve


def known_names() -> tuple:
    """Registered experiment ids and nav2_profiles navigators, or (None, None)."""
    experiments = navigators = None
    try:
        from gazebo_test.script_utils.parsing_utils import get_experiment_ids
        experiments = get_experiment_ids()
    except Exception:  # noqa: BLE001
        pass
    try:
        from gazebo_test.script_utils.navigator_utils import list_available_navigators
        navigators = list_available_navigators()
    except Exception:  # noqa: BLE001
        pass
    return experiments, navigators
