"""What gazebo_test recorded for each batch, read from its results tree.

The experiment manager writes one outcomes CSV per (algorithm, experiment):
``<base>/<namespace>/<algorithm>/<experiment>/<algorithm>_outcomes.csv``, one row
per run keyed by ``(experiment_tag, run_id)``. ``<namespace>`` is ``no_checkpoint``
without a checkpoint database, else a label derived from its DSN (the manager may
also take the DSN from its config file or the environment, so the file is found by
pattern, preferring the expected namespace). ``<experiment>`` is the experiment id
without an ``_<algorithm>`` suffix, as the CLI passes it.

Progress is never typed into the dashboard: a batch is done when its runs are in
the outcomes file.
"""

from __future__ import annotations

import hashlib
import re
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple
from urllib.parse import urlparse

import pandas as pd

SUCCESS = "Success"


def checkpoint_namespace(dsn: str) -> str:
    """The results sub-folder of a checkpoint DSN, exactly as
    ``ExperimentManager._derive_checkpoint_namespace`` computes it."""
    if not dsn:
        return "no_checkpoint"
    digest = hashlib.sha256(dsn.encode("utf-8")).hexdigest()[:12]
    components: List[str] = []
    parsed = urlparse(dsn)
    if parsed.scheme:
        if parsed.hostname:
            components.append(parsed.hostname)
        if parsed.path and parsed.path not in {"", "/"}:
            components.append(parsed.path.strip("/"))
        if parsed.port:
            components.append(str(parsed.port))
    if not components:
        for key, value in re.findall(r"([A-Za-z_]+)=([^\s]+)", dsn):
            if key.lower() in {"dbname", "database", "host", "port"} and value:
                components.append(value)
    sanitized = [re.sub(r"[^A-Za-z0-9_.-]+", "_", piece.strip())
                 for piece in components if piece.strip()]
    label = "_".join(filter(None, sanitized))
    return f"{label}__{digest}" if label else f"checkpoint__{digest}"


def experiment_identifier(experiment: str, algorithm: str) -> str:
    """The results folder of an experiment, as the CLI derives it."""
    return experiment.removesuffix(f"_{algorithm}")


def outcomes_path(base: str, dsn: str, experiment: str, algorithm: str) -> Optional[Path]:
    """The outcomes CSV of (experiment, algorithm), or None before the first run."""
    name = f"{algorithm}_outcomes.csv"
    tail = Path(algorithm) / experiment_identifier(experiment, algorithm) / name
    expected = Path(base) / checkpoint_namespace(dsn) / tail
    if expected.exists():
        return expected
    found = [p for p in Path(base).glob(f"*/{tail}") if p.is_file()]
    return max(found, key=lambda p: p.stat().st_mtime) if found else None


class OutcomesCache:
    """Outcomes CSVs, re-read only when they change."""

    def __init__(self) -> None:
        self._frames: Dict[Path, Tuple[int, pd.DataFrame]] = {}

    def frame(self, path: Optional[Path]) -> Optional[pd.DataFrame]:
        if path is None:
            return None
        try:
            mtime = path.stat().st_mtime_ns
        except FileNotFoundError:
            return None
        cached = self._frames.get(path)
        if cached and cached[0] == mtime:
            return cached[1]
        try:
            frame = pd.read_csv(path)
        except (pd.errors.EmptyDataError, pd.errors.ParserError, OSError):
            return None
        if "experiment_tag" not in frame or "run_id" not in frame:
            return None
        frame["experiment_tag"] = frame["experiment_tag"].astype(str)
        frame["run_id"] = pd.to_numeric(frame["run_id"], errors="coerce")
        self._frames[path] = (mtime, frame)
        return frame


def batch_rows(frame: Optional[pd.DataFrame], episodes: Iterable[str],
               repetitions: int) -> pd.DataFrame:
    """The batch's rows: its episodes (all when none are listed), runs 1..repetitions,
    one row per (episode, run) (the last one when a run was written twice)."""
    if frame is None or frame.empty:
        return pd.DataFrame(columns=["experiment_tag", "run_id", "result"])
    episodes = [e for e in episodes if e]
    mask = frame["run_id"].between(1, repetitions)
    if episodes:
        mask &= frame["experiment_tag"].isin(episodes)
    rows = frame[mask]
    return rows.drop_duplicates(subset=["experiment_tag", "run_id"], keep="last")


def summarize(rows: pd.DataFrame, expected: Optional[int]) -> Dict[str, Any]:
    done = int(len(rows))
    results = rows["result"].astype(str) if "result" in rows else pd.Series(dtype=str)
    successes = int((results == SUCCESS).sum())
    last = rows.iloc[-1] if done else None
    return {
        "done": done,
        "expected": expected,
        "complete": expected is not None and done >= expected,
        "success_rate": round(successes / done, 3) if done else None,
        "failures": {k: int(v) for k, v in results[results != SUCCESS].value_counts().items()},
        "last": None if last is None else {
            "episode": str(last["experiment_tag"]), "run_id": int(last["run_id"]),
            "result": str(last.get("result", "")),
        },
    }


_STARTING = re.compile(r"Starting (\S+) run (\d+)")


def current_run(manager_output: str) -> Optional[Dict[str, Any]]:
    """The run the experiment manager started last, from its log
    (``Starting <episode> run <n>``, logged by the job coordinator), or None."""
    matches = _STARTING.findall(manager_output or "")
    if not matches:
        return None
    episode, run_id = matches[-1]
    return {"episode": episode, "run_id": int(run_id)}


def pairing_check(frames: Dict[str, Optional[pd.DataFrame]]) -> Optional[Dict[str, Any]]:
    """Whether the algorithms of one experiment saw the same pedestrian draws.

    ``frames`` maps algorithm -> its outcomes frame. A run is *paired* when its
    ``ped_draw`` (seed + reset index) also appears in every other algorithm that has
    draws; paired tests blocked on ``ped_draw`` drop the others. While a campaign is
    running, the algorithms behind show as unpaired runs of the ones ahead. Returns
    None when no frame has a ``ped_draw`` column (deterministic pedestrians, or a
    HuNav build without the motion_models topic).
    """
    draws: Dict[str, List[str]] = {}
    for algorithm, frame in frames.items():
        if frame is not None and "ped_draw" in frame:
            values = frame["ped_draw"].dropna().astype(str)
            draws[algorithm] = list(values[values != ""])
    draws = {a: d for a, d in draws.items() if d}
    if not draws:
        return None
    common = set.intersection(*(set(d) for d in draws.values()))
    per_algorithm = {
        a: {"runs": len(d), "paired": sum(v in common for v in d),
            "unpaired": sum(v not in common for v in d),
            "repeated": len(d) - len(set(d))}
        for a, d in sorted(draws.items())
    }
    return {
        "algorithms": per_algorithm,
        "without_draws": sorted(a for a in frames if a not in draws),
        "common_draws": len(common),
        "ok": all(v["unpaired"] == 0 and v["repeated"] == 0 for v in per_algorithm.values()),
    }
