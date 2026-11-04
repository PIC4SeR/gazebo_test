"""The campaign: batches, their queue and the lanes that run them.

The batch list lives in a state file (``campaign.csv``): order, status, attempts,
lane, times, notes. Progress (runs done, results) is never stored there: it is
read from the outcomes CSVs each time (status.py).

:meth:`Campaign.tick` is the scheduler, called every few seconds by a thread
(or directly by tests):

* a running lane whose launcher exited is judged from its outcomes: all runs in
  -> ``done``; otherwise the batch goes back to ``pending`` (the next launch
  resumes with ``--resume-checkpoint``) or, after ``max_attempts``, ``failed``;
* a lane past ``batch_timeout_s``, without a new outcome row for
  ``stall_timeout_s``, or whose Gazebo sim time has not advanced for
  ``sim_stall_s``, is torn down and its batch handled the same way;
* while the campaign is running, a free lane takes the first pending batch, at
  least ``stagger_s`` after the previous start on any lane.

A lane whose tmux session exists without this dashboard having started it
(a crash, a manual run, a previous dashboard) is ``occupied`` and never used or
killed until the operator cleans it.
"""

from __future__ import annotations

import csv
import os
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

from . import status as S
from .lanes import LaneBackend, LaneHealth, log_tail, session_name
from .plan import EpisodeResolver, Plan, expand

# Optional tmux windows of a launch: name -> (flag, flag turns the window on?).
WINDOWS = {"gazebo_gui": ("--headless", False), "rviz": ("--no-rviz", False),
           "hunav_eval": ("--hunav-eval", True)}
COLUMNS = ["id", "order", "key", "experiment", "algorithm", "episodes", "repetitions",
           "status", "attempts", "lane", "started", "finished", "last_exit", "last_log",
           "reason", "notes"]
# Kept when the plan is regenerated (matched by batch key).
KEPT_COLUMNS = ["order", "status", "attempts", "lane", "started", "finished", "last_exit",
                "last_log", "reason", "notes"]
STATUSES = ("pending", "running", "done", "failed", "skipped")
HEALTH_EVERY_S = 20.0


def _now() -> str:
    return datetime.now().isoformat(timespec="seconds")


def read_state(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        return []
    with path.open(newline="") as fh:
        rows = list(csv.DictReader(fh))
    for row in rows:
        for key in ("id", "order", "repetitions", "attempts"):
            row[key] = int(row.get(key) or 0)
        for key in COLUMNS:
            row.setdefault(key, "")
    return rows


def write_state(path: Path, rows: List[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".csv.tmp")
    with tmp.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=COLUMNS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(sorted(rows, key=lambda r: r["order"]))
    tmp.replace(path)


def merge_state(batches: List[Dict[str, Any]], old: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """The plan's batches, keeping the state of those already in the old file
    (matched by key). New batches go to the end of the queue."""
    previous = {row["key"]: row for row in old}
    rows: List[Dict[str, Any]] = []
    next_order = max([r["order"] for r in old], default=0) + 1
    for i, batch in enumerate(batches, start=1):
        row = {key: "" for key in COLUMNS}
        row.update(batch, id=i, status="pending", attempts=0)
        if batch["key"] in previous:
            row.update({k: previous[batch["key"]].get(k, "") for k in KEPT_COLUMNS})
            row["attempts"] = int(row["attempts"] or 0)
        else:
            row["order"] = next_order
            next_order += 1
        rows.append(row)
    for n, row in enumerate(sorted(rows, key=lambda r: int(r["order"] or 0)), start=1):
        row["order"] = n
    return rows


@dataclass
class LaneState:
    domain: int
    enabled: bool = True
    batch: Optional[int] = None  # batch id
    started_at: Optional[float] = None
    log_path: Optional[Path] = None
    progress: int = -1  # outcome rows of the batch at the last change
    progress_at: Optional[float] = None
    health: Optional[LaneHealth] = None
    health_at: float = 0.0
    sim_time: Optional[float] = None  # last distinct sim time, and when it was seen
    sim_time_at: Optional[float] = None
    rtf_history: List[float] = field(default_factory=list)
    message: str = ""


class Campaign:
    def __init__(self, plan: Plan, backend: LaneBackend,
                 episode_resolver: Optional[EpisodeResolver] = None,
                 clock=time.time) -> None:
        self.plan = plan
        self.backend = backend
        self.resolve_episodes = episode_resolver
        self.clock = clock
        self.path = Path(plan.state_file)
        self.log_dir = self.path.parent / "logs"
        self.cache = S.OutcomesCache()
        self.lock = threading.RLock()
        self.running = False  # the campaign: may lanes take new batches?
        # Windows of the next launches; defaults from the plan's common_args.
        # ponytail: in memory only, a restart falls back to the plan
        self.windows = {name: (flag in plan.common_args) == on for name, (flag, on) in WINDOWS.items()}
        self.last_start: Optional[float] = None
        self.lanes = {d: LaneState(domain=d) for d in plan.lanes.domains}
        self.rows = merge_state(expand(plan, episode_resolver), read_state(self.path))
        for row in self.rows:
            if row["status"] == "running":  # the dashboard stopped under it
                row["status"] = "pending"
                row["reason"] = "interrupted: dashboard restarted (check the lane before cleaning it)"
            elif row["status"] == "done":  # the plan may now ask for more runs
                progress = self.progress(row)
                if progress["expected"] is not None and not progress["complete"]:
                    row["status"] = "pending"
                    row["reason"] = f"plan changed: {progress['done']}/{progress['expected']} runs"
        self._save()

    # --- state -------------------------------------------------------------
    def _save(self) -> None:
        write_state(self.path, self.rows)

    def batch(self, batch_id: int) -> Dict[str, Any]:
        for row in self.rows:
            if row["id"] == batch_id:
                return row
        raise KeyError(f"no batch {batch_id}")

    def _episodes(self, row: Dict[str, Any]) -> List[str]:
        if row["episodes"]:
            return row["episodes"].split(",")
        resolved = self.resolve_episodes(row["experiment"]) if self.resolve_episodes else None
        return list(resolved or [])

    def expected(self, row: Dict[str, Any]) -> Optional[int]:
        episodes = self._episodes(row)
        return len(episodes) * int(row["repetitions"]) if episodes else None

    def outcomes(self, experiment: str, algorithm: str):
        path = S.outcomes_path(self.plan.results_base_path, self.plan.checkpoint_dsn,
                               experiment, algorithm)
        return path, self.cache.frame(path)

    def progress(self, row: Dict[str, Any]) -> Dict[str, Any]:
        path, frame = self.outcomes(row["experiment"], row["algorithm"])
        rows = S.batch_rows(frame, row["episodes"].split(",") if row["episodes"] else [],
                            int(row["repetitions"]))
        summary = S.summarize(rows, self.expected(row))
        summary["outcomes"] = str(path) if path else ""
        return summary

    # --- launching ---------------------------------------------------------
    def command(self, row: Dict[str, Any]) -> List[str]:
        plan = self.plan
        exp = plan.experiment(row["experiment"])
        alg = plan.algorithm(row["algorithm"])
        window_flags = {flag for flag, _ in WINDOWS.values()}
        command = [*plan.cli, "run", exp.name, *[a for a in plan.common_args if a not in window_flags],
                   *[flag for name, (flag, on) in WINDOWS.items() if self.windows[name] == on],
                   *exp.args, *alg.args,
                   "--algorithm-name", alg.name,
                   "--repetitions", str(row["repetitions"]),
                   "--base-path-for-results", plan.results_base_path,
                   "--ros-domain-id", str(self.lane_of(row))]
        if row["episodes"]:
            command += ["--episodes", row["episodes"]]
        if plan.exp_config_pkg:
            command += ["--exp-config-pkg", plan.exp_config_pkg]
        if plan.checkpoint_dsn:
            command += ["--checkpoint-dsn", plan.checkpoint_dsn]
        else:
            command += ["--disable-checkpoint"]
        if "--resume-checkpoint" not in command:
            # Runs already in the outcomes (or the checkpoint store) are skipped,
            # so a requeued batch only runs what is missing.
            command.append("--resume-checkpoint")
        return command

    def lane_of(self, row: Dict[str, Any]) -> int:
        return int(row["lane"])

    def env(self) -> Dict[str, str]:
        env = dict(os.environ)
        if not self.plan.checkpoint_dsn:
            # Otherwise the manager falls back to it and writes under another namespace.
            env.pop("GAZEBO_TEST_CHECKPOINT_DSN", None)
        return env

    def _start(self, lane: LaneState, row: Dict[str, Any], now: float) -> None:
        row["lane"] = lane.domain
        row["attempts"] = int(row["attempts"]) + 1
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log = self.log_dir / f"batch{row['id']:03d}_{row['algorithm']}_{row['experiment']}" \
            f"_a{row['attempts']}_{stamp}.log"
        self.backend.start(lane.domain, self.command(row), log, self.env())
        row.update(status="running", started=_now(), finished="", last_log=str(log), reason="")
        lane.batch, lane.started_at, lane.log_path = row["id"], now, log
        lane.progress, lane.progress_at = self.progress(row)["done"], now
        lane.sim_time = lane.sim_time_at = None
        lane.health, lane.health_at, lane.rtf_history = None, 0.0, []
        lane.message = ""
        self.last_start = now

    def _finish(self, lane: LaneState, row: Dict[str, Any], exit_code: Optional[int],
                reason: str) -> None:
        """Judge a batch whose launch ended (by itself or torn down)."""
        progress = self.progress(row)
        row["finished"] = _now()
        row["last_exit"] = "" if exit_code is None else exit_code
        if progress["complete"] or (progress["expected"] is None and progress["done"] > 0
                                    and exit_code == 0 and not reason):
            row["status"], row["reason"] = "done", ""
        elif int(row["attempts"]) >= self.plan.lanes.max_attempts:
            row["status"] = "failed"
            row["reason"] = reason or f"{progress['done']}/{progress['expected']} runs after " \
                f"{row['attempts']} attempt(s)"
        else:
            row["status"] = "pending"
            row["reason"] = reason or f"incomplete ({progress['done']}/{progress['expected']}); " \
                "requeued"
        lane.batch, lane.started_at = None, None
        lane.message = f"batch {row['id']} {row['status']}" + (f": {row['reason']}" if row["reason"] else "")

    # --- the scheduler -----------------------------------------------------
    def tick(self) -> None:
        with self.lock:
            now = self.clock()
            for lane in self.lanes.values():
                if lane.batch is not None:
                    self._watch(lane, now)
            if self.running:
                self._dispatch(now)
            self._save()

    def _watch(self, lane: LaneState, now: float) -> None:
        row = self.batch(lane.batch)
        exit_code = self.backend.poll(lane.domain)
        if exit_code is not None:
            # The CLI's tmux session ends with it; anything left is torn down.
            self.backend.teardown(lane.domain)
            self._finish(lane, row, exit_code, "")
            return
        done = self.progress(row)["done"]
        if done != lane.progress:
            lane.progress, lane.progress_at = done, now
        limits = self.plan.lanes
        reason = ""
        if now - lane.started_at > limits.batch_timeout_s:
            reason = f"timeout: {limits.batch_timeout_s:g} s wall clock"
        elif now - (lane.progress_at or lane.started_at) > limits.stall_timeout_s:
            reason = f"stalled: no new outcome row for {limits.stall_timeout_s:g} s"
        elif lane.sim_time_at is not None and now - lane.sim_time_at > limits.sim_stall_s:
            reason = f"stalled: Gazebo sim time stuck at {lane.sim_time:.1f} s " \
                f"for {limits.sim_stall_s:g} s"
        if reason:
            stopped = self.backend.teardown(lane.domain)
            self._finish(lane, row, None, reason + (f" (stopped {', '.join(stopped)})" if stopped else ""))

    def sample_health(self) -> None:
        """Sample Gazebo on every running lane (``gz stats`` takes seconds, so this
        runs outside the lock, in its own thread)."""
        with self.lock:
            running = {d: lane.batch for d, lane in self.lanes.items() if lane.batch is not None}
        samples = {d: self.backend.health(d) for d in running}
        with self.lock:
            now = self.clock()
            for domain, health in samples.items():
                lane = self.lanes[domain]
                if lane.batch != running[domain]:
                    continue  # the batch changed meanwhile
                lane.health, lane.health_at = health, now
                if health.sim_time is not None:
                    if lane.sim_time is None or health.sim_time > lane.sim_time + 1e-6:
                        lane.sim_time, lane.sim_time_at = health.sim_time, now
                    if health.rtf is not None:
                        lane.rtf_history = (lane.rtf_history + [round(health.rtf, 3)])[-60:]

    def _dispatch(self, now: float) -> None:
        for lane in self.lanes.values():
            if lane.batch is not None or not lane.enabled:
                continue
            if self.last_start is not None and now - self.last_start < self.plan.lanes.stagger_s:
                return
            if self.backend.occupied(lane.domain):
                lane.message = f"occupied: tmux session {session_name(lane.domain)} exists " \
                    "(not started by this dashboard): Clean lane to free it"
                continue
            row = self.next_pending()
            if row is None:
                return
            self._start(lane, row, now)

    def next_pending(self) -> Optional[Dict[str, Any]]:
        busy = {lane.batch for lane in self.lanes.values() if lane.batch is not None}
        for row in sorted(self.rows, key=lambda r: r["order"]):
            if row["status"] == "pending" and row["id"] not in busy:
                return row
        return None

    # --- operator actions --------------------------------------------------
    def set_running(self, running: bool) -> Dict[str, Any]:
        with self.lock:
            self.running = running
            return {"ok": True, "running": running}

    def set_windows(self, windows: Dict[str, Any]) -> Dict[str, Any]:
        """Which optional windows the next launches open (running lanes keep theirs)."""
        with self.lock:
            unknown = set(windows) - set(WINDOWS)
            if unknown:
                raise ValueError(f"unknown windows {sorted(unknown)}; known: {sorted(WINDOWS)}")
            self.windows.update({k: bool(v) for k, v in windows.items()})
            return {"ok": True, "windows": dict(self.windows)}

    def stop_lane(self, domain: int) -> Dict[str, Any]:
        """Tear the lane down and disable it; its batch returns to the queue without
        counting the attempt."""
        with self.lock:
            lane = self._lane(domain)
            stopped = self.backend.teardown(domain)
            if lane.batch is not None:
                row = self.batch(lane.batch)
                row["attempts"] = max(0, int(row["attempts"]) - 1)
                row.update(status="pending", finished=_now(), reason="stopped by the operator")
                lane.batch, lane.started_at = None, None
            lane.enabled = False
            lane.message = "stopped by the operator: " + (", ".join(stopped) or "nothing was running")
            self._save()
            return {"ok": True, "stopped": stopped}

    def clean_lane(self, domain: int) -> Dict[str, Any]:
        """Tear down leftovers on an idle lane (an occupied session, orphans)."""
        with self.lock:
            lane = self._lane(domain)
            if lane.batch is not None:
                raise PermissionError(f"lane {domain} is running batch {lane.batch}: stop it first")
            stopped = self.backend.teardown(domain)
            lane.message = "cleaned: " + (", ".join(stopped) or "nothing to clean")
            return {"ok": True, "stopped": stopped}

    def enable_lane(self, domain: int, enabled: bool) -> Dict[str, Any]:
        with self.lock:
            self._lane(domain).enabled = enabled
            return {"ok": True}

    def _lane(self, domain: int) -> LaneState:
        if domain not in self.lanes:
            raise KeyError(f"no lane {domain}")
        return self.lanes[domain]

    def edit(self, batch_id: int, action: str, value: str = "") -> Dict[str, Any]:
        with self.lock:
            row = self.batch(batch_id)
            if action == "notes":
                row["notes"] = value
            elif action in ("skip", "unskip", "requeue"):
                if row["status"] == "running":
                    raise PermissionError(f"batch {batch_id} is running")
                if action == "skip":
                    row["status"] = "skipped"
                elif action == "unskip" and row["status"] == "skipped":
                    row["status"] = "pending"
                elif action == "requeue":
                    row.update(status="pending", attempts=0, reason="requeued by the operator")
            elif action == "run":  # value = lane domain; starts now, even when paused
                if row["status"] == "running":
                    raise PermissionError(f"batch {batch_id} is running")
                lane = self._lane(int(value))
                now = self.clock()
                if lane.batch is not None or not lane.enabled or self.backend.occupied(lane.domain):
                    raise PermissionError(f"lane {lane.domain} is not free")
                if self.last_start is not None and now - self.last_start < self.plan.lanes.stagger_s:
                    wait = self.plan.lanes.stagger_s - (now - self.last_start)
                    raise PermissionError(f"another lane started {now - self.last_start:.0f} s ago: "
                                          f"wait {wait:.0f} s (stagger_s)")
                self._start(lane, row, now)
            elif action in ("up", "down"):
                queue = sorted(self.rows, key=lambda r: r["order"])
                i = queue.index(row)
                j = i - 1 if action == "up" else i + 1
                if 0 <= j < len(queue):
                    queue[i]["order"], queue[j]["order"] = queue[j]["order"], queue[i]["order"]
            else:
                raise ValueError(f"unknown action {action}")
            self._save()
            return {"ok": True}

    def requeue_failed(self) -> Dict[str, Any]:
        with self.lock:
            failed = [r for r in self.rows if r["status"] == "failed"]
            for row in failed:
                row.update(status="pending", attempts=0, reason="requeued by the operator")
            self._save()
            return {"ok": True, "requeued": len(failed)}

    # --- views ---------------------------------------------------------------
    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            now = self.clock()
            batches = []
            for row in sorted(self.rows, key=lambda r: r["order"]):
                item = dict(row)
                item["progress"] = self.progress(row)
                batches.append(item)
            lanes = []
            for lane in self.lanes.values():
                row = self.batch(lane.batch) if lane.batch is not None else None
                h = lane.health
                # Enough history to still hold the last "Starting <episode> run <n>".
                pane = self.backend.pane_tail(lane.domain, 3000) if row else ""
                current = self._current(row, pane) if row else None
                lanes.append({
                    "domain": lane.domain,
                    "session": session_name(lane.domain),
                    "enabled": lane.enabled,
                    "state": ("running" if row else "disabled" if not lane.enabled
                              else "occupied" if lane.message.startswith("occupied") else "idle"),
                    "batch": None if row is None else {
                        "id": row["id"], "experiment": row["experiment"],
                        "algorithm": row["algorithm"], "episodes": row["episodes"],
                        "attempt": row["attempts"],
                        "current": current,
                        "progress": next(b["progress"] for b in batches if b["id"] == row["id"]),
                    },
                    "elapsed_s": round(now - lane.started_at) if lane.started_at else None,
                    "since_progress_s": round(now - lane.progress_at) if row and lane.progress_at else None,
                    "rtf": None if h is None else h.rtf,
                    "sim_time": None if h is None else h.sim_time,
                    "paused": None if h is None else h.paused,
                    "health_error": "" if h is None else h.error,
                    "rtf_history": list(lane.rtf_history),
                    "message": lane.message,
                    "log": str(lane.log_path) if lane.log_path else "",
                    "log_tail": log_tail(lane.log_path, 15) if row else "",
                    "pane_tail": "\n".join(pane.splitlines()[-25:]),
                })
            for item in batches:
                lane_info = next((l for l in lanes if l["batch"] and l["batch"]["id"] == item["id"]), None)
                item["current"] = lane_info["batch"]["current"] if lane_info else None
            return {
                "running": self.running,
                "windows": dict(self.windows),
                "plan": str(self.plan.path),
                "results": self.plan.results_base_path,
                "lanes": lanes,
                "batches": batches,
                "matrix": self.matrix(batches),
                "pairing": self.pairing(),
                "counts": {s: sum(1 for r in self.rows if r["status"] == s) for s in STATUSES},
                "stagger_left_s": max(0, round(self.plan.lanes.stagger_s - (now - self.last_start)))
                if self.last_start else 0,
            }

    def _current(self, row: Dict[str, Any], pane: str) -> Optional[Dict[str, Any]]:
        """The run in progress on a lane: episode, run, its position in the batch, and
        whether it already has its outcome row (then the stack is resetting)."""
        current = S.current_run(pane)
        if current is None:
            return None
        episodes = self._episodes(row)
        _, frame = self.outcomes(row["experiment"], row["algorithm"])
        recorded = S.batch_rows(frame, [current["episode"]], int(row["repetitions"]))
        current.update(
            repetitions=int(row["repetitions"]),
            episode_index=episodes.index(current["episode"]) + 1 if current["episode"] in episodes else None,
            episodes=len(episodes) or None,
            recorded=bool((recorded["run_id"] == current["run_id"]).any()),
        )
        return current

    def matrix(self, batches: List[Dict[str, Any]]) -> Dict[str, Any]:
        cells: Dict[str, Dict[str, Any]] = {}
        for b in batches:
            cell = cells.setdefault(f"{b['experiment']}|{b['algorithm']}", {
                "experiment": b["experiment"], "algorithm": b["algorithm"], "done": 0,
                "expected": 0, "successes": 0, "statuses": [], "attempts": 0,
                "outcomes": b["progress"]["outcomes"]})
            p = b["progress"]
            cell["done"] += p["done"]
            cell["expected"] = None if cell["expected"] is None or p["expected"] is None \
                else cell["expected"] + p["expected"]
            if p["success_rate"] is not None:
                cell["successes"] += round(p["success_rate"] * p["done"])
            cell["statuses"].append(b["status"])
            cell["attempts"] += int(b["attempts"])
        for cell in cells.values():
            cell["success_rate"] = round(cell["successes"] / cell["done"], 3) if cell["done"] else None
            statuses = cell.pop("statuses")
            cell["status"] = next((s for s in ("running", "failed", "pending", "skipped")
                                   if s in statuses), "done")
        return {"experiments": [e.name for e in self.plan.experiments],
                "algorithms": [a.name for a in self.plan.algorithms],
                "cells": cells}

    def pairing(self) -> Dict[str, Any]:
        out = {}
        for exp in self.plan.experiments:
            frames = {alg.name: self.outcomes(exp.name, alg.name)[1] for alg in self.plan.algorithms}
            check = S.pairing_check(frames)
            if check is not None:
                out[exp.name] = check
        return out
