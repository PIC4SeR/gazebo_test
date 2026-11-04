"""Lanes: where a batch runs, and how it is started, watched and torn down.

A lane is one ROS domain. ``ros2 gazeboexp run --ros-domain-id <d>`` puts the
whole stack of a batch in the tmux session ``gazebo_test_<d>``, with
``ROS_DOMAIN_ID=<d>`` and ``GAZEBO_MASTER_URI=http://localhost:<11345 + 10 d>`` in
the session environment, so every process of the lane carries that URI.
:class:`TmuxLaneBackend` relies on that: teardown kills the session, the CLI
process group, then every process whose environment holds the lane's URI (Gazebo
can outlive its tmux pane), and nothing else.

:class:`LaneBackend` is the seam for other ways of running a lane (containers,
see docs/gazebo_campaign_dashboard_plan.md, Part B).
"""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Protocol


def session_name(domain: int) -> str:
    """The tmux session gazebo_test's CLI creates for a domain."""
    return f"gazebo_test_{domain}"


def exact(session: str) -> str:
    """A tmux target matching only this session: a bare ``gazebo_test_1`` would
    also match ``gazebo_test_11`` by prefix when the first does not exist."""
    return f"={session}"


def gazebo_uri(domain: int) -> str:
    """The Gazebo master URI gazebo_test's CLI derives for a domain."""
    return f"http://localhost:{domain * 10 + 11345}"


@dataclass
class LaneHealth:
    rtf: Optional[float] = None  # real-time factor, 0..1
    sim_time: Optional[float] = None  # s
    paused: Optional[bool] = None
    sampled_at: float = field(default_factory=time.time)
    error: str = ""


class LaneBackend(Protocol):
    def start(self, domain: int, command: List[str], log_path: Path,
              env: Optional[Dict[str, str]] = None) -> None: ...

    def poll(self, domain: int) -> Optional[int]:
        """None while the batch runs, else the launcher's exit code."""

    def health(self, domain: int) -> LaneHealth: ...

    def occupied(self, domain: int) -> bool:
        """Something of this lane is alive that this backend did not start."""

    def teardown(self, domain: int) -> List[str]:
        """Stop everything of the lane and nothing else; returns what was stopped."""

    def pane_tail(self, domain: int, lines: int = 40) -> str: ...


def parse_gz_stats(text: str) -> LaneHealth:
    """The last sample of ``gz stats -p``: ``rtf, sim s, real s, paused (T/F)``. The
    header says "percent", but the first column is a factor (0.31 = 31 %)."""
    for line in reversed(text.splitlines()):
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 4 or line.lstrip().startswith("#"):
            continue
        try:
            return LaneHealth(rtf=float(parts[0]), sim_time=float(parts[1]),
                              paused=parts[3].upper().startswith("T"))
        except ValueError:
            continue
    message = text.strip().splitlines()[-1] if text.strip() else "no gz stats output"
    return LaneHealth(error=message[:200])


def lane_pids(uri: str, exclude: Optional[set] = None) -> List[int]:
    """Every process whose environment holds ``GAZEBO_MASTER_URI=<uri>``."""
    needle = f"GAZEBO_MASTER_URI={uri}".encode()
    exclude = (exclude or set()) | {os.getpid()}
    pids = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) in exclude:
            continue
        try:
            environ = (entry / "environ").read_bytes()
        except OSError:  # gone, or another user's process
            continue
        if needle in environ.split(b"\0"):
            pids.append(int(entry.name))
    return pids


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    try:  # a zombie is dead for our purposes
        return Path(f"/proc/{pid}/stat").read_text().split(")")[-1].split()[0] != "Z"
    except OSError:
        return False


def _signal_all(pids: List[int], sig: int) -> None:
    for pid in pids:
        try:
            os.kill(pid, sig)
        except (ProcessLookupError, PermissionError):
            pass


def _wait_gone(pids: List[int], timeout: float) -> List[int]:
    deadline = time.time() + timeout
    left = [p for p in pids if _alive(p)]
    while left and time.time() < deadline:
        time.sleep(0.2)
        left = [p for p in left if _alive(p)]
    return left


class TmuxLaneBackend:
    """Runs ``gazeboexp run`` as a child process per lane (it builds the tmux
    session itself) and samples Gazebo with ``gz stats``."""

    def __init__(self, gz_stats: bool = True, kill_grace_s: float = 10.0) -> None:
        self._procs: Dict[int, subprocess.Popen] = {}
        self._lock = threading.Lock()
        self.gz_stats = gz_stats
        self.kill_grace_s = kill_grace_s

    def start(self, domain: int, command: List[str], log_path: Path,
              env: Optional[Dict[str, str]] = None) -> None:
        with self._lock:
            proc = self._procs.get(domain)
            if proc is not None and proc.poll() is None:
                raise RuntimeError(f"lane {domain} is busy")
            log_path.parent.mkdir(parents=True, exist_ok=True)
            with log_path.open("w") as log:
                log.write("$ " + " ".join(command) + "\n\n")
                log.flush()
                # Own process group: stopping a lane never signals the dashboard.
                self._procs[domain] = subprocess.Popen(
                    command, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                    start_new_session=True, env=env)

    def poll(self, domain: int) -> Optional[int]:
        with self._lock:
            proc = self._procs.get(domain)
            if proc is None:
                raise KeyError(f"lane {domain} has no batch")
            return proc.poll()

    def health(self, domain: int) -> LaneHealth:
        if not self.gz_stats:
            return LaneHealth(error="gz stats disabled")
        env = {**os.environ, "GAZEBO_MASTER_URI": gazebo_uri(domain)}
        try:
            out = subprocess.run(["gz", "stats", "-p", "-d", "2"], capture_output=True,
                                 text=True, timeout=8, env=env)
            return parse_gz_stats(out.stdout + out.stderr)
        except subprocess.TimeoutExpired as exc:
            text = (exc.stdout or b"")
            text = text.decode(errors="replace") if isinstance(text, bytes) else text
            return parse_gz_stats(text)
        except FileNotFoundError:
            return LaneHealth(error="gz not found")

    def occupied(self, domain: int) -> bool:
        with self._lock:
            proc = self._procs.get(domain)
            ours = proc is not None and proc.poll() is None
        return not ours and tmux_session_exists(session_name(domain))

    def teardown(self, domain: int) -> List[str]:
        stopped: List[str] = []
        if subprocess.run(["tmux", "kill-session", "-t", exact(session_name(domain))],
                          capture_output=True).returncode == 0:
            stopped.append(f"tmux session {session_name(domain)}")
        with self._lock:
            proc = self._procs.get(domain)
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=self.kill_grace_s)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
            except ProcessLookupError:
                pass
            stopped.append(f"launcher pid {proc.pid}")
        exclude = {proc.pid} if proc is not None else set()
        pids = lane_pids(gazebo_uri(domain), exclude)
        if pids:
            _signal_all(pids, signal.SIGINT)
            left = _wait_gone(pids, self.kill_grace_s)
            _signal_all(left, signal.SIGKILL)
            left = _wait_gone(left, 5)
            stopped.append(f"{len(pids) - len(left)} lane process(es)"
                           + (f", {len(left)} still alive: {left}" if left else ""))
        return stopped

    def pane_tail(self, domain: int, lines: int = 40) -> str:
        """The experiment manager's window (the session's last one)."""
        out = subprocess.run(["tmux", "capture-pane", "-p", "-J", "-t",
                              f"{exact(session_name(domain))}:{{end}}", "-S", f"-{lines}"],
                             capture_output=True, text=True)
        if out.returncode != 0:
            return ""
        return "\n".join(out.stdout.rstrip().splitlines()[-lines:])


def tmux_session_exists(session: str) -> bool:
    return subprocess.run(["tmux", "has-session", "-t", exact(session)],
                          capture_output=True).returncode == 0


def log_tail(path: Optional[Path], lines: int = 40) -> str:
    if not path or not Path(path).exists():
        return ""
    with Path(path).open("rb") as fh:
        fh.seek(0, os.SEEK_END)
        fh.seek(max(0, fh.tell() - 16000))
        return "\n".join(fh.read().decode(errors="replace").splitlines()[-lines:])
