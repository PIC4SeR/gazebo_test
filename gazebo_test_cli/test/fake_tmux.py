"""A stand-in for the parts of libtmux that ``full_experiment.run`` uses.

It records, per session, the windows in creation order with their final name
and every command sent to their pane, so tests can compare the tmux lane a run
builds without a tmux server.
"""

from __future__ import annotations

from typing import Dict, List, Optional


class FakePane:
    def __init__(self, window: "FakeWindow") -> None:
        self.window = window
        self.sent: List[str] = []

    def select(self) -> None:
        pass

    def send_keys(self, command: str) -> None:
        self.sent.append(command)

    def capture_pane(self) -> List[str]:
        return list(self.window.session.server.pane_output)


class FakeWindow:
    def __init__(self, session: "FakeSession", name: str) -> None:
        self.session = session
        self.name = name
        self.panes = [FakePane(self)]

    def rename_window(self, name: str) -> None:
        self.name = name


class FakeSession:
    def __init__(self, server: "FakeServer", name: str, environment: Dict[str, str]) -> None:
        self.server = server
        self.name = name
        self.environment = dict(environment)
        self.windows = [FakeWindow(self, "")]
        self.active_window = self.windows[0]

    def getenv(self, key: str) -> Optional[str]:
        return self.environment.get(key)

    def new_window(self, attach: bool = True, window_name: str = "") -> FakeWindow:
        window = FakeWindow(self, window_name)
        self.windows.append(window)
        self.active_window = window
        return window

    def __enter__(self) -> "FakeSession":
        return self

    def __exit__(self, *exc) -> None:
        self.server.closed.append(self.name)

    def record(self) -> Dict:
        return {
            "session": self.name,
            "environment": self.environment,
            "windows": [[w.name, w.panes[0].sent] for w in self.windows],
        }


class FakeServer:
    """``libtmux.Server`` replacement; ``instances`` collects every server made."""

    instances: List["FakeServer"] = []
    pane_output: List[str] = []

    def __init__(self) -> None:
        self.sessions: List[str] = []
        self.created: List[FakeSession] = []
        self.commands: List[tuple] = []
        self.closed: List[str] = []
        FakeServer.instances.append(self)

    def new_session(self, session_name: str, environment: Dict[str, str]) -> FakeSession:
        session = FakeSession(self, session_name, environment)
        self.created.append(session)
        return session

    def cmd(self, *args) -> None:
        self.commands.append(args)
