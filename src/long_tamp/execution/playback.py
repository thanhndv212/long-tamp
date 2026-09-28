"""Play a time-parameterized path as an execution backend (viewer or headless)."""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any

from .contract import ExecutionCommand, ExecutionStatus, Feedback


def _t0(path: Any) -> float:
    if not hasattr(path, "timeRange"):
        return 0.0
    tr = path.timeRange()
    return tr.first if hasattr(tr, "first") else tr[0]


class PathPlaybackBackend:
    """Executes a command by playing its path in (scaled) real time.

    ``command.payload`` is a path (anything with ``length()`` and
    ``eval(t) -> (q, ok)``, like an HPP path) or a key that ``get_path`` turns
    into one (an HPP path id). Each poll evaluates the path at the elapsed time
    times ``speed``, hands the configuration to ``display`` (a viewer; ``None``
    plays headless) and reports progress as a heartbeat.
    """

    def __init__(
        self,
        get_path: Callable[[Any], Any] | None = None,
        display: Callable[[Any], None] | None = None,
        speed: float = 1.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.get_path = get_path
        self.display = display
        self.speed = speed
        self.clock = clock
        self._path: Any = None
        self._started: float | None = None
        self.cancelled = False

    def start(self, command: ExecutionCommand) -> ExecutionStatus:
        payload = command.payload
        path = self.get_path(payload) if self.get_path is not None else payload
        if path is None or not hasattr(path, "eval"):
            return ExecutionStatus.FAILURE
        self._path, self._started, self.cancelled = path, self.clock(), False
        return ExecutionStatus.RUNNING

    def poll(self) -> tuple[ExecutionStatus, Feedback | None]:
        if self._path is None:
            return ExecutionStatus.FAILURE, Feedback(message="nothing started")
        if self.cancelled:
            return ExecutionStatus.FAILURE, Feedback(message="cancelled")
        length = self._path.length()
        t = min(length, (self.clock() - self._started) * self.speed)
        q, ok = self._path.eval(_t0(self._path) + t)
        if not ok:
            return ExecutionStatus.FAILURE, Feedback(
                message=f"path evaluation failed at t={t:.3f}"
            )
        if self.display is not None:
            self.display(q)
        progress = t / length if length > 0 else 1.0
        if t >= length:
            return ExecutionStatus.SUCCESS, Feedback(progress=1.0)
        return ExecutionStatus.RUNNING, Feedback(progress=progress)

    def cancel(self) -> None:
        self.cancelled = True
