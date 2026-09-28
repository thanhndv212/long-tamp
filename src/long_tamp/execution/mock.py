"""A scriptable in-memory backend that can produce every status (tests, demos)."""

from __future__ import annotations

import time
from collections.abc import Callable

from .contract import ExecutionCommand, ExecutionStatus, Feedback


class MockBackend:
    """Runs a command in ``duration / rtf`` seconds of ``clock`` time.

    - ``rtf``: real-time factor (0.4 = a simulator at 40 % of real time);
    - ``busy_starts``: the first N ``start`` calls answer ``BUSY``;
    - ``fail_at``: fail this many seconds after starting, with ``message``;
    - ``stall_at``: stop sending heartbeats (and never finish) from then on.

    A heartbeat is sent on every poll while progressing.
    """

    def __init__(
        self,
        clock: Callable[[], float] = time.monotonic,
        rtf: float = 1.0,
        busy_starts: int = 0,
        fail_at: float | None = None,
        stall_at: float | None = None,
        message: str = "mock failure",
    ) -> None:
        self.clock = clock
        self.rtf = rtf
        self.busy_starts = busy_starts
        self.fail_at = fail_at
        self.stall_at = stall_at
        self.message = message
        self.cancelled = False
        self.command: ExecutionCommand | None = None
        self._t_start: float | None = None

    def start(self, command: ExecutionCommand) -> ExecutionStatus:
        if self.busy_starts > 0:
            self.busy_starts -= 1
            return ExecutionStatus.BUSY
        self.command = command
        self.cancelled = False
        self._t_start = self.clock()
        return ExecutionStatus.RUNNING

    def poll(self) -> tuple[ExecutionStatus, Feedback | None]:
        if self._t_start is None:
            return ExecutionStatus.BUSY, None
        if self.cancelled:
            return ExecutionStatus.FAILURE, Feedback(message="cancelled")
        t = self.clock() - self._t_start
        duration = (self.command.duration if self.command else None) or 1.0
        if self.fail_at is not None and t >= self.fail_at:
            return ExecutionStatus.FAILURE, Feedback(message=self.message)
        if self.stall_at is not None and t >= self.stall_at:
            return ExecutionStatus.RUNNING, None
        progress = min(1.0, t * self.rtf / duration)
        if progress >= 1.0:
            return ExecutionStatus.SUCCESS, Feedback(progress=1.0)
        return ExecutionStatus.RUNNING, Feedback(progress=progress)

    def cancel(self) -> None:
        self.cancelled = True
