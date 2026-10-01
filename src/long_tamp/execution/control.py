"""Pause, resume, stop and breakpoints at step boundaries (ADR-0004).

The executor calls ``checkpoint(step_id, "before")`` before starting a step and
``checkpoint(step_id, "after")`` once it is done. A checkpoint blocks while the
execution is paused (by ``pause()`` or by a breakpoint on that boundary) and
returns ``False`` if the execution was stopped, in which case the executor
does not start the step. Commands are never interrupted by a pause: they
finish, and the pause takes effect at the next boundary. ``stop()`` also
cancels a running command (see ``run_command``). Thread-safe: control calls
come from another thread (a UI, a ROS service) than the executor's.
"""

from __future__ import annotations

import threading
from collections.abc import Callable

_WHEN = ("before", "after")


class ExecutionControl:
    def __init__(self) -> None:
        self._cond = threading.Condition()
        self.paused = False
        self.stopped = False
        #: The boundary a paused executor is waiting at, or None.
        self.waiting_at: tuple[str, str] | None = None
        self._breakpoints: set[tuple[str, str]] = set()
        #: Called as ``on_wait(step_id, when, waiting)``: ``waiting=True``
        #: when a checkpoint starts blocking, then ``False`` when it returns
        #: (resumed or stopped). The executor uses it for ``pause`` events.
        self.on_wait: Callable[[str, str, bool], None] | None = None

    def pause(self) -> None:
        with self._cond:
            self.paused = True

    def resume(self) -> None:
        with self._cond:
            self.paused = False
            self._cond.notify_all()

    def stop(self) -> None:
        with self._cond:
            self.stopped = True
            self._cond.notify_all()

    def add_breakpoint(self, step_id: str, when: str = "before") -> None:
        if when not in _WHEN:
            raise ValueError(f"when must be one of {_WHEN}, got {when!r}")
        with self._cond:
            self._breakpoints.add((step_id, when))

    def remove_breakpoint(self, step_id: str, when: str = "before") -> None:
        with self._cond:
            self._breakpoints.discard((step_id, when))

    def checkpoint(self, step_id: str, when: str) -> bool:
        """Block at a step boundary while paused; ``False`` means stop."""
        with self._cond:
            if (step_id, when) in self._breakpoints:
                self.paused = True
            waited = self.paused and not self.stopped
            if waited and self.on_wait is not None:
                self.on_wait(step_id, when, True)
            while self.paused and not self.stopped:
                self.waiting_at = (step_id, when)
                self._cond.wait()
            self.waiting_at = None
            if waited and self.on_wait is not None:
                self.on_wait(step_id, when, False)
            return not self.stopped
