"""Pause, resume, stop and breakpoints at step boundaries (ADR-0004).

The executor calls ``checkpoint(step_id, "before")`` before starting a step and
``checkpoint(step_id, "after")`` once it is done. A checkpoint blocks while the
execution is paused (by ``pause()`` or by a breakpoint on that boundary) and
returns ``False`` if the execution was stopped, in which case the executor
does not start the step. Commands are never interrupted by a pause: they
finish, and the pause takes effect at the next boundary. ``request("skip")``
and ``request("abort_step")`` act *inside* a step that is planning, at its
next ``activity.checkpoint`` (#108). ``stop()`` also
cancels a running command (see ``run_command``). Thread-safe: control calls
come from another thread (a UI, a ROS service) than the executor's.
"""

from __future__ import annotations

import threading
from collections.abc import Callable

_WHEN = ("before", "after")
_REQUESTS = ("skip", "abort_step")


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
        #: A pending in-step request: (action, reason), see ``request``.
        self.pending: tuple[str, str] | None = None

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

    def reset(self) -> None:
        """Clear a pause, a stop and a pending request, for the next run
        (breakpoints stay)."""
        with self._cond:
            self.paused = False
            self.stopped = False
            self.pending = None
            self._cond.notify_all()

    def request(self, action: str, reason: str = "operator") -> None:
        """Ask the step being planned to ``skip`` its current search (it goes
        on without it) or to ``abort_step`` (it fails now, without retry).
        Taken at the step's next checkpoint; a later request replaces it."""
        if action not in _REQUESTS:
            raise ValueError(f"action must be one of {_REQUESTS}, got {action!r}")
        with self._cond:
            self.pending = (action, reason)

    def take_request(self, scope: str) -> tuple[str, str] | None:
        """The pending request a checkpoint in ``scope`` acts on, cleared:
        ``abort_step`` anywhere, ``skip`` only in a search."""
        with self._cond:
            if self.pending is None:
                return None
            if self.pending[0] == "skip" and scope != "search":
                return None
            request, self.pending = self.pending, None
            return request

    def clear_request(self) -> None:
        """Drop a request no checkpoint took (the step ended first)."""
        with self._cond:
            self.pending = None

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
