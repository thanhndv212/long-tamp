"""What a step is doing while it plans, and stopping it early (#108).

Planning one step can take minutes: a lookahead search, phase after phase.
The executor marks the step being planned (``begin``/``end``); planning code
then calls, wherever it is useful:

- ``progress(message, **metrics)``: a ``progress`` event on the step, which the
  viewer shows as what the step is doing now;
- ``checkpoint(scope)``: a safe point to stop at. If the operator (or a
  watchdog) asked, through the ``ExecutionControl``, to ``abort_step``, it
  raises ``StepAborted``; to ``skip`` and ``scope`` is ``"search"``, it raises
  ``SearchSkipped``;
- ``searching(description)``: marks a search that ``skip`` may abandon (the
  caller catches ``SearchSkipped`` and goes on without its result).

``StepInterrupted`` derives from ``BaseException``, like ``KeyboardInterrupt``:
planning code is full of ``except Exception`` blocks that retry on any error,
and an interruption must get through them. The executor catches it at the
step boundary: an aborted step fails without retry. Code that holds state an
interruption would leave half-done catches it, cleans up and re-raises.

Without ``begin`` (planning outside an executor) every call is a no-op.
"""

from __future__ import annotations

import contextlib
import threading
import time
from collections.abc import Iterator
from typing import Any

from long_tamp.tasks.task_planning.events import EventSink, make_event

PROGRESS_ROLE = "progress"


class StepInterrupted(BaseException):
    """Planning stopped at a checkpoint on request (see the module docstring)."""

    action = ""

    def __init__(self, reason: str = "") -> None:
        super().__init__(reason)
        self.reason = reason


class StepAborted(StepInterrupted):
    action = "abort_step"


class SearchSkipped(StepInterrupted):
    action = "skip"


class _Activity:
    def __init__(self, step_id: str, label: str, sink: EventSink | None, control: Any):
        self.step_id, self.label = step_id, label
        self.sink, self.control = sink, control
        self.started = time.monotonic()
        self.searches: list[str] = []


_local = threading.local()


def _current() -> _Activity | None:
    return getattr(_local, "activity", None)


def begin(
    step_id: str, label: str = "", sink: EventSink | None = None, control: Any = None
) -> None:
    """Planning of ``step_id`` starts in this thread."""
    _local.activity = _Activity(step_id, label or step_id, sink, control)


def end() -> None:
    _local.activity = None


def current_step() -> str | None:
    activity = _current()
    return activity.step_id if activity else None


def progress(message: str, **metrics: Any) -> None:
    """Report what the step is doing now (no-op outside a step)."""
    activity = _current()
    if activity is None or activity.sink is None:
        return
    metrics["elapsed"] = round(time.monotonic() - activity.started, 1)
    if activity.searches:
        metrics["search"] = activity.searches[-1]
    activity.sink(
        make_event(
            activity.step_id,
            PROGRESS_ROLE,
            activity.label,
            "RUNNING",
            message=message,
            metrics=metrics,
        )
    )


def checkpoint(scope: str = "step") -> None:
    """Stop here if asked (see the module docstring)."""
    activity = _current()
    if activity is None or activity.control is None:
        return
    take = getattr(activity.control, "take_request", None)
    if take is None:
        return
    request = take("search" if scope == "search" and activity.searches else "step")
    if request is None:
        return
    action, reason = request
    if action == "abort_step":
        raise StepAborted(reason)
    if action == "skip":
        raise SearchSkipped(reason)


@contextlib.contextmanager
def searching(description: str) -> Iterator[None]:
    """A search ``skip`` may abandon: ``checkpoint("search")`` inside it
    raises ``SearchSkipped``, which the caller catches."""
    activity = _current()
    if activity is None:
        yield
        return
    activity.searches.append(description)
    try:
        yield
    finally:
        activity.searches.pop()


__all__ = [
    "PROGRESS_ROLE",
    "SearchSkipped",
    "StepAborted",
    "StepInterrupted",
    "begin",
    "checkpoint",
    "current_step",
    "end",
    "progress",
    "searching",
]
