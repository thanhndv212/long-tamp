"""Mission event stream: one JSON object per status transition (JSONL).

The Python executor (``run_plan(..., on_event=...)``, ``PlanExecutor``) and the
BehaviorTree.CPP host (``agimus_taskplan_bt --events <path>``) write the same
schema, so a mission can be traced, replayed or visualized the same way
whichever runs it. Schema: ``docs/usage/events.md``.

Every event names the TaskPlan IR node it belongs to (``ir_id``) and its
``role`` within that node. A transaction compiles to several BT elements (see
``compiler.py``), each with its own role; other IR nodes have one element, its
role being the node's type. The compiler stamps ``_ir_id`` and ``_ir_role`` on
every BT element it emits for an IR node, which is how the C++ host maps BT
nodes back to the plan.

Transitions follow BehaviorTree.CPP: composites go ``RUNNING`` then
``SUCCESS``/``FAILURE``; leaves go straight to their result; transitions back to
``IDLE`` are not events. ``previous`` is the status before the transition.
"""

from __future__ import annotations

import json
import time
from collections.abc import Callable
from pathlib import Path
from typing import IO, Any

SCHEMA = "long-tamp.events/1"

# Roles of the BT elements a transaction compiles to, and the element names.
TRANSACTION_ROLES = {
    "transaction": "{label} transaction",  # Fallback
    "complete": "{label} complete",  # TaskStepComplete
    "ready": "{label} ready",  # Sequence
    "precondition": "{label} precondition",  # TaskStepReady
    "attempts": "{label} retry",  # RetryUntilSuccessful
    "execute": "{label}",  # ExecuteTaskStep
}
#: Python executor only: one command executed on an execution backend.
MOTION_ROLE = "motion"

STATUSES = ("RUNNING", "SUCCESS", "FAILURE", "SKIPPED")

EventSink = Callable[[dict[str, Any]], None]


def element_name(label: str, role: str) -> str:
    """The BT element name for ``role`` of a transaction labelled ``label``."""
    return TRANSACTION_ROLES[role].format(label=label)


def make_event(
    ir_id: str,
    role: str,
    name: str,
    status: str,
    previous: str = "IDLE",
    *,
    source: str = "python",
    t: float | None = None,
    message: str | None = None,
    metrics: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """An event dict in the stream's schema (optional fields left out)."""
    event: dict[str, Any] = {
        "schema": SCHEMA,
        "t": time.time() if t is None else t,
        "source": source,
        "ir_id": ir_id,
        "role": role,
        "name": name,
        "status": status,
        "previous": previous,
    }
    if message:
        event["message"] = message
    if metrics:
        event["metrics"] = metrics
    return event


class JsonlEventWriter:
    """An event sink writing one line per event, flushed immediately, so a
    killed mission keeps every event up to the kill."""

    def __init__(self, target: str | Path | IO[str]) -> None:
        if isinstance(target, (str, Path)):
            self._file: IO[str] = open(target, "a", encoding="utf-8")
            self._owned = True
        else:
            self._file, self._owned = target, False

    def __call__(self, event: dict[str, Any]) -> None:
        self._file.write(json.dumps(event, sort_keys=True) + "\n")
        self._file.flush()

    def close(self) -> None:
        if self._owned:
            self._file.close()

    def __enter__(self) -> JsonlEventWriter:
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()


def read_events(path: str | Path) -> list[dict[str, Any]]:
    """The events in a JSONL file (blank lines ignored)."""
    with open(path, encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


__all__ = [
    "MOTION_ROLE",
    "SCHEMA",
    "STATUSES",
    "TRANSACTION_ROLES",
    "EventSink",
    "JsonlEventWriter",
    "element_name",
    "make_event",
    "read_events",
]
