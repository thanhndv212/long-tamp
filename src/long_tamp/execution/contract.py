"""What an execution backend promises, and what the executor promises it (ADR-0004).

A backend runs one command at a time (typically a planned trajectory) on a
robot, a simulator or a viewer. It is driven by polling, so it can be a ROS
action client, a MuJoCo loop or a stub, and needs no threads of its own:

- ``start(command)`` accepts the command (``RUNNING``), cannot take it right now
  (``BUSY``: another command is still running, the controller is not ready; the
  caller retries later, it is *not* a failure), or rejects it (``FAILURE``);
- ``poll()`` reports the status and, when there is progress, a :class:`Feedback`
  heartbeat; a backend that stops sending heartbeats is considered stuck;
- ``cancel()`` stops the running command (called on timeouts and stop requests).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Protocol


class ExecutionStatus(str, Enum):
    RUNNING = "running"
    SUCCESS = "success"
    FAILURE = "failure"
    BUSY = "busy"


@dataclass(frozen=True)
class ExecutionCommand:
    """One unit of execution: a step's motion, for example.

    ``duration`` is the command's nominal duration in seconds (a trajectory's
    time span), used to scale its deadline; ``None`` means unknown (no
    deadline, the inactivity timeout still applies). ``payload`` is whatever
    the backend executes (a trajectory, a path id, ...).
    """

    step_id: str
    duration: float | None = None
    payload: Any = None


@dataclass(frozen=True)
class Feedback:
    """A heartbeat: proof of progress, optionally with how much.

    ``metrics`` are numbers the backend measures (a simulator's tracking error,
    for example); the last ones reported end up in the command's result and
    its ``motion`` event. ``facts`` likewise.
    """

    progress: float | None = None
    message: str = ""
    metrics: dict[str, float] | None = None
    #: Ground atoms the command established (a skill's postconditions) or,
    #: on failure, why it failed (e.g. ``screw_misaligned(driver, part1/h_hole1)``).
    facts: tuple[str, ...] = ()


class ExecutionBackend(Protocol):
    def start(self, command: ExecutionCommand) -> ExecutionStatus:
        """``RUNNING`` if accepted, ``BUSY`` to be retried later, ``FAILURE`` if rejected."""

    def poll(self) -> tuple[ExecutionStatus, Feedback | None]:
        """Current status, and a heartbeat when there was progress since the last poll."""

    def cancel(self) -> None:
        """Stop the running command."""


@dataclass
class ExecutionPolicy:
    """How long to wait, and when to give up.

    - ``inactivity_timeout``: seconds without a heartbeat before the backend is
      considered stuck. Long commands that keep reporting progress never hit it.
    - ``min_real_time_factor`` / ``deadline_margin``: a command may take up to
      ``duration / min_real_time_factor + deadline_margin`` seconds. A slow
      simulator legitimately runs slower than real time; a flat wall-clock
      deadline would cancel healthy commands.
    - ``busy_retries`` / ``busy_backoff``: how often, and how far apart, to retry
      a ``BUSY`` start before failing with reason ``busy``.
    - ``poll_interval``: seconds between polls.
    """

    inactivity_timeout: float = 30.0
    min_real_time_factor: float = 0.5
    deadline_margin: float = 10.0
    busy_retries: int = 10
    busy_backoff: float = 1.0
    poll_interval: float = 0.1

    def deadline(self, command: ExecutionCommand) -> float | None:
        if command.duration is None:
            return None
        return command.duration / self.min_real_time_factor + self.deadline_margin


@dataclass
class ExecutionResult:
    """How a command ended.

    ``reason`` says why a ``FAILURE`` happened: ``"failed"`` (the backend
    reported it), ``"busy"``, ``"inactivity"``, ``"deadline"`` or ``"stopped"``.
    """

    status: ExecutionStatus
    reason: str = ""
    message: str = ""
    elapsed: float = 0.0
    feedback_count: int = 0
    busy_retries: int = 0
    #: The last metrics the backend reported (``Feedback.metrics``).
    metrics: dict[str, float] = field(default_factory=dict)
    #: The last facts the backend reported (``Feedback.facts``).
    facts: tuple[str, ...] = ()
