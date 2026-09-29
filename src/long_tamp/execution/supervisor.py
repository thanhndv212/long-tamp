"""Run one command on a backend under the execution policy (ADR-0004)."""

from __future__ import annotations

import time
from collections.abc import Callable

from .contract import (
    ExecutionBackend,
    ExecutionCommand,
    ExecutionPolicy,
    ExecutionResult,
    ExecutionStatus,
)
from .control import ExecutionControl


def run_command(
    backend: ExecutionBackend,
    command: ExecutionCommand,
    policy: ExecutionPolicy | None = None,
    control: ExecutionControl | None = None,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> ExecutionResult:
    """Start ``command``, supervise it to the end, and say how it ended.

    - ``BUSY`` starts are retried every ``busy_backoff`` seconds, up to
      ``busy_retries`` times;
    - a running command is cancelled when no heartbeat arrives for
      ``inactivity_timeout`` seconds, when it exceeds its duration-scaled
      deadline, or when ``control`` is stopped.

    ``clock`` and ``sleep`` are injectable so the timeouts can be tested
    without waiting.
    """
    policy = policy or ExecutionPolicy()
    t0 = clock()
    retries = 0

    metrics: dict[str, float] = {}

    def done(status, reason="", message="", feedback=0):
        return ExecutionResult(
            status=status,
            reason=reason,
            message=message,
            elapsed=clock() - t0,
            feedback_count=feedback,
            busy_retries=retries,
            metrics=dict(metrics),
        )

    while True:
        if control is not None and control.stopped:
            return done(ExecutionStatus.FAILURE, "stopped", "stop requested")
        status = backend.start(command)
        if status is not ExecutionStatus.BUSY:
            break
        if retries >= policy.busy_retries:
            return done(
                ExecutionStatus.FAILURE,
                "busy",
                f"backend still busy after {retries} retries",
            )
        retries += 1
        sleep(policy.busy_backoff)
    if status is ExecutionStatus.FAILURE:
        return done(ExecutionStatus.FAILURE, "failed", "backend rejected the command")

    started = last_heartbeat = clock()
    heartbeats = 0
    deadline = policy.deadline(command)
    while True:
        status, feedback = backend.poll()
        now = clock()
        if feedback is not None:
            heartbeats += 1
            last_heartbeat = now
            if feedback.metrics:
                metrics.update(feedback.metrics)
        if status is ExecutionStatus.SUCCESS:
            message = feedback.message if feedback is not None else ""
            return done(status, message=message, feedback=heartbeats)
        if status is ExecutionStatus.FAILURE:
            message = feedback.message if feedback is not None else ""
            return done(status, "failed", message, heartbeats)
        if control is not None and control.stopped:
            backend.cancel()
            return done(
                ExecutionStatus.FAILURE, "stopped", "stop requested", heartbeats
            )
        if now - last_heartbeat > policy.inactivity_timeout:
            backend.cancel()
            return done(
                ExecutionStatus.FAILURE,
                "inactivity",
                f"no heartbeat for {policy.inactivity_timeout:g} s",
                heartbeats,
            )
        if deadline is not None and now - started > deadline:
            backend.cancel()
            return done(
                ExecutionStatus.FAILURE,
                "deadline",
                f"exceeded {deadline:g} s (duration {command.duration:g} s)",
                heartbeats,
            )
        sleep(policy.poll_interval)
