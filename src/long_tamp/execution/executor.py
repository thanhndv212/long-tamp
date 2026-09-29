"""The Python TaskPlan executor: plan each step, then execute its motion.

``PlanExecutor(session, backend).run()`` walks the plan with ``run_plan`` (the
compiled BehaviorTree's semantics). For every step that runs:

1. ``control.checkpoint(step, "before")``: pause, breakpoints, stop;
2. the step's capability plans it (``execute_step``) and hands the motion to
   execute to ``executor.submit(command)``; commands from a failed attempt
   are discarded, only the successful attempt's run;
3. each submitted command runs on ``backend`` through ``run_command`` (BUSY
   retries, heartbeats, duration-scaled deadline); a failed execution fails
   the step, and the plan stops there;
4. ``control.checkpoint(step, "after")``.

``on_event`` receives the mission's event stream (``task_planning.events``):
the plan's transitions, plus a ``motion`` event pair (``RUNNING``, then the
result with its metrics) for every command run on the backend.

Without a backend, steps are planned only (submitted commands are dropped),
which is how planning-only runs and batch validation use it.

Motion is passed out of band (``submit``) rather than through the session's
JSON responses, which exist for the C++ host and can't carry path objects.
Execution happens after planning, so the world state (e.g. the grasp tracker)
already reflects a step when its motion runs; a failed execution stops the
mission rather than rolling that back.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from long_tamp.tasks.task_planning.events import MOTION_ROLE, EventSink, make_event
from long_tamp.tasks.task_planning.runner import PlanRun, run_plan

from .contract import (
    ExecutionBackend,
    ExecutionCommand,
    ExecutionPolicy,
    ExecutionResult,
    ExecutionStatus,
)
from .control import ExecutionControl
from .supervisor import run_command


@dataclass
class StepExecution:
    """One command executed for a step, and how it ended."""

    step_id: str
    command: ExecutionCommand
    result: ExecutionResult


class _AttemptSession:
    """The session, with a hook at the start of every execution attempt."""

    def __init__(self, session: Any, on_attempt: Callable[[], None]) -> None:
        self._session = session
        self._on_attempt = on_attempt

    def execute_step(self, step_id: str) -> str:
        self._on_attempt()
        return self._session.execute_step(step_id)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._session, name)


class PlanExecutor:
    def __init__(
        self,
        session: Any,
        backend: ExecutionBackend | None = None,
        policy: ExecutionPolicy | None = None,
        control: ExecutionControl | None = None,
        on_skip: Callable[[str, str], None] | None = None,
        on_event: EventSink | None = None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.session = session
        self.backend = backend
        self.policy = policy or ExecutionPolicy()
        self.control = control or ExecutionControl()
        self.on_skip = on_skip
        self.on_event = on_event
        self.clock, self.sleep = clock, sleep
        self._pending: list[ExecutionCommand] = []
        self._executions: list[StepExecution] = []

    def submit(self, command: ExecutionCommand) -> None:
        """Queue motion for the step being planned; called by capabilities."""
        self._pending.append(command)

    def run(self) -> PlanRun:
        self._executions = []
        run = run_plan(
            _AttemptSession(self.session, self._pending.clear),
            on_skip=self.on_skip,
            before_step=self._before,
            after_step=self._after,
            on_event=self.on_event,
        )
        run.executions = list(self._executions)
        return run

    def _before(self, node: dict[str, Any]) -> bool:
        self._pending.clear()
        return self.control.checkpoint(node["id"], "before")

    def _after(self, node: dict[str, Any], result: dict[str, Any]) -> str | None:
        commands, self._pending = self._pending, []
        if self.backend is not None:
            for command in commands:
                self._emit(node, command, "RUNNING")
                outcome = run_command(
                    self.backend,
                    command,
                    self.policy,
                    control=self.control,
                    clock=self.clock,
                    sleep=self.sleep,
                )
                self._executions.append(StepExecution(node["id"], command, outcome))
                self._emit(node, command, outcome.status.name, "RUNNING", outcome)
                if outcome.status is not ExecutionStatus.SUCCESS:
                    return f"execution failed ({outcome.reason}): {outcome.message}"
        self.control.checkpoint(node["id"], "after")
        return None

    def _emit(
        self,
        node: dict[str, Any],
        command: ExecutionCommand,
        status: str,
        previous: str = "IDLE",
        outcome: ExecutionResult | None = None,
    ) -> None:
        if self.on_event is None:
            return
        metrics = None
        if outcome is not None:
            metrics = {
                "seconds": round(outcome.elapsed, 3),
                "feedback_count": outcome.feedback_count,
                "busy_retries": outcome.busy_retries,
            }
            if outcome.reason:
                metrics["reason"] = outcome.reason
            if command.duration is not None:
                metrics["duration"] = round(command.duration, 3)
            for key, value in outcome.metrics.items():
                metrics.setdefault(key, value)
        self.on_event(
            make_event(
                node["id"],
                MOTION_ROLE,
                command.step_id,
                status,
                previous,
                message=outcome.message if outcome is not None else "",
                metrics=metrics,
            )
        )
