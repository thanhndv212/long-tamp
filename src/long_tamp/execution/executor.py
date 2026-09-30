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
mission rather than rolling that back. Recorded facts are the exception: with
a backend, a step's recorded effects (a screw driven) are written only once
its motion has executed.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from long_tamp.tasks.task_planning.events import (
    DRIFT_ROLE,
    MOTION_ROLE,
    EventSink,
    make_event,
)
from long_tamp.tasks.task_planning.runner import PlanRun, run_plan

from .contract import (
    ExecutionBackend,
    ExecutionCommand,
    ExecutionPolicy,
    ExecutionResult,
    ExecutionStatus,
)
from .concurrent import merge_lanes
from .control import ExecutionControl
from .sampled import sampled
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
    """Runs a TaskPlan: each step is planned, then its motion executes.

    - ``on_drift(node, observed)``: before a step's motion runs, the executor
      asks the backend how far the robot is from where the plan starts
      (``backend.start_error(command)``). Beyond ``policy.max_start_drift``
      the cached plan is stale: ``on_drift`` replans the step from
      ``observed`` (``backend.observed_config``) and returns its new
      commands; without it, the step fails with reason ``drift``.
    - ``plan_ahead``: plan step k+1 while step k's motion executes in a worker
      thread. At the handoff the executor waits for step k's motion, commits
      its recorded effects, checks drift, and only then starts step k+1's
      motion. Paths are sampled (``SampledPath``) in the planning thread, so
      the worker never evaluates a planner path while the planner plans.
    - ``concurrent``: a ``parallel`` group's steps are planned first, then
      their lanes' motions run together, merged (``concurrent.merge_lanes``;
      ``validate_config`` checks the merged motion for collisions). If they
      can't be merged, the steps' motions run one after another, in planning
      order. Either way the group's steps commit once all of it has run.
    """

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
        on_drift: Callable[[dict[str, Any], Any], list[ExecutionCommand]] | None = None,
        plan_ahead: bool = False,
        concurrent: bool = False,
        validate_config: Callable[[Any], bool] | None = None,
    ) -> None:
        self.session = session
        self.backend = backend
        self.policy = policy or ExecutionPolicy()
        self.control = control or ExecutionControl()
        self.on_skip = on_skip
        self._lock = threading.Lock()
        self.on_event = self._locked(on_event) if on_event is not None else None
        self.clock, self.sleep = clock, sleep
        self.on_drift = on_drift
        self.plan_ahead = plan_ahead
        self.concurrent = concurrent
        self.validate_config = validate_config
        self._pending: list[ExecutionCommand] = []
        self._executions: list[StepExecution] = []
        self._worker: threading.Thread | None = None
        #: The motion in flight: (the steps it commits, the node its events
        #: go to, its error once done).
        self._in_flight: tuple[list[dict[str, Any]], dict[str, Any], str | None] | None
        self._in_flight = None
        #: The parallel group being planned: its node, each step's lane, and
        #: the planned steps' commands.
        self._group: dict[str, Any] | None = None
        self._timing: dict[str, float] = {}

    def _locked(self, sink: EventSink) -> EventSink:
        def emit(event: dict[str, Any]) -> None:
            with self._lock:
                sink(event)

        return emit

    def submit(self, command: ExecutionCommand) -> None:
        """Queue motion for the step being planned; called by capabilities."""
        self._pending.append(command)

    def run(self) -> PlanRun:
        self._executions = []
        self._timing = {
            "execution": 0.0,
            "idle_between_motions": 0.0,
            "waited_for_motion": 0.0,
            "drift_replans": 0,
            "merged_groups": 0,
            "sequential_groups": 0,
        }
        self._last_motion_end: float | None = None
        # Recorded facts (a screw driven) hold once the motion has executed,
        # not when it was planned.
        if self.backend is not None and hasattr(self.session, "commit_effects"):
            self.session.defer_recording = True
        t0 = self.clock()
        run = run_plan(
            _AttemptSession(self.session, self._pending.clear),
            on_skip=self.on_skip,
            before_step=self._before,
            after_step=self._after,
            on_event=self.on_event,
            on_group=self._on_group if self.concurrent else None,
        )
        # The last step's motion may still be running.
        failed = self._join()
        if failed is not None and run.success:
            run.success = False
            run.failed_step, run.message = failed
        elif failed is not None:
            run.failed_step, run.message = failed
        self._timing["wall"] = self.clock() - t0
        run.executions = list(self._executions)
        run.timing = {k: round(v, 3) for k, v in self._timing.items()}
        return run

    def _before(self, node: dict[str, Any]) -> bool:
        self._pending.clear()
        # A failed motion in flight stops the plan at the next boundary.
        if self._worker is not None and not self._worker.is_alive():
            if self._in_flight is not None and self._in_flight[2] is not None:
                return False
        return self.control.checkpoint(node["id"], "before")

    def _after(self, node: dict[str, Any], result: dict[str, Any]) -> str | None:
        commands, self._pending = self._pending, []
        if self.backend is None:
            self.control.checkpoint(node["id"], "after")
            return None
        if self._group is not None and node["id"] in self._group["lane_of"]:
            # Runs with the rest of the group, when it is planned.
            self._group["planned"].append((node, list(commands)))
            return None
        return self._run_motion([node], node, commands)

    def _run_motion(
        self,
        nodes: list[dict[str, Any]],
        report: dict[str, Any],
        commands: list[ExecutionCommand],
    ) -> str | None:
        """Execute ``commands`` (events under ``report``), then commit
        ``nodes``: now, or in the worker thread with plan-ahead."""
        node = report
        if self.plan_ahead:
            commands = [
                ExecutionCommand(c.step_id, c.duration, sampled(c.payload))
                for c in commands
            ]
            failed = self._join()  # the previous step's motion
            if failed is not None:
                return f"{failed[0]}: {failed[1]}"
        error, commands = self._check_drift(node, commands)
        if error:
            return error
        if self.plan_ahead:
            self._in_flight = (nodes, node, None)
            self._worker = threading.Thread(
                target=self._execute_async, args=(nodes, node, commands), daemon=True
            )
            self._worker.start()
            return None
        error = self._execute(node, commands)
        if error:
            return error
        for done in nodes:
            self._commit(done)
            self.control.checkpoint(done["id"], "after")
        return None

    # -- parallel groups ---------------------------------------------------

    def _on_group(self, node: dict[str, Any], ok: bool | None) -> str | None:
        from long_tamp.tasks.task_planning.partial_order import lane_steps

        if self.backend is None:
            return None
        if ok is None:
            lane_of = {
                step["id"]: k
                for k, lane in enumerate(node["children"])
                for step in lane_steps(lane)
            }
            self._group = {"node": node, "lane_of": lane_of, "planned": []}
            return None
        group, self._group = self._group, None
        if group is None or not group["planned"]:
            return None
        planned = group["planned"]
        nodes = [step for step, _ in planned]
        merged = None
        if ok:
            lanes: dict[int, list[ExecutionCommand]] = {}
            for step, commands in planned:
                lanes.setdefault(group["lane_of"][step["id"]], []).extend(commands)
            merged = merge_lanes(
                [lanes[k] for k in sorted(lanes)],
                self.validate_config,
                step_id=node.get("label", node["id"]),
            )
        if merged is not None:
            self._timing["merged_groups"] += 1
            return self._run_motion(nodes, node, merged)
        # One after another, in planning order (also what a failed group
        # planned before it failed: the world state already has it).
        self._timing["sequential_groups"] += ok is not False
        commands = [c for _, step_commands in planned for c in step_commands]
        return self._run_motion(nodes, node, commands)

    # -- drift -------------------------------------------------------------

    def _check_drift(
        self, node: dict[str, Any], commands: list[ExecutionCommand]
    ) -> tuple[str, list[ExecutionCommand]]:
        tolerance = self.policy.max_start_drift
        start_error = getattr(self.backend, "start_error", None)
        if tolerance is None or not commands or start_error is None:
            return "", commands
        drift = start_error(commands[0])
        if drift is None or drift <= tolerance:
            return "", commands
        self._emit_drift(node, drift, tolerance)
        # Replanning is per step; a parallel group's merged motion can't be.
        if self.on_drift is None or node.get("type") == "parallel":
            return (
                f"execution failed (drift): the robot is {drift:.3f} rad from "
                f"where the plan starts (tolerance {tolerance:g})"
            ), []
        observe = getattr(self.backend, "observed_config", None)
        self._timing["drift_replans"] += 1
        try:
            replanned = self.on_drift(node, observe)
        except Exception as error:  # noqa: BLE001 - replanning boundary
            return f"execution failed (drift): replanning failed: {error}", []
        if self.plan_ahead:
            replanned = [
                ExecutionCommand(c.step_id, c.duration, sampled(c.payload))
                for c in replanned
            ]
        return "", list(replanned)

    def _emit_drift(self, node: dict[str, Any], drift: float, tolerance: float) -> None:
        if self.on_event is None:
            return
        self.on_event(
            make_event(
                node["id"],
                DRIFT_ROLE,
                node.get("label", node["id"]),
                "FAILURE",
                message="the robot drifted from where the plan starts: replanning",
                metrics={"start_drift": round(drift, 5), "tolerance": tolerance},
            )
        )

    # -- executing ---------------------------------------------------------

    def _execute(self, node: dict[str, Any], commands: list[ExecutionCommand]) -> str:
        for command in commands:
            started = self.clock()
            if self._last_motion_end is not None:
                self._timing["idle_between_motions"] += started - self._last_motion_end
            self._emit(node, command, "RUNNING")
            outcome = run_command(
                self.backend,
                command,
                self.policy,
                control=self.control,
                clock=self.clock,
                sleep=self.sleep,
            )
            self._last_motion_end = self.clock()
            self._timing["execution"] += self._last_motion_end - started
            self._executions.append(StepExecution(node["id"], command, outcome))
            self._emit(node, command, outcome.status.name, "RUNNING", outcome)
            if outcome.status is not ExecutionStatus.SUCCESS:
                return f"execution failed ({outcome.reason}): {outcome.message}"
        return ""

    def _execute_async(
        self,
        nodes: list[dict[str, Any]],
        node: dict[str, Any],
        commands: list[ExecutionCommand],
    ) -> None:
        self._in_flight = (nodes, node, self._execute(node, commands) or None)

    def _join(self) -> tuple[str, str] | None:
        """Wait for the motion in flight; commit its step, or return
        (step, message) if it failed."""
        if self._worker is None:
            return None
        waited = self.clock()
        self._worker.join()
        self._timing["waited_for_motion"] += self.clock() - waited
        self._worker = None
        nodes, node, error = self._in_flight
        self._in_flight = None
        if error:
            return node["id"], error
        for done in nodes:
            self._commit(done)
            self.control.checkpoint(done["id"], "after")
        return None

    def _commit(self, node: dict[str, Any]) -> None:
        if getattr(self.session, "defer_recording", False):
            self.session.commit_effects(node["id"])

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
