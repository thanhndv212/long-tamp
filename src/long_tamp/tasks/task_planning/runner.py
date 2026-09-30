"""Run a TaskPlan through a session, the way its compiled BehaviorTree does.

``run_plan(session)`` walks the plan tree synchronously with the same
semantics as the compiled BT (``compiler.py``):

- ``sequence``: children in order, failing at the first failure;
- ``parallel``: every lane must succeed. Lanes are independent
  (``partial_order``), so they are planned one after another, in order; an
  executor may run their motions concurrently (``on_group`` marks the
  group's start and end);
- ``fallback``: children in order, succeeding at the first success;
- ``retry``: the child up to its effective attempts;
- ``condition``: the capability's verdict (``evaluate_condition``);
- ``transaction``: complete (its effect holds in the world, or it completed
  in this run) -> skipped; preconditions not ready -> failure; otherwise
  ``execute_step``, retried up to the transaction's effective attempts.

It is the reference semantics for missions run from Python; the full
executor (statuses, heartbeats, pause/resume) builds on it (roadmap M2).
"""

from __future__ import annotations

import json
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from .events import EventSink, element_name, make_event
from .session import TaskPlanningSession


@dataclass
class PlanRun:
    """What happened in a ``run_plan`` call."""

    success: bool = True
    #: Labels of skipped work: transactions whose effect held, and guard
    #: conditions that were true (skipping what they guard).
    skipped: list[str] = field(default_factory=list)
    #: The step that stopped the run, and why.
    failed_step: str | None = None
    message: str = ""
    #: Commands executed for the steps (filled by executors; see
    #: ``long_tamp.execution.executor``).
    executions: list[Any] = field(default_factory=list)
    #: Where the time went (filled by executors): wall, planning and
    #: execution seconds, idle time between motions, drift replans.
    timing: dict[str, float] = field(default_factory=dict)


def run_plan(
    session: TaskPlanningSession,
    on_skip: Callable[[str, str], None] | None = None,
    before_step: Callable[[dict[str, Any]], bool] | None = None,
    after_step: Callable[[dict[str, Any], dict[str, Any]], str | None] | None = None,
    on_event: EventSink | None = None,
    on_group: Callable[[dict[str, Any], bool | None], str | None] | None = None,
) -> PlanRun:
    """Run ``session``'s plan from its root; see the module docstring.

    Hooks for executors (all optional):

    - ``before_step(node)`` runs when a step is ready, before it executes;
      returning ``False`` stops the run there (message ``"stopped"``);
    - ``after_step(node, result)`` runs after a step executed successfully
      (``result`` is ``execute_step``'s response); returning an error message
      fails the step with it;
    - ``on_event(event)`` receives one event per status transition, in the
      schema the BehaviorTree.CPP host writes (``events.py``);
    - ``on_group(node, None)`` runs when a ``parallel`` node starts, and
      ``on_group(node, ok)`` when its lanes are done; returning an error
      message from the second call fails the node with it.
    """
    run = PlanRun()
    hooks = _Hooks(on_skip, before_step, after_step, on_event, on_group)
    run.success = _run(session, session.plan.document["root"], run, hooks)
    return run


@dataclass
class _Hooks:
    on_skip: Callable[[str, str], None] | None = None
    before_step: Callable[[dict[str, Any]], bool] | None = None
    after_step: Callable[[dict[str, Any], dict[str, Any]], str | None] | None = None
    on_event: EventSink | None = None
    on_group: Callable[[dict[str, Any], bool | None], str | None] | None = None

    def emit(
        self,
        node: dict[str, Any],
        role: str,
        status: str,
        previous: str = "IDLE",
        message: str = "",
        metrics: dict[str, Any] | None = None,
    ) -> None:
        if self.on_event is None:
            return
        label = _label(node)
        name = element_name(label, role) if role in _TX_ROLES else label
        self.on_event(
            make_event(
                node["id"],
                role,
                name,
                status,
                previous,
                message=message,
                metrics=metrics,
            )
        )


_TX_ROLES = {"transaction", "complete", "ready", "precondition", "attempts", "execute"}


def _result(ok: bool) -> str:
    return "SUCCESS" if ok else "FAILURE"


def _label(node: dict[str, Any]) -> str:
    return node.get("label", node["id"])


def _run(
    session: TaskPlanningSession,
    node: dict[str, Any],
    run: PlanRun,
    hooks: _Hooks,
) -> bool:
    kind = node["type"]
    if kind == "parallel":
        return _run_parallel(session, node, run, hooks)
    if kind in ("sequence", "fallback", "retry"):
        hooks.emit(node, kind, "RUNNING")
        if kind == "sequence":
            ok = all(_run(session, child, run, hooks) for child in node["children"])
        elif kind == "fallback":
            ok = any(_run(session, child, run, hooks) for child in node["children"])
        else:
            attempts = session.plan.effective_attempts.get(node["id"], 1)
            ok = any(_run(session, node["child"], run, hooks) for _ in range(attempts))
        hooks.emit(node, kind, _result(ok), "RUNNING")
        return ok
    if kind == "condition":
        value = json.loads(session.evaluate_condition(node["id"])).get("value", False)
        hooks.emit(node, "condition", _result(bool(value)))
        if value:
            run.skipped.append(_label(node))
            if hooks.on_skip is not None:
                hooks.on_skip(_label(node), "condition holds")
        return bool(value)
    if kind == "transaction":
        return _run_transaction(session, node, run, hooks)
    # A bare operation: one ExecuteTaskStep in the compiled tree.
    ok = _run_step(
        session,
        node,
        run,
        _Hooks(
            hooks.on_skip, hooks.before_step, hooks.after_step, None, hooks.on_group
        ),
    )
    hooks.emit(node, "operation", _result(ok), message="" if ok else run.message)
    return ok


def _run_parallel(
    session: TaskPlanningSession,
    node: dict[str, Any],
    run: PlanRun,
    hooks: _Hooks,
) -> bool:
    hooks.emit(node, "parallel", "RUNNING")
    if hooks.on_group is not None:
        hooks.on_group(node, None)
    ok = all(_run(session, child, run, hooks) for child in node["children"])
    if hooks.on_group is not None:
        error = hooks.on_group(node, ok)
        if error and ok:
            ok = False
            run.failed_step = node["id"]
            run.message = error
    hooks.emit(
        node, "parallel", _result(ok), "RUNNING", message="" if ok else run.message
    )
    return ok


def _run_transaction(
    session: TaskPlanningSession,
    node: dict[str, Any],
    run: PlanRun,
    hooks: _Hooks,
) -> bool:
    hooks.emit(node, "transaction", "RUNNING")
    ok = _run_step(session, node, run, hooks)
    hooks.emit(node, "transaction", _result(ok), "RUNNING")
    return ok


def _run_step(
    session: TaskPlanningSession,
    node: dict[str, Any],
    run: PlanRun,
    hooks: _Hooks,
) -> bool:
    """A transaction's checks and attempts (events: its inner BT elements)."""
    done = json.loads(session.is_step_complete(node["id"]))
    if done.get("complete"):
        hooks.emit(node, "complete", "SUCCESS", message=done.get("reason", ""))
        run.skipped.append(_label(node))
        if hooks.on_skip is not None:
            hooks.on_skip(_label(node), done.get("reason", "complete"))
        return True
    hooks.emit(node, "complete", "FAILURE", message=done.get("reason", ""))
    hooks.emit(node, "ready", "RUNNING")
    ok = _ready_and_execute(session, node, run, hooks)
    hooks.emit(node, "ready", _result(ok), "RUNNING")
    return ok


def _ready_and_execute(
    session: TaskPlanningSession,
    node: dict[str, Any],
    run: PlanRun,
    hooks: _Hooks,
) -> bool:
    ready = json.loads(session.check_precondition(node["id"]))
    if not ready.get("ready"):
        run.failed_step = node["id"]
        unsatisfied = ready.get("unsatisfied") or [ready.get("message", "")]
        run.message = f"not ready: {', '.join(unsatisfied)}"
        hooks.emit(node, "precondition", "FAILURE", message=run.message)
        return False
    hooks.emit(node, "precondition", "SUCCESS")
    if hooks.before_step is not None and not hooks.before_step(node):
        run.failed_step = node["id"]
        run.message = "stopped"
        return False
    hooks.emit(node, "attempts", "RUNNING")
    attempts = session.plan.effective_attempts.get(node["id"], 1)
    result: dict[str, Any] = {}
    ok = hook_failed = False
    for attempt in range(1, attempts + 1):
        t0 = time.monotonic()
        result = json.loads(session.execute_step(node["id"]))
        if result["status"] in ("success", "skipped"):
            error = (
                hooks.after_step(node, result) if hooks.after_step is not None else None
            )
            metrics = {"attempt": attempt, "seconds": round(time.monotonic() - t0, 3)}
            if error:
                run.failed_step = node["id"]
                run.message = error
                hook_failed = True
                hooks.emit(node, "execute", "FAILURE", message=error, metrics=metrics)
                break
            status = "SKIPPED" if result["status"] == "skipped" else "SUCCESS"
            hooks.emit(node, "execute", status, metrics=metrics)
            ok = True
            break
        hooks.emit(
            node,
            "execute",
            "FAILURE",
            message=result.get("message", ""),
            metrics={"attempt": attempt, "seconds": round(time.monotonic() - t0, 3)},
        )
        if result["status"] != "retry":
            break
    if not ok and not hook_failed:
        run.failed_step = node["id"]
        run.message = result.get("message", "")
    hooks.emit(node, "attempts", _result(ok), "RUNNING")
    return ok
