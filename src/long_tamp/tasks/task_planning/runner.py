"""Run a TaskPlan through a session, the way its compiled BehaviorTree does.

``run_plan(session)`` walks the plan tree synchronously with the same
semantics as the compiled BT (``compiler.py``):

- ``sequence``: children in order, failing at the first failure;
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
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

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


def run_plan(
    session: TaskPlanningSession,
    on_skip: Callable[[str, str], None] | None = None,
    before_step: Callable[[dict[str, Any]], bool] | None = None,
    after_step: Callable[[dict[str, Any], dict[str, Any]], str | None] | None = None,
) -> PlanRun:
    """Run ``session``'s plan from its root; see the module docstring.

    Hooks for executors (all optional):

    - ``before_step(node)`` runs when a step is ready, before it executes;
      returning ``False`` stops the run there (message ``"stopped"``);
    - ``after_step(node, result)`` runs after a step executed successfully
      (``result`` is ``execute_step``'s response); returning an error message
      fails the step with it.
    """
    run = PlanRun()
    hooks = _Hooks(on_skip, before_step, after_step)
    run.success = _run(session, session.plan.document["root"], run, hooks)
    return run


@dataclass
class _Hooks:
    on_skip: Callable[[str, str], None] | None = None
    before_step: Callable[[dict[str, Any]], bool] | None = None
    after_step: Callable[[dict[str, Any], dict[str, Any]], str | None] | None = None


def _label(node: dict[str, Any]) -> str:
    return node.get("label", node["id"])


def _run(
    session: TaskPlanningSession,
    node: dict[str, Any],
    run: PlanRun,
    hooks: _Hooks,
) -> bool:
    kind = node["type"]
    if kind == "sequence":
        return all(_run(session, child, run, hooks) for child in node["children"])
    if kind == "fallback":
        return any(_run(session, child, run, hooks) for child in node["children"])
    if kind == "retry":
        attempts = session.plan.effective_attempts.get(node["id"], 1)
        return any(_run(session, node["child"], run, hooks) for _ in range(attempts))
    if kind == "condition":
        value = json.loads(session.evaluate_condition(node["id"])).get("value", False)
        if value:
            run.skipped.append(_label(node))
            if hooks.on_skip is not None:
                hooks.on_skip(_label(node), "condition holds")
        return bool(value)

    done = json.loads(session.is_step_complete(node["id"]))
    if done.get("complete"):
        run.skipped.append(_label(node))
        if hooks.on_skip is not None:
            hooks.on_skip(_label(node), done.get("reason", "complete"))
        return True
    ready = json.loads(session.check_precondition(node["id"]))
    if not ready.get("ready"):
        run.failed_step = node["id"]
        unsatisfied = ready.get("unsatisfied") or [ready.get("message", "")]
        run.message = f"not ready: {', '.join(unsatisfied)}"
        return False
    if hooks.before_step is not None and not hooks.before_step(node):
        run.failed_step = node["id"]
        run.message = "stopped"
        return False
    attempts = session.plan.effective_attempts.get(node["id"], 1)
    result: dict[str, Any] = {}
    for _ in range(attempts):
        result = json.loads(session.execute_step(node["id"]))
        if result["status"] in ("success", "skipped"):
            error = (
                hooks.after_step(node, result) if hooks.after_step is not None else None
            )
            if error:
                run.failed_step = node["id"]
                run.message = error
                return False
            return True
        if result["status"] != "retry":
            break
    run.failed_step = node["id"]
    run.message = result.get("message", "")
    return False
