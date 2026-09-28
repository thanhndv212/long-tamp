"""Plan, execute, and replan around refinement failures (ADR-0001, issue #15).

``plan_execute_repair`` runs the task-level repair loop:

1. ``plan(blocked)`` plans the goal from the *current* world state, avoiding
   every blocked binding (``pddl.to_pddl(..., blocked=...)``), and returns a
   TaskPlan document;
2. ``execute(document)`` runs it; it returns ``None`` on success, or the
   failure: the step, its capability and parameters, and the facts the
   refiner reported (``refiner.failure_facts``);
3. ``policy(failure)`` turns the failure into bindings to block
   (``[(capability, {parameter: value})]``); the loop replans with them.

It stops on success, after ``max_rounds`` plans, when the policy adds
nothing new (replanning would repeat the same plan), or when no plan avoids
the blocked bindings (``NoPlanFound``). Replanning starts from
wherever the failure left the world: completed steps are not redone,
because the planner plans from the world state.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from .skeleton import NoPlanFound

Binding = tuple[str, dict[str, str]]
Failure = dict[str, Any]


@dataclass
class RepairRound:
    """One plan-and-execute round."""

    document: dict[str, Any]
    failure: Failure | None = None
    #: Bindings the policy blocked after this round's failure.
    blocked: list[Binding] = field(default_factory=list)


@dataclass
class RepairOutcome:
    success: bool
    rounds: list[RepairRound] = field(default_factory=list)
    blocked: list[Binding] = field(default_factory=list)
    message: str = ""


def block_failed_step(failure: Failure) -> list[Binding]:
    """The default policy: block the failed step's own binding."""
    return [(failure["capability"], dict(failure.get("parameters", {})))]


def plan_execute_repair(
    plan: Callable[[list[Binding]], dict[str, Any]],
    execute: Callable[[dict[str, Any]], Failure | None],
    policy: Callable[[Failure], list[Binding]] = block_failed_step,
    max_rounds: int = 3,
    on_replan: Callable[[Failure, list[Binding]], None] | None = None,
) -> RepairOutcome:
    """Run the repair loop; see the module docstring."""
    outcome = RepairOutcome(success=False)
    for _ in range(max_rounds):
        try:
            document = plan(list(outcome.blocked))
        except NoPlanFound as error:
            outcome.message = f"no plan avoids {outcome.blocked}: {error}"
            return outcome
        record = RepairRound(document)
        outcome.rounds.append(record)
        failure = execute(document)
        if failure is None:
            outcome.success = True
            return outcome
        record.failure = failure
        new = [b for b in policy(failure) if not _contains(outcome.blocked, b)]
        if not new:
            outcome.message = (
                f"{failure.get('step')}: failed, and the policy has nothing new "
                "to block"
            )
            return outcome
        record.blocked = new
        outcome.blocked += new
        if on_replan is not None:
            on_replan(failure, new)
    outcome.message = f"still failing after {max_rounds} plans"
    return outcome


def _contains(bindings: list[Binding], binding: Binding) -> bool:
    capability, values = binding
    return any(c == capability and dict(v) == dict(values) for c, v in bindings)


__all__ = [
    "Binding",
    "Failure",
    "RepairOutcome",
    "RepairRound",
    "block_failed_step",
    "plan_execute_repair",
]
