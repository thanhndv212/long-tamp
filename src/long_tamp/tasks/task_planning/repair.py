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

Some failures are worth retrying as they are: the step wasn't wrong, the
world changed under it (a grasp missed: the object wasn't where it was
seen). ``retry(failure)`` says so (``retry_missed_grasp``); the loop then
replans without blocking anything, at most ``max_retries`` times per step
(each retry is a round too). Whoever executes recovers first: opens the
hand, observes the world again (#128).

It stops on success, after ``max_rounds`` plans, when the policy adds
nothing new (replanning would repeat the same plan), when a step's retries
are spent, or when no plan avoids the blocked bindings (``NoPlanFound``).
Replanning starts from wherever the failure left the world: completed steps
are not redone, because the planner plans from the world state.
``RepairOutcome.failures`` is every failure, oldest first: the fact history
a supervisor gets when the loop gives up.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from .predicates import parse_atom
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
    #: Whether the failed step was retried as it was (``retry``).
    retried: bool = False


@dataclass
class RepairOutcome:
    success: bool
    rounds: list[RepairRound] = field(default_factory=list)
    blocked: list[Binding] = field(default_factory=list)
    message: str = ""

    @property
    def failures(self) -> list[Failure]:
        """Every round's failure, oldest first."""
        return [r.failure for r in self.rounds if r.failure is not None]


def block_failed_step(failure: Failure) -> list[Binding]:
    """The default policy: block the failed step's own binding."""
    return [(failure["capability"], dict(failure.get("parameters", {})))]


def retry_missed_grasp(failure: Failure) -> bool:
    """Retry a step that failed with ``grasp_missed`` (ADR-0009): the object
    wasn't where it was seen, so pick it again where it is."""
    return any(
        parse_atom(fact).name == "grasp_missed" for fact in failure.get("facts", [])
    )


def plan_execute_repair(
    plan: Callable[[list[Binding]], dict[str, Any]],
    execute: Callable[[dict[str, Any]], Failure | None],
    policy: Callable[[Failure], list[Binding]] = block_failed_step,
    max_rounds: int = 3,
    on_replan: Callable[[Failure, list[Binding]], None] | None = None,
    retry: Callable[[Failure], bool] | None = None,
    max_retries: int = 2,
) -> RepairOutcome:
    """Run the repair loop; see the module docstring."""
    outcome = RepairOutcome(success=False)
    retries: dict[Any, int] = {}
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
        if retry is not None and retry(failure):
            step = failure.get("step")
            retries[step] = retries.get(step, 0) + 1
            if retries[step] > max_retries:
                outcome.message = (
                    f"{step}: still failing after {max_retries} retries "
                    f"({', '.join(failure.get('facts', []))})"
                )
                return outcome
            record.retried = True
            if on_replan is not None:
                on_replan(failure, [])
            continue
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
    "retry_missed_grasp",
]
