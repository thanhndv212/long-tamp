"""The execution supervisor (ADR-0006): goal-level decisions, without a human.

The deterministic repair loop (``repair.plan_execute_repair``) handles what
the refiner can explain: a clamp that can't take a part is blocked, and the
planner plans around it. When it gives up (no plan avoids what's blocked,
nothing new to block, rounds spent), the mission would stop. The supervisor
decides what happens then, at goal level only:

- ``retry``: run the same goal again (a transient failure);
- ``relax_goal``: drop part of the goal and continue with what remains;
- ``abort``: stop, the goal is out of reach;
- ``escalate``: stop and hand over to a human with a report.

A model proposes the decision (``SUPERVISE_ROLE``); ``check_decision``
accepts it only if:

- the action is allowed (``Limits.allowed``);
- a relaxed goal is a non-empty, strict **subset of the original goal's
  literals** (it can drop requirements, never invent them), passes
  ``check_goal`` and is reachable by the task planner from the current state;
- a retry doesn't repeat a failure that was already retried.

Everything is bounded (``Limits``): decisions, wall-clock, and model tokens.
Past a limit, or without an acceptable decision, the supervisor escalates:
the fallback is always to stop and report, never to guess. Escalation and
abort write ``escalation.json`` and ``escalation.md`` to the run folder.
"""

from __future__ import annotations

import json
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from long_tamp.ai.roles import ModelRole, Rejected, refine

from .language import Vocabulary, check_goal

#: The decisions the supervisor can take.
ACTIONS = ("retry", "relax_goal", "abort", "escalate")


@dataclass(frozen=True)
class Limits:
    """How far the supervisor may go on its own."""

    #: Decisions (each followed by another repair loop) before escalating.
    max_decisions: int = 3
    #: Wall-clock seconds from the start, then escalate (None: no limit).
    max_seconds: float | None = None
    #: Model tokens (input + output, all roles on the client), then escalate.
    max_tokens: int | None = None
    #: Actions the model may choose (escalate is always possible).
    allowed: tuple[str, ...] = ACTIONS


@dataclass
class Decision:
    action: str
    goal: list[str] | None = None
    reason: str = ""


@dataclass
class Situation:
    """What the supervisor sees after a repair loop gave up."""

    instruction: str
    original_goal: list[str]
    goal: list[str]
    failure: dict[str, Any] | None
    message: str
    blocked: list[Any]
    vocabulary: Vocabulary
    history: list[dict[str, Any]] = field(default_factory=list)


@dataclass
class Supervised:
    """How a supervised mission ended."""

    success: bool
    goal: list[str]
    decisions: list[dict[str, Any]]
    #: Why it stopped, when it didn't succeed.
    stopped: str = ""
    report: Path | None = None
    #: The last repair loop's outcome (the caller's own result object).
    last: Any = None


# -- the decision ----------------------------------------------------------------


def check_decision(
    decision: Decision,
    situation: Situation,
    limits: Limits,
    reachable: Callable[[list[str]], str | None] | None = None,
) -> list[str]:
    """Why ``decision`` can't be taken (empty: it can)."""
    action = decision.action
    if action not in ACTIONS:
        return [f"unknown action {action!r} (one of {', '.join(ACTIONS)})"]
    if action != "escalate" and action not in limits.allowed:
        return [
            f"action {action!r} is not allowed here (allowed: {', '.join(limits.allowed)})"
        ]
    if action == "retry":
        same = [
            h
            for h in situation.history
            if h["action"] == "retry" and h.get("message") == situation.message
        ]
        if same:
            return ["this failure was already retried: retrying again won't help"]
    if action != "relax_goal":
        return []
    goal = list(decision.goal or [])
    if not goal:
        return ["relax_goal needs the remaining goal (non-empty)"]
    original = set(situation.original_goal)
    invented = [g for g in goal if g not in original]
    if invented:
        return [
            f"a relaxed goal may only keep literals of the original goal; not in it: {invented}"
        ]
    if set(goal) == set(situation.goal):
        return ["the relaxed goal must drop something from the current goal"]
    errors = check_goal(goal, situation.vocabulary)
    if not errors and reachable is not None:
        why = reachable(goal)
        if why:
            errors = [f"no plan reaches the relaxed goal from the current state: {why}"]
    return errors


_SYSTEM = """\
You supervise a robot mission that stopped. A deterministic repair loop already \
tried to plan around the failure and gave up. Decide what to do next, choosing \
exactly one action:
- "retry": run the same goal again, only if the failure looks transient;
- "relax_goal": continue with part of the goal. "goal" must list the literals to \
keep, copied exactly from the CURRENT goal; drop only what the failure makes \
impossible, and keep everything else;
- "abort": stop, the goal is out of reach;
- "escalate": stop and ask a human, when you are unsure.
Never invent goal literals or plan steps. Explain your choice in "reason"."""

_SCHEMA = {
    "type": "object",
    "properties": {
        "action": {"type": "string", "enum": list(ACTIONS)},
        "goal": {"type": "array", "items": {"type": "string"}},
        "reason": {"type": "string"},
    },
    "required": ["action", "goal", "reason"],
    "additionalProperties": False,
}


def _prompt(situation: Situation, feedback: Sequence[str]) -> str:
    lines = [f"Operator's instruction: {situation.instruction}", ""]
    lines += ["Original goal:", *[f"- {g}" for g in situation.original_goal]]
    lines += ["", "Current goal:", *[f"- {g}" for g in situation.goal]]
    lines += ["", "Current state:", *[f"- {a}" for a in situation.vocabulary.state]]
    if not situation.vocabulary.state:
        lines.append("- (nothing)")
    lines += ["", f"Why the repair loop gave up: {situation.message}"]
    if situation.failure:
        lines += [
            f"Last failed step: {situation.failure.get('step')} "
            f"({situation.failure.get('capability')} {situation.failure.get('parameters')})",
            "Facts reported: " + ", ".join(situation.failure.get("facts", [])),
        ]
    if situation.blocked:
        lines.append(f"Ruled out so far: {situation.blocked}")
    if situation.history:
        lines += ["", "Decisions so far:"]
        lines += [f"- {h['action']}: {h.get('reason', '')}" for h in situation.history]
    if situation.vocabulary.notes:
        lines += ["", "Notes on this domain:", situation.vocabulary.notes.strip()]
    if feedback:
        lines += ["", *feedback, "Decide again."]
    return "\n".join(lines)


def _parse(answer: Any) -> Decision:
    if not isinstance(answer, dict) or "action" not in answer:
        raise Rejected(
            'answer with a JSON object like {"action": "relax_goal", "goal": [...], '
            '"reason": "..."}'
        )
    goal = answer.get("goal")
    return Decision(
        action=str(answer["action"]).strip(),
        goal=[str(g) for g in goal] if isinstance(goal, list) else None,
        reason=str(answer.get("reason", "")),
    )


#: The supervisor as a model role: the request is a ``Situation``.
SUPERVISE_ROLE = ModelRole(
    name="supervise", system=_SYSTEM, schema=_SCHEMA, render=_prompt, parse=_parse
)


# -- the loop ------------------------------------------------------------------


def _tokens(client: Any) -> int:
    return sum(
        (r.input_tokens or 0) + (r.output_tokens or 0)
        for r in getattr(client, "records", [])
    )


def supervise(
    run_repair: Callable[[list[str]], Any],
    situation_of: Callable[[Any, list[str]], Situation],
    instruction: str,
    goal: list[str],
    client: Any = None,
    limits: Limits | None = None,
    reachable: Callable[[list[str]], str | None] | None = None,
    report_dir: Path | None = None,
    clock: Callable[[], float] = time.monotonic,
) -> Supervised:
    """Run repair loops under supervision (see the module docstring).

    ``run_repair(goal)`` runs one deterministic repair loop and returns its
    result, which has ``success``; ``situation_of(result, goal)`` describes a
    failed one. Without ``client`` there is no model: the supervisor escalates
    at the first failure.
    """
    limits = limits or Limits()
    t0 = clock()
    original = list(goal)
    decisions: list[dict[str, Any]] = []
    result = None
    while True:
        result = run_repair(goal)
        if getattr(result, "success", None) or (
            isinstance(result, dict) and result.get("success")
        ):
            return Supervised(True, goal, decisions, last=result)
        situation = situation_of(result, goal)
        situation.original_goal, situation.history = original, decisions
        stop = None
        if len(decisions) >= limits.max_decisions:
            stop = f"decision limit reached ({limits.max_decisions})"
        elif limits.max_seconds is not None and clock() - t0 > limits.max_seconds:
            stop = f"time limit reached ({limits.max_seconds:g} s)"
        elif limits.max_tokens is not None and _tokens(client) > limits.max_tokens:
            stop = f"model token limit reached ({limits.max_tokens})"
        elif client is None:
            stop = "no model to decide"
        if stop is not None:
            decision, fallback = Decision("escalate", reason=stop), stop
            attempts: list[dict[str, Any]] = []
        else:
            outcome = refine(
                SUPERVISE_ROLE.proposer(client, situation),
                lambda d: check_decision(d, situation, limits, reachable),
                max_rounds=2,
                fallback=lambda why: Decision(
                    "escalate", reason=f"no valid decision: {why}"
                ),
                name="supervise",
            )
            decision, fallback, attempts = (
                outcome.value,
                outcome.fallback,
                outcome.attempts,
            )
        decisions.append(
            {
                "action": decision.action,
                "goal": decision.goal,
                "reason": decision.reason,
                "message": situation.message,
                "fallback": fallback,
                "rejected": [a for a in attempts if a.get("errors") or a.get("error")],
            }
        )
        if decision.action == "retry":
            continue
        if decision.action == "relax_goal":
            goal = list(decision.goal or [])
            continue
        report = (
            write_report(report_dir, situation, decisions, decision)
            if report_dir is not None
            else None
        )
        return Supervised(
            False,
            goal,
            decisions,
            stopped=f"{decision.action}: {decision.reason}",
            report=report,
            last=result,
        )


def write_report(
    folder: Path, situation: Situation, decisions: list[dict[str, Any]], final: Decision
) -> Path:
    """``escalation.json`` and ``escalation.md`` in ``folder``: what failed,
    what was tried, the state. Returns the Markdown file's path."""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    data = {
        "stopped": final.action,
        "reason": final.reason,
        "instruction": situation.instruction,
        "original_goal": situation.original_goal,
        "goal": situation.goal,
        "failure": situation.failure,
        "repair_message": situation.message,
        "blocked": situation.blocked,
        "state": list(situation.vocabulary.state),
        "decisions": decisions,
    }
    (folder / "escalation.json").write_text(json.dumps(data, indent=2, default=str))
    lines = [
        f"# Mission stopped: {final.action}",
        "",
        f"**Why:** {final.reason}",
        "",
        f"**Instruction:** {situation.instruction}",
        "",
        "## What failed",
        "",
        f"- Repair loop: {situation.message}",
    ]
    if situation.failure:
        lines.append(
            f"- Last failed step: {situation.failure.get('step')} "
            f"({', '.join(situation.failure.get('facts', []))})"
        )
    if situation.blocked:
        lines.append(f"- Ruled out: {situation.blocked}")
    lines += [
        "",
        "## Goal",
        "",
        "Original:",
        *[f"- {g}" for g in situation.original_goal],
    ]
    if situation.goal != situation.original_goal:
        lines += ["", "Current (relaxed):", *[f"- {g}" for g in situation.goal]]
    lines += ["", "## Decisions", ""]
    for i, d in enumerate(decisions, 1):
        lines.append(f"{i}. **{d['action']}**: {d['reason']}")
    lines += ["", "## State", "", *[f"- {a}" for a in situation.vocabulary.state]]
    md = folder / "escalation.md"
    md.write_text("\n".join(lines) + "\n")
    return md


__all__ = [
    "ACTIONS",
    "Decision",
    "Limits",
    "SUPERVISE_ROLE",
    "Situation",
    "Supervised",
    "check_decision",
    "supervise",
    "write_report",
]
