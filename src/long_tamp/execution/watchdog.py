"""A watchdog for steps that plan too long (#109).

``StepWatchdog`` follows a mission's event stream (``observe``, as an event
sink) and the clock (a thread). When the step being planned passes
``soft`` seconds, it decides what to do, from a fixed menu:

- ``wait``: give it longer (``wait_s`` seconds, never past ``hard``);
- ``skip``: abandon the search the step is in (it plans without its result);
- ``abort_step``: end the step now, as a failure; the run stops there.

A **model** decides when a client is given (a typed role, ADR-0006: its
decision is checked, and the rule decides when it can't be used); otherwise
the **rule**: skip if the step is in a search, else wait. At ``hard``, the
rule aborts, whatever was decided before: the model can't extend past it.
Decisions act through the ``ExecutionControl`` (``request``), as an
operator's buttons do, and are reported as ``watchdog`` events and to
``on_decision``. An operator's request always wins: the watchdog never
replaces one that is pending.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from long_tamp.tasks.task_planning.events import EventSink, make_event

WATCHDOG_ROLE = "watchdog"
ACTIONS = ("wait", "skip", "abort_step")


@dataclass
class StepWatch:
    """What the watchdog knows about the step being planned."""

    step_id: str
    label: str
    started: float
    progress: list[str] = field(default_factory=list)
    search: str | None = None
    deadline: float = 0.0  # when to decide next (monotonic)
    decisions: list[dict[str, Any]] = field(default_factory=list)

    def elapsed(self, now: float) -> float:
        return now - self.started


@dataclass
class WatchDecision:
    action: str
    reason: str = ""
    wait_s: float | None = None
    by: str = "rule"


def check_decision(
    decision: WatchDecision, watch: StepWatch, remaining: float
) -> list[str]:
    """Why ``decision`` can't be applied (empty: it can)."""
    errors = []
    if decision.action not in ACTIONS:
        errors.append(f"action must be one of {list(ACTIONS)}, not {decision.action!r}")
    if decision.action == "skip" and watch.search is None:
        errors.append("skip: the step is not in a search now")
    if decision.action == "wait":
        if decision.wait_s is None or decision.wait_s <= 0:
            errors.append("wait: give wait_minutes > 0")
        elif decision.wait_s > remaining:
            errors.append(
                f"wait: at most {remaining / 60:.1f} more minutes before the hard limit"
            )
    if not decision.reason.strip():
        errors.append("give a reason")
    return errors


def rule_decision(watch: StepWatch, remaining: float, wait_s: float) -> WatchDecision:
    """No model: skip a search, else wait (the hard limit aborts)."""
    if watch.search is not None:
        return WatchDecision(
            "skip", f"over the soft limit while in a search ({watch.search})"
        )
    return WatchDecision(
        "wait", "over the soft limit, but not in a search", min(wait_s, remaining)
    )


class StepWatchdog:
    """See the module docstring. ``decide(watch, elapsed, remaining)`` is the
    model's turn (``model_decider``); ``None``: the rule decides."""

    def __init__(
        self,
        control: Any,
        soft: float = 300.0,
        hard: float = 900.0,
        wait_s: float = 120.0,
        decide: Callable[[StepWatch, float, float], WatchDecision] | None = None,
        sink: EventSink | None = None,
        on_decision: Callable[[StepWatch, WatchDecision], None] | None = None,
        poll: float = 1.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not 0 < soft <= hard:
            raise ValueError("need 0 < soft <= hard")
        self.control, self.soft, self.hard, self.wait_s = control, soft, hard, wait_s
        self.decide, self.sink, self.on_decision = decide, sink, on_decision
        self.poll, self.clock = poll, clock
        self.watch: StepWatch | None = None
        self.decisions: list[dict[str, Any]] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # -- the event stream ------------------------------------------------------

    def observe(self, event: dict[str, Any]) -> None:
        """An event sink: follows which step is planning, and its progress."""
        role, status = event.get("role"), event.get("status")
        with self._lock:
            watch = self.watch
            if role == "attempts" and status == "RUNNING":
                now = self.clock()
                self.watch = StepWatch(
                    event["ir_id"], event.get("name", event["ir_id"]), now,
                    deadline=now + self.soft,
                )  # fmt: skip
                if self.watch.label.endswith(" retry"):
                    self.watch.label = self.watch.label[: -len(" retry")]
            elif watch is None or event.get("ir_id") != watch.step_id:
                return
            elif role == "progress":
                watch.progress.append(event.get("message", ""))
                del watch.progress[:-12]
                watch.search = (event.get("metrics") or {}).get("search")
            elif role == "execute" and status == "FAILURE":
                # the next attempt (if any) plans again: watch it afresh; an
                # "attempts" FAILURE follows when none is left
                now = self.clock()
                self.watch = StepWatch(
                    watch.step_id, watch.label, now, deadline=now + self.soft
                )
            elif role in ("execute", "attempts", "transaction") and status != "RUNNING":
                self.watch = None  # planned (or failed): not planning any more

    # -- the clock -------------------------------------------------------------

    def start(self) -> StepWatchdog:
        self._thread = threading.Thread(target=self._run, name="watchdog", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        while not self._stop.wait(self.poll):
            self.tick()

    def tick(self) -> WatchDecision | None:
        """Decide if the step being planned is due (the thread calls this)."""
        now = self.clock()
        with self._lock:
            watch = self.watch
            if watch is None or now < watch.deadline:
                return None
            if getattr(self.control, "pending", None) is not None:
                return None  # an operator's request is on its way
            if getattr(self.control, "paused", False):
                return None
            elapsed = watch.elapsed(now)
            remaining = max(self.hard - elapsed, 0.0)
            watch.deadline = float("inf")  # one decision at a time
        if remaining <= 0:
            decision = WatchDecision(
                "abort_step", f"hard limit ({self.hard:.0f} s) reached", by="rule"
            )
        else:
            decision = self._decide(watch, elapsed, remaining)
        self._apply(watch, decision, elapsed, now)
        return decision

    def _decide(
        self, watch: StepWatch, elapsed: float, remaining: float
    ) -> WatchDecision:
        if self.decide is not None:
            try:
                decision = self.decide(watch, elapsed, remaining)
                if not check_decision(decision, watch, remaining):
                    return decision
            except Exception:  # noqa: BLE001 - the rule decides instead
                pass
        return rule_decision(watch, remaining, self.wait_s)

    def _apply(
        self, watch: StepWatch, decision: WatchDecision, elapsed: float, now: float
    ) -> None:
        if decision.action == "wait":
            wait = min(decision.wait_s or self.wait_s, max(self.hard - elapsed, 0.0))
            watch.deadline = now + wait
        else:
            self.control.request(decision.action, f"watchdog ({decision.by})")
            # if the request isn't taken (no checkpoint before the step ends,
            # or a skip outside a search), the hard limit still applies
            watch.deadline = watch.started + self.hard
        record = {
            "step": watch.step_id,
            "action": decision.action,
            "reason": decision.reason,
            "by": decision.by,
            "elapsed": round(elapsed, 1),
        }
        if decision.action == "wait":
            record["wait_s"] = round(watch.deadline - now, 1)
        watch.decisions.append(record)
        self.decisions.append(record)
        if self.sink is not None:
            self.sink(
                make_event(
                    watch.step_id,
                    WATCHDOG_ROLE,
                    watch.label,
                    "SUCCESS",
                    message=f"{decision.action}: {decision.reason}",
                    metrics={k: v for k, v in record.items() if k != "reason"},
                )
            )
        if self.on_decision is not None:
            self.on_decision(watch, decision)


# -- the model's turn ----------------------------------------------------------------

_SYSTEM = """\
You watch a robot mission's planner. One step has been planning longer than \
expected. Decide what to do now, from this menu only:
- "wait": give it more time ("wait_minutes"); right when it is making progress \
(phases advancing, a search narrowing) and time remains before the hard limit.
- "skip": abandon the search the step is in; the step then plans without the \
search's result (it may fail later, but the mission moves on). Only while the step \
is in a search.
- "abort_step": end the step now, as a failure; the run stops and the operator or \
the chat can plan around it. Right when the step looks stuck or hopeless.
Answer one JSON object: {"action": ..., "wait_minutes": number or null, "reason": \
one sentence}."""

_SCHEMA = {
    "type": "object",
    "properties": {
        "action": {"type": "string", "enum": list(ACTIONS)},
        "wait_minutes": {"type": ["number", "null"]},
        "reason": {"type": "string"},
    },
    "required": ["action", "wait_minutes", "reason"],
}


def _render(request: tuple[StepWatch, float, float], feedback: list[str]) -> str:
    watch, elapsed, remaining = request
    lines = [
        f"Step: {watch.label} ({watch.step_id})",
        f"Planning for {elapsed / 60:.1f} min; {remaining / 60:.1f} min left before "
        "the hard limit (then it is aborted).",
        f"In a search now: {watch.search or 'no'}",
        "Latest progress (oldest first):",
        *([f"- {p}" for p in watch.progress] or ["- (none reported)"]),
    ]
    if watch.decisions:
        lines += ["Earlier decisions for this step:"]
        lines += [
            f"- {d['action']} at {d['elapsed'] / 60:.1f} min" for d in watch.decisions
        ]
    if feedback:
        lines += ["", *feedback, "Decide again."]
    return "\n".join(lines)


def _parse(answer: Any) -> WatchDecision:
    from long_tamp.ai.roles import Rejected

    if not isinstance(answer, dict) or "action" not in answer:
        raise Rejected(
            'answer like {"action": "wait", "wait_minutes": 2, "reason": "..."}'
        )
    minutes = answer.get("wait_minutes")
    try:
        wait_s = float(minutes) * 60 if minutes is not None else None
    except (TypeError, ValueError):
        raise Rejected("wait_minutes must be a number") from None
    return WatchDecision(
        str(answer["action"]).strip(), str(answer.get("reason", "")), wait_s, by="model"
    )


def model_decider(
    client: Any, max_rounds: int = 2
) -> Callable[[StepWatch, float, float], WatchDecision]:
    """The model's turn, for ``StepWatchdog(decide=...)``: a checked role
    whose fallback is the rule."""
    from long_tamp.ai.roles import ModelRole, refine

    role = ModelRole(
        name="watchdog", system=_SYSTEM, schema=_SCHEMA, render=_render, parse=_parse
    )

    def decide(watch: StepWatch, elapsed: float, remaining: float) -> WatchDecision:
        outcome = refine(
            role.proposer(client, (watch, elapsed, remaining)),
            lambda d: check_decision(d, watch, remaining),
            max_rounds=max_rounds,
            fallback=lambda why: _fallback(watch, remaining, why),
            name="watchdog",
        )
        return outcome.value

    return decide


def _fallback(watch: StepWatch, remaining: float, why: str) -> WatchDecision:
    decision = rule_decision(watch, remaining, 120.0)
    decision.reason += f" (the model couldn't decide: {why})"
    return decision


__all__ = [
    "ACTIONS",
    "WATCHDOG_ROLE",
    "StepWatch",
    "StepWatchdog",
    "WatchDecision",
    "check_decision",
    "model_decider",
    "rule_decision",
]
