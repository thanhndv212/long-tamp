"""Refiner: bind a plan step to geometry, with recovery (ADR-0001).

A task planner produces a plan skeleton; the refiner binds each step to
grasps, configurations and paths on HPP's constraint graph. On failure it
returns *facts* the task planner can use (the phase it could not reach),
not just a message.

:class:`Refiner` is the interface. :class:`GraspSequenceRefiner` is the HPP
implementation: a step is a grasp sequence planned with
:func:`~long_tamp.tasks.block_recovery.run_block_with_recovery`, optionally
guided by a phase-target lookahead
(:func:`~long_tamp.tasks.block_recovery.make_lookahead_hints_factory`).

Per ADR-0001 the interface is kept small and is not frozen until a second
implementation exists.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Protocol, Sequence

from .block_recovery import make_lookahead_hints_factory, run_block_with_recovery

if TYPE_CHECKING:
    from .grasp_sequence import GraspSequencePlanner

# Failure kinds from run_block_with_recovery -> the predicate reporting them.
FAILURE_PREDICATES = {
    "unreachable": "unreachable",
    "stuck": "phase_failed",
    "hint_chain_broken": "lookahead_failed",
}


@dataclass(frozen=True)
class Lookahead:
    """Probe phase ``pair[0]``'s target so that phase ``pair[1]`` (and every
    phase in ``also``) stays reachable from it."""

    pair: tuple[int, int] = (0, 1)
    also: tuple[int, ...] = ()
    verify_paths: bool = False


@dataclass(frozen=True)
class RefinementStep:
    """One step of a plan skeleton: a grasp sequence to bind to geometry.

    ``sequence`` holds ``(gripper, handle)`` phases, ``handle=None``
    releasing. ``frozen`` is ``{phase_idx: [arm keyword, ...]}`` (``None``:
    the planner decides).
    """

    label: str
    sequence: tuple[tuple[str, str | None], ...]
    frozen: dict[int, list[str]] | None = None
    lookahead: Lookahead | None = None


@dataclass
class Refinement:
    """A refined step, or why it could not be refined."""

    success: bool
    label: str
    final_config: list[float]
    phases: list[dict[str, Any]] = field(default_factory=list)
    resumes: int = 0
    replans: int = 0
    message: str = ""
    facts: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        """The ``run_block_with_recovery`` result shape, plus ``facts``."""
        return {
            "success": self.success,
            "final_config": self.final_config,
            "replans": self.replans,
            "resumes": self.resumes,
            "message": self.message,
            "label": self.label,
            "facts": list(self.facts),
        }


class Refiner(Protocol):
    def refine(self, step: RefinementStep, q_init: Sequence[float]) -> Refinement:
        """Bind ``step`` to geometry from ``q_init``."""
        ...


def _constant(text: str) -> str:
    """``text`` as a predicate argument (no whitespace, commas, parentheses)."""
    return re.sub(r"[\s,()?]+", "_", text).strip("_") or "_"


def failure_facts(step: RefinementStep, failure: dict[str, Any] | None) -> list[str]:
    """Ground atoms describing a failed refinement.

    Always ``refinement_failed(<label>)``; plus, when the failing phase is
    known, ``unreachable`` / ``phase_failed`` / ``lookahead_failed``
    ``(<gripper>, <handle>)`` (``handle`` ``none`` for a release).
    """
    facts = [f"refinement_failed({_constant(step.label)})"]
    if not failure:
        return facts
    predicate = FAILURE_PREDICATES.get(failure.get("kind", ""))
    idx = failure.get("phase_idx")
    if predicate and idx is not None and 0 <= idx < len(step.sequence):
        gripper, handle = step.sequence[idx]
        facts.append(
            f"{predicate}({_constant(gripper)}, {_constant(handle or 'none')})"
        )
    return facts


class GraspSequenceRefiner:
    """Refine grasp-sequence steps on a ``GraspSequencePlanner``.

    ``recovery`` is passed to ``run_block_with_recovery`` (``resume_limit``,
    ``unreachable_resumes``, ``unfreeze_after``, ...).
    """

    def __init__(
        self,
        planner: GraspSequencePlanner,
        q_scene_init: Sequence[float] | None = None,
        max_replans: int | None = 10,
        verbose: bool = True,
        **recovery: Any,
    ) -> None:
        self.planner = planner
        self.q_scene_init = list(q_scene_init) if q_scene_init is not None else None
        self.max_replans = max_replans
        self.verbose = verbose
        self.recovery = recovery

    def refine(self, step: RefinementStep, q_init: Sequence[float]) -> Refinement:
        hints_factory = None
        if step.lookahead is not None:
            hints_factory = make_lookahead_hints_factory(
                self.planner,
                step.sequence,
                q_init,
                q_scene_init=self.q_scene_init,
                per_phase_frozen_arms=step.frozen,
                phase_pair=step.lookahead.pair,
                also_protect=step.lookahead.also,
                verify_paths=step.lookahead.verify_paths,
                verbose=self.verbose,
            )
        r = run_block_with_recovery(
            self.planner,
            step.sequence,
            q_init,
            q_scene_init=self.q_scene_init,
            per_phase_frozen_arms=step.frozen,
            label=step.label,
            hints_factory=hints_factory,
            max_replans=self.max_replans,
            verbose=self.verbose,
            **self.recovery,
        )
        return Refinement(
            success=r["success"],
            label=step.label,
            final_config=list(r["final_config"]),
            phases=list(self.planner.phase_results) if r["success"] else [],
            resumes=r["resumes"],
            replans=r["replans"],
            message=r["message"],
            facts=[] if r["success"] else failure_facts(step, r.get("failure")),
        )


__all__ = [
    "FAILURE_PREDICATES",
    "GraspSequenceRefiner",
    "Lookahead",
    "Refinement",
    "RefinementStep",
    "Refiner",
    "failure_facts",
]
