"""Skills: capabilities that end in controller-level behaviour (ADR-0001).

A motion-planned capability moves the robot along collision-free paths. Some
steps can't be planned to the millimetre: driving a screw, inserting a peg,
pressing a button. A *skill* splits such a step in two:

- **planned motion** brings the robot to the skill's ``start`` pose (e.g. the
  driver's tip a few millimetres before the hole, on its axis): the planner
  does this part, as for any capability;
- the **skill** itself runs from there on the robot's controller (compliant
  approach, force or torque thresholds) to its ``end`` pose, and reports
  whether it succeeded: its ``postconditions`` as facts, or a failure fact
  (``failures``) that the task planner can plan around.

A :class:`SkillSpec` declares this contract; a :class:`SkillCommand` is what
the executor sends a backend for one grounded run of it. A backend that can
run the skill (the MuJoCo backend runs registered skill controllers) does; any
other backend executes ``command.approach``, the planned path from ``start``
to ``end``, as ordinary motion: a ``SkillCommand`` is a path too.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any

from .capabilities import CapabilityDescriptor
from .predicates import Literal


@dataclass(frozen=True)
class SkillPose:
    """Where ``frame`` is with respect to ``target`` (both may be ``?parameters``).

    ``offset`` is the distance along the target's approach axis, in metres:
    negative before contact (a start pose), 0 at the target (an end pose).
    """

    frame: str
    target: str
    offset: float = 0.0

    def ground(self, parameters: Mapping[str, str]) -> SkillPose:
        return SkillPose(
            _ground(self.frame, parameters),
            _ground(self.target, parameters),
            self.offset,
        )


@dataclass(frozen=True)
class SkillSpec:
    """The contract of a skill.

    ``preconditions`` / ``postconditions`` are literals over ``?parameters``
    (the same syntax as capability preconditions and effects).
    ``failures`` are failure-fact templates the skill may report instead of
    its postconditions. ``start`` is where planned motion leaves the robot,
    ``end`` where the skill leaves it on success.
    """

    name: str
    parameters: tuple[str, ...]
    preconditions: tuple[str, ...] = ()
    postconditions: tuple[str, ...] = ()
    failures: tuple[str, ...] = ()
    start: SkillPose | None = None
    end: SkillPose | None = None
    description: str = ""

    def __post_init__(self) -> None:
        for text in (*self.preconditions, *self.postconditions, *self.failures):
            unknown = Literal.parse(text).variables() - set(self.parameters)
            if unknown:
                raise ValueError(
                    f"skill {self.name}: {text!r} uses undeclared parameters "
                    f"{sorted(unknown)}"
                )

    def ground(
        self, parameters: Mapping[str, str], templates: tuple[str, ...]
    ) -> list[str]:
        """``templates`` (e.g. ``postconditions``) with ``parameters`` filled in."""
        missing = set(self.parameters) - set(parameters)
        if missing:
            raise ValueError(f"skill {self.name}: missing parameters {sorted(missing)}")
        return [_ground_literal(t, parameters) for t in templates]

    def descriptor(self, version: str = "1") -> CapabilityDescriptor:
        """The skill as a TaskPlan capability: its conditions, string parameters."""
        return CapabilityDescriptor(
            capability_id=self.name,
            version=version,
            required_parameters={p: str for p in self.parameters},
            preconditions=self.preconditions,
            effects=self.postconditions,
            safety_class="motion",
        )


@dataclass
class SkillCommand:
    """One grounded run of a skill: the payload of an ``ExecutionCommand``.

    ``approach`` is the planned path from the skill's start pose to its end
    pose. Backends that don't run skills execute it as ordinary motion: a
    ``SkillCommand`` answers ``length()``, ``eval(t)`` and anything else (an
    HPP path's ``timeRange()``) like the path.
    """

    spec: SkillSpec
    parameters: dict[str, str]
    approach: Any
    options: dict[str, Any] = field(default_factory=dict)

    @property
    def name(self) -> str:
        return self.spec.name

    def postconditions(self) -> list[str]:
        return self.spec.ground(self.parameters, self.spec.postconditions)

    def failure(self, predicate: str) -> str:
        """The grounded failure fact whose predicate is ``predicate``."""
        for template in self.spec.failures:
            if Literal.parse(template).atom.name == predicate:
                return _ground_literal(template, self.parameters)
        raise KeyError(f"skill {self.name} declares no failure {predicate!r}")

    # A SkillCommand is also its approach path.
    def length(self) -> float:
        return self.approach.length()

    def eval(self, t: float):
        return self.approach.eval(t)

    def __getattr__(self, name: str) -> Any:
        if name == "approach":
            raise AttributeError(name)
        return getattr(self.approach, name)


def _ground(term: str, parameters: Mapping[str, str]) -> str:
    return parameters[term[1:]] if term.startswith("?") else term


def _ground_literal(template: str, parameters: Mapping[str, str]) -> str:
    return str(Literal.parse(template).ground(parameters))
