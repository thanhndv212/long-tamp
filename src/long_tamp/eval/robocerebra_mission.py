"""RoboCerebra as a mission domain (#186, R9): the whole task planned by
long_tamp's mission loop instead of one benchmark segment at a time.

A case's ``goal.json`` is one ordered chain of goals per object
(``popcorn_1: pick > pour plate_1 > in coffee_table_table_region``). The
benchmark's checker (``_check_success``) is a counter per object: the
object's next goal counts once it holds, and only that one. There is no
order across objects. The domain says exactly that to a task planner:

- ``progress(object, sK)``: the object's counter is at ``K`` (the env's
  ``check()["progress"]``, observed);
- ``goal_pick(object, sK)``, ``goal_place(object, sK, region)``,
  ``goal_pour(object, sK, receiver)``: the object's goal number ``K``
  (static facts, from ``goal.json``); ``next(sK, sK+1)`` orders them;
- ``grasped(gripper, object)``: what the hand holds (observed from the
  finger contacts);
- ``rest(object, sK)``: at ``K`` the object may be let go of: its next goal
  is a pick, it has none left, or it is given up on (static).

``pick``, ``place`` and ``pour`` move an object's counter on when its next
goal is theirs; ``regrasp`` and ``putdown`` take and release an object
without moving it on: ``regrasp`` an object that must be held and is not
(it fell), ``putdown`` one that need not be (to free the hand). So a plan
never puts down what it is carrying to a goal, nor picks up what it need
not hold. The goal is every object's counter at the end of its chain, cut
before the first goal the domain has no capability for (``open``,
``turnon``: #161): the checker never counts past it.

This module is the symbolic half; the geometry and execution (the
``SegmentPlanner`` refining each step, the OSC tracker running it) follow.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from typing import Any

from long_tamp.mission.domain import BaseMissionDomain
from long_tamp.mission.goals import grasped
from long_tamp.tasks.task_planning.capabilities import CapabilityDescriptor
from long_tamp.tasks.task_planning.language import Vocabulary
from long_tamp.tasks.task_planning.predicates import Atom

from .robocerebra_plan import GRIPPER

#: ``goal.json`` predicate -> the capability that reaches it.
GOAL_CAPABILITIES = {"pick": "pick", "in": "place", "on": "place", "pour": "pour"}


def stage(index: int) -> str:
    """The constant for an object's counter at ``index``."""
    return f"s{index}"


def chains(goal: Mapping[str, Sequence[Sequence[str]]]) -> dict[str, list[list[str]]]:
    """Each object's goals, lower-cased, cut before the first unsupported
    one (the checker never counts past it)."""
    out = {}
    for obj, specs in goal.items():
        kept = []
        for spec in specs:
            spec = [str(spec[0]).lower(), *(str(a) for a in spec[1:])]
            if spec[0] not in GOAL_CAPABILITIES:
                break
            kept.append(spec)
        out[obj] = kept
    return out


def observed_atoms(
    progress: Mapping[str, int], held: str | None, gripper: str = GRIPPER
) -> list[str]:
    """The world as the domain's predicates: each object's counter (the
    env's ``progress``) and what the hand holds (an object without goals at
    ``s0``)."""
    atoms = [f"progress({obj}, {stage(int(k))})" for obj, k in progress.items()]
    if held is not None:
        atoms.append(grasped(held, gripper))
        if held not in progress:
            atoms.append(f"progress({held}, {stage(0)})")
    return atoms


class RoboCerebraDomain(BaseMissionDomain):
    """One RoboCerebra case: one Panda hand, the case's objects, and their
    chains of goals (``goal``: ``load``'s ``goal``, per object its
    predicates in order)."""

    name = "robocerebra"
    goal_notes = (
        "progress(object, sK) says K of the object's goals are reached; its "
        "goal number K is goal_pick, goal_place or goal_pour at sK. "
        "grasped(gripper, object) while the hand holds it."
    )
    intro = (
        "A Panda arm in a RoboCerebra scene moves objects between regions and "
        "pours them over receivers, reaching each object's goals in order."
    )

    def __init__(
        self,
        goal: Mapping[str, Sequence[Sequence[str]]],
        case: str = "robocerebra",
        gripper: str = GRIPPER,
        others: Iterable[str] = (),
    ) -> None:
        self.case = case
        self.gripper = gripper
        self.full_goal = {obj: [list(s) for s in specs] for obj, specs in goal.items()}
        self.chains = chains(goal)
        #: The scene's other objects: no goals, but the hand may hold one.
        self.others = tuple(o for o in others if o not in self.full_goal)
        #: The objects given up on (``give_up``).
        self.given_up: set[str] = set()

    @classmethod
    def from_info(cls, info: Mapping[str, Any]) -> RoboCerebraDomain:
        """From the env server's ``load`` reply."""
        return cls(
            info["goal"],
            case=str(info.get("case", "robocerebra")),
            others=info.get("objects", ()),
        )

    def give_up(self, obj: str) -> None:
        """Stop planning ``obj``'s goals: they leave the goal, and the
        object may be put down wherever its chain stands."""
        self.given_up.add(obj)

    # -- symbolic --------------------------------------------------------------

    def descriptors(self) -> dict[str, CapabilityDescriptor]:
        advance = ("progress(?object, ?next)", "not progress(?object, ?stage)")
        reached = ("progress(?object, ?stage)", "next(?stage, ?next)")
        pick = CapabilityDescriptor(
            "pick",
            "1.0",
            {"gripper": str, "object": str, "stage": str, "next": str},
            preconditions=(
                "not grasped(?gripper, _)",
                "goal_pick(?object, ?stage)",
                *reached,
            ),
            effects=("grasped(?gripper, ?object)", *advance),
            writes=("grasp_state",),
            restartable=True,
        )
        place = CapabilityDescriptor(
            "place",
            "1.0",
            {"gripper": str, "object": str, "region": str, "stage": str, "next": str},
            preconditions=(
                "grasped(?gripper, ?object)",
                "goal_place(?object, ?stage, ?region)",
                *reached,
            ),
            effects=("not grasped(?gripper, ?object)", *advance),
            writes=("grasp_state",),
            restartable=True,
        )
        pour = CapabilityDescriptor(
            "pour",
            "1.0",
            {
                "gripper": str,
                "object": str,
                "receiver": str,
                "stage": str,
                "next": str,
            },
            preconditions=(
                "grasped(?gripper, ?object)",
                "goal_pour(?object, ?stage, ?receiver)",
                *reached,
            ),
            effects=advance,
            writes=("object_pose",),
            restartable=True,
        )
        regrasp = CapabilityDescriptor(
            "regrasp",
            "1.0",
            {"gripper": str, "object": str, "stage": str},
            preconditions=(
                "not grasped(?gripper, _)",
                "progress(?object, ?stage)",
                "not rest(?object, ?stage)",
            ),
            effects=("grasped(?gripper, ?object)",),
            writes=("grasp_state",),
            restartable=True,
        )
        putdown = CapabilityDescriptor(
            "putdown",
            "1.0",
            {"gripper": str, "object": str, "stage": str},
            preconditions=(
                "grasped(?gripper, ?object)",
                "progress(?object, ?stage)",
                "rest(?object, ?stage)",
            ),
            effects=("not grasped(?gripper, ?object)",),
            writes=("grasp_state",),
            restartable=True,
        )
        return {d.capability_id: d for d in (pick, place, pour, regrasp, putdown)}

    def objects(self) -> tuple[str, ...]:
        """The objects with goals (receivers are objects too), then the
        others."""
        found = dict.fromkeys((*self.full_goal, *self.others))
        for specs in self.chains.values():
            for spec in specs:
                if spec[0] == "pour":
                    found.setdefault(spec[2])
        return tuple(found)

    def regions(self) -> tuple[str, ...]:
        found = dict.fromkeys(
            spec[2]
            for specs in self.chains.values()
            for spec in specs
            if GOAL_CAPABILITIES[spec[0]] == "place"
        )
        return tuple(found)

    def stages(self) -> tuple[str, ...]:
        longest = max((len(specs) for specs in self.chains.values()), default=0)
        return tuple(stage(k) for k in range(longest + 1))

    def static_facts(self) -> tuple[str, ...]:
        facts = [f"gripper({self.gripper})"]
        stages = self.stages()
        facts += [f"next({a}, {b})" for a, b in zip(stages, stages[1:])]
        facts += [f"rest({obj}, {stage(0)})" for obj in self.others]
        for obj, specs in self.chains.items():
            if obj in self.given_up:
                facts += [f"rest({obj}, {stage(k)})" for k in range(len(specs) + 1)]
            else:
                facts.append(f"rest({obj}, {stage(len(specs))})")
            for k, spec in enumerate(specs):
                capability = GOAL_CAPABILITIES[spec[0]]
                if capability == "pick":
                    facts.append(f"goal_pick({obj}, {stage(k)})")
                    if obj not in self.given_up:
                        facts.append(f"rest({obj}, {stage(k)})")
                elif capability == "place":
                    facts.append(f"goal_place({obj}, {stage(k)}, {spec[2]})")
                else:
                    facts.append(f"goal_pour({obj}, {stage(k)}, {spec[2]})")
        return tuple(facts)

    def static_preconditions(self) -> dict[str, tuple[str, ...]]:
        hand = ("gripper(?gripper)",)
        return {
            "pick": hand,
            "place": hand,
            "pour": hand,
            "regrasp": hand,
            "putdown": hand,
        }

    def default_goal(self) -> list[str]:
        return self.goal()

    def goal(self, without: Iterable[str] = ()) -> list[str]:
        """Every object's counter at the end of its chain, but for the
        objects given up on and those ``without``."""
        skip = self.given_up | set(without)
        return [
            f"progress({obj}, {stage(len(specs))})"
            for obj, specs in self.chains.items()
            if specs and obj not in skip
        ]

    def initial_atoms(self) -> list[str]:
        """The start: no goal reached, the hand empty."""
        return observed_atoms({obj: 0 for obj in self.full_goal}, None, self.gripper)

    def unsupported(self) -> dict[str, list[list[str]]]:
        """Per object, the goals cut from its chain."""
        return {
            obj: specs[len(self.chains[obj]) :]
            for obj, specs in self.full_goal.items()
            if len(specs) > len(self.chains[obj])
        }

    def goal_vocabulary(self, state: Iterable[str | Atom] = ()) -> Vocabulary:
        return Vocabulary.from_domain(
            self.descriptors(),
            [str(a) for a in state],
            objects=(*self.objects(), *self.regions(), *self.stages(), self.gripper),
            notes=self.goal_notes,
        )

    def scene_info(self) -> dict[str, Any]:
        return {"id": self.name, "case": self.case, "objects": list(self.objects())}

    def expand_step(
        self,
        index: int,
        capability: str,
        parameters: dict[str, Any],
        held: Mapping[str, str],
    ) -> list[tuple[str, dict[str, Any], str]]:
        """One transaction per step, labelled without the counters."""
        shown = [
            str(parameters[k])
            for k in ("object", "region", "receiver")
            if k in parameters
        ]
        return [(capability, parameters, f"{capability} {' '.join(shown)}")]

    # -- repair --------------------------------------------------------------

    def repair_policy(self, failure: Mapping[str, Any]) -> list[Any]:
        """Nothing to block: every goal names its region or receiver, so a
        failed binding is the only one. A replan from the observed world is
        the repair (the geometry samples again)."""
        return []


__all__ = [
    "GOAL_CAPABILITIES",
    "RoboCerebraDomain",
    "chains",
    "observed_atoms",
    "stage",
]
