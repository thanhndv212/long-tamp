"""Object-level goal predicates every mission domain can share (ADR-0011).

A goal is a list of literals in the predicate language (ADR-0002). Each
domain chooses its own predicates; the screw assembly's are
``holds(gripper, handle)`` and ``screwed(part, hole)``. Two are common
enough to share, so that a goal written outside a domain (a benchmark's
goal file, a VLM) can name them without knowing the domain's handles:

- ``grasped(gripper, object)``: ``gripper`` holds ``object``, by any of its
  handles. ``grasped(_, mug)``: something holds the mug.
- ``placed_at(object, region)``: ``object`` rests in or on ``region`` (a
  named place: a tray, a shelf, a drawer).

A domain that uses them declares capabilities with these effects (a pick
whose effect is ``grasped(?gripper, ?object)``, a place whose effect is
``placed_at(?object, ?region)``) and lists its regions
(``MissionDomain.regions``). :class:`ObjectLevelState` derives ``grasped``
from the grasp tracker's ``holds(gripper, handle)`` atoms, so ``grasped``
is observed, like ``holds``.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Sequence

from long_tamp.tasks.task_planning.predicates import (
    WILDCARD,
    Atom,
    Literal,
    parse_state,
)

GRASPED = "grasped"
PLACED_AT = "placed_at"
#: Predicate name -> arity.
OBJECT_PREDICATES = {GRASPED: 2, PLACED_AT: 2}


def object_of(handle: str) -> str:
    """The object a handle belongs to: ``part1/h_grasp`` -> ``part1``."""
    return handle.split("/", 1)[0]


def grasped(obj: str, gripper: str = WILDCARD, negate: bool = False) -> str:
    """The literal ``grasped(gripper, obj)`` (any gripper by default)."""
    return f"{'not ' if negate else ''}{GRASPED}({gripper}, {obj})"


def placed_at(obj: str, region: str, negate: bool = False) -> str:
    """The literal ``placed_at(obj, region)``."""
    return f"{'not ' if negate else ''}{PLACED_AT}({obj}, {region})"


class ObjectLevelState:
    """``grasped(gripper, object)`` atoms from a source of ``holds(gripper,
    handle)`` atoms (a ``GraspTrackerState``): a world-state source to add to
    a ``CompositeWorldState`` next to the one it reads."""

    def __init__(
        self, source: Callable[[], Iterable[object]], predicate: str = "holds"
    ):
        self.source = source
        self.predicate = predicate

    def __call__(self) -> frozenset[Atom]:
        return frozenset(
            Atom(GRASPED, (atom.args[0], object_of(atom.args[1])))
            for atom in parse_state(self.source())
            if atom.name == self.predicate and len(atom.args) == 2
        )


def check_object_goal(
    goal: Sequence[str], objects: Iterable[str], regions: Iterable[str]
) -> list[str]:
    """Why the ``grasped`` / ``placed_at`` literals of ``goal`` can't be used
    (empty: they can): an object that isn't one, a region that isn't one.
    Other literals are left to ``task_planning.language.check_goal``."""
    objects, regions = set(objects), set(regions)
    errors = []
    for text in goal:
        try:
            atom = Literal.parse(text).atom
        except ValueError as error:
            errors.append(str(error))
            continue
        if atom.name not in OBJECT_PREDICATES:
            continue
        if len(atom.args) != OBJECT_PREDICATES[atom.name]:
            errors.append(
                f"{text}: {atom.name} takes {OBJECT_PREDICATES[atom.name]} arguments"
            )
            continue
        obj = atom.args[1] if atom.name == GRASPED else atom.args[0]
        if obj != WILDCARD and obj not in objects:
            errors.append(f"{text}: unknown object {obj!r}")
        if atom.name == PLACED_AT and atom.args[1] not in regions | {WILDCARD}:
            errors.append(f"{text}: unknown region {atom.args[1]!r}")
    return errors


__all__ = [
    "GRASPED",
    "OBJECT_PREDICATES",
    "PLACED_AT",
    "ObjectLevelState",
    "check_object_goal",
    "grasped",
    "object_of",
    "placed_at",
]
