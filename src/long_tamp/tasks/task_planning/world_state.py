"""World-state providers for runtime precondition and effect checks (ADR-0002).

A session's ``world_state`` is a callable returning the current ground atoms.
Two kinds of atoms feed it, and nothing else may:

- **observed** predicates, read from the world model each time they are asked
  for (:class:`GraspTrackerState` reads the planner's grasp tracker);
- **recorded** facts, which no sensor shows once the step is over (a screw
  driven in, a tool put back): :class:`RecordedFacts` holds them, and only a
  *completed execution* writes them, from the step's declared effects.

Planner bookkeeping (a plan computed, an attempt count) is never an atom.
:class:`CompositeWorldState` merges the sources into one callable.
"""

from __future__ import annotations

import json
import os
from collections.abc import Callable, Iterable
from pathlib import Path
from typing import Any

from .predicates import Atom, Literal, apply_effects, parse_state


class GraspTrackerState:
    """Observed ``holds(gripper, handle)`` atoms from a ``GraspStateTracker``.

    Reads ``tracker.current_grasps`` on every call, so it always reflects the
    planner's current grasp state.
    """

    def __init__(self, tracker: Any, predicate: str = "holds") -> None:
        self.tracker = tracker
        self.predicate = predicate

    def __call__(self) -> frozenset[Atom]:
        return frozenset(
            Atom(self.predicate, (gripper, handle))
            for gripper, handle in self.tracker.current_grasps.items()
            if handle is not None
        )


class RecordedFacts:
    """Facts for the given ``predicates``, written only by completed execution.

    With a ``path``, facts are loaded from it and every change is written
    atomically (temporary file, then ``os.replace``), so a restarted mission
    sees what earlier runs achieved. ``path=None`` keeps them in memory.
    """

    def __init__(self, path: str | Path | None, predicates: Iterable[str]) -> None:
        self.path = Path(path) if path is not None else None
        self.predicates = frozenset(predicates)
        self._facts: frozenset[Atom] = frozenset()
        if self.path is not None and self.path.exists():
            data = json.loads(self.path.read_text())
            self._facts = parse_state(data.get("facts", []))

    def __call__(self) -> frozenset[Atom]:
        return self._facts

    def apply(self, effects: Iterable[Literal]) -> list[Literal]:
        """Apply the ground effects on this store's predicates; return them.

        Effects on other predicates (observed ones) are ignored: the world
        model reports those itself.
        """
        own = [e for e in effects if e.atom.name in self.predicates]
        if not own:
            return []
        self._facts = apply_effects(self._facts, own)
        self._save()
        return own

    def _save(self) -> None:
        if self.path is None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(self.path.suffix + ".tmp")
        data = {
            "predicates": sorted(self.predicates),
            "facts": sorted(str(atom) for atom in self._facts),
        }
        tmp.write_text(json.dumps(data, indent=2))
        os.replace(tmp, self.path)


class CompositeWorldState:
    """The union of several world-state sources (callables returning atoms)."""

    def __init__(self, *sources: Callable[[], Iterable[Any]]) -> None:
        self.sources = sources

    def __call__(self) -> frozenset[Atom]:
        atoms: set[Atom] = set()
        for source in self.sources:
            atoms |= parse_state(source())
        return frozenset(atoms)
