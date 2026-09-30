"""Partial-order plans: which steps of a sequence can run concurrently.

A plan skeleton is a total order; most of it is forced (the left arm must
grasp a part before placing it), but not all of it: the right arm going home
doesn't care what the left arm does meanwhile. ``parallelize(document,
registry)`` rewrites a TaskPlan document so that runs of steps with no
ordering constraint between them become ``parallel`` nodes, one lane per
independent chain, which executors may run concurrently.

Two steps are ordered (the later *depends* on the earlier) when:

- they occupy the same resource: a capability's ``resources`` name the
  parameters whose values a step holds exclusively (``("gripper",)``), and
  values conflict when equal or one is a path prefix of the other
  (``ur10_left`` and ``ur10_left/gripper`` are the same arm; ``part1`` and
  ``part1/h_grasp`` the same object);
- their literals interfere: one's effects touch an atom the other's
  preconditions or effects mention (wildcards match anything);
- either step declares neither resources nor literals: nothing says what it
  touches, so it is ordered with everything.

This is the symbolic side only. Two independent steps may still collide when
their motions run at the same time; executors check that geometrically and
fall back to running the lanes one after another (see
``long_tamp.execution.concurrent``).
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Any

from .capabilities import CapabilityRegistry
from .model import grounded_literals
from .predicates import WILDCARD, Atom, Literal

#: Node types a lane may hold: the steps themselves.
STEP_TYPES = ("transaction", "operation")


@dataclass(frozen=True)
class Footprint:
    """What a step touches: resources held and literals read/written."""

    resources: frozenset[str]
    preconditions: tuple[Literal, ...]
    effects: tuple[Literal, ...]

    @property
    def unknown(self) -> bool:
        return not (self.resources or self.preconditions or self.effects)


def footprint(node: dict[str, Any], registry: CapabilityRegistry) -> Footprint:
    """The footprint of a step (a transaction or its operation)."""
    operation = node["children"][0] if node["type"] == "transaction" else node
    descriptor = registry.descriptor(operation["capability"])
    parameters = operation.get("parameters", {})
    pre, effects = grounded_literals(operation, descriptor)
    resources = frozenset(
        str(parameters[name]) for name in descriptor.resources if name in parameters
    )
    return Footprint(resources, tuple(pre), tuple(effects))


def _same_resource(a: str, b: str) -> bool:
    return a == b or a.startswith(b + "/") or b.startswith(a + "/")


def _atoms_overlap(a: Atom, b: Atom) -> bool:
    return (
        a.name == b.name
        and len(a.args) == len(b.args)
        and all(x == y or WILDCARD in (x, y) for x, y in zip(a.args, b.args))
    )


def _touches(effects: tuple[Literal, ...], literals: tuple[Literal, ...]) -> bool:
    return any(_atoms_overlap(e.atom, lit.atom) for e in effects for lit in literals)


def depends(earlier: Footprint, later: Footprint) -> bool:
    """Whether ``later`` must stay after ``earlier`` (see the module docstring)."""
    if earlier.unknown or later.unknown:
        return True
    if any(_same_resource(a, b) for a in earlier.resources for b in later.resources):
        return True
    return _touches(earlier.effects, later.preconditions + later.effects) or _touches(
        later.effects, earlier.preconditions
    )


def lanes(steps: list[Footprint]) -> list[list[list[int]]]:
    """Split consecutive steps into groups of independent lanes, in order.

    A group is a run of consecutive steps whose dependencies (between steps
    of the run; what came before is done by then) split into two or more
    connected components: its lanes, each in plan order. A run grows while it
    still splits; a step that would join everything up starts the next run.
    Steps that no run can take stay alone (a one-lane group).
    """
    groups: list[list[list[int]]] = []
    i = 0
    while i < len(steps):
        end = i + 1
        best = None
        while end < len(steps):
            components = _components(steps, i, end + 1)
            if len(components) < 2:
                break
            best, end = components, end + 1
        if best is None:
            groups.append([[i]])
            i += 1
        else:
            groups.append(best)
            i = end
    return groups


def _components(steps: list[Footprint], start: int, stop: int) -> list[list[int]]:
    """Connected components of the dependency graph on steps[start:stop]."""
    parent = {i: i for i in range(start, stop)}

    def root(i: int) -> int:
        while parent[i] != i:
            i = parent[i]
        return i

    for j in range(start, stop):
        for i in range(start, j):
            if depends(steps[i], steps[j]):
                parent[root(j)] = root(i)
    lanes: dict[int, list[int]] = {}
    for i in range(start, stop):
        lanes.setdefault(root(i), []).append(i)
    return sorted(lanes.values())


def parallelize(
    document: dict[str, Any], registry: CapabilityRegistry
) -> dict[str, Any]:
    """A copy of ``document`` where independent consecutive steps of every
    sequence run in ``parallel`` nodes. Other node kinds (conditions,
    fallbacks, retries) stay where they are and order what's around them."""
    document = deepcopy(document)
    document["root"] = _rewrite(document["root"], registry)
    return document


def _rewrite(node: dict[str, Any], registry: CapabilityRegistry) -> dict[str, Any]:
    if node["type"] == "retry":
        node["child"] = _rewrite(node["child"], registry)
        return node
    if node["type"] not in ("sequence", "fallback"):
        return node
    children = [_rewrite(child, registry) for child in node["children"]]
    if node["type"] == "fallback":
        node["children"] = children
        return node
    out: list[dict[str, Any]] = []
    run: list[dict[str, Any]] = []
    for child in children + [None]:
        if child is not None and child["type"] in STEP_TYPES:
            run.append(child)
            continue
        out.extend(_group(node["id"], len(out), run, registry))
        run = []
        if child is not None:
            out.append(child)
    node["children"] = out
    return node


def _group(
    sequence_id: str,
    offset: int,
    steps: list[dict[str, Any]],
    registry: CapabilityRegistry,
) -> list[dict[str, Any]]:
    """Consecutive steps as plain steps and parallel nodes."""
    out: list[dict[str, Any]] = []
    for group in lanes([footprint(step, registry) for step in steps]):
        if len(group) == 1:
            out.extend(steps[i] for i in group[0])
            continue
        par_id = f"{sequence_id}-par{offset + len(out)}"
        children = []
        for k, lane in enumerate(group):
            if len(lane) == 1:
                children.append(steps[lane[0]])
            else:
                children.append(
                    {
                        "type": "sequence",
                        "id": f"{par_id}-lane{k}",
                        "label": f"lane {k}",
                        "children": [steps[i] for i in lane],
                    }
                )
        out.append(
            {
                "type": "parallel",
                "id": par_id,
                "label": " | ".join(
                    steps[lane[0]].get("label", steps[lane[0]]["id"]) for lane in group
                ),
                "children": children,
            }
        )
    return out


def lane_steps(node: dict[str, Any]) -> list[dict[str, Any]]:
    """The steps in a lane (a step, or a sequence of steps), in order."""
    if node["type"] in STEP_TYPES:
        return [node]
    return [step for child in node["children"] for step in lane_steps(child)]
