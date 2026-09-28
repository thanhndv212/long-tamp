"""Export capabilities, a world state and a goal as PDDL (ADR-0001, roadmap M3).

``to_pddl(descriptors, init, goal)`` writes a PDDL domain and problem any
classical planner can read: one action per capability that has effects, its
preconditions and effects translated from the predicate language
(``predicates.py``). A plan found for them is a *skeleton*: a sequence of
grounded capability calls, which ``from_pddl_plan`` maps back to capability
ids and parameter values, for a TaskPlan to be built from.

Translation:

- ``holds(?g, ?h)`` -> ``(holds ?g ?h)``; ``not holds(?g, ?h)`` -> ``(not ...)``.
- The wildcard ``_``: ``not holds(?g, _)`` as a precondition becomes
  ``(not (exists (?w0) (holds ?g ?w0)))``; as an effect,
  ``(forall (?w0) (not (holds ?g ?w0)))``. A positive precondition with ``_``
  becomes ``(exists ...)``.
- Only parameters that appear in a literal are action parameters. The others
  (e.g. an implementation's block label) don't affect planning; they are left
  for whoever builds the TaskPlan from the skeleton.
- Names: PDDL names are letters, digits, ``-`` and ``_``, starting with a
  letter, so constants such as ``ur10_left/gripper`` are renamed
  (``ur10_left__gripper``); ``PddlExport.names`` maps every PDDL name back.

Grounding cost is the planner's: every action parameter ranges over every
object, and a parameter that only appears in effects is unconstrained. So
``static_preconditions`` adds, per capability, preconditions over *static*
facts (``can_grasp(?gripper, ?handle)``, ``hole_of(?part, ?hole)``) given in
``init`` and changed by no action. They exist only in the export: the
capabilities' run-time preconditions are unchanged.
"""

from __future__ import annotations

import re
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from typing import Any

from .capabilities import CapabilityDescriptor
from .predicates import WILDCARD, Atom, Literal, parse_atom

_VALID = re.compile(r"[A-Za-z][A-Za-z0-9_-]*")


@dataclass
class PddlExport:
    """A PDDL domain and problem, and the name mapping back to long_tamp."""

    domain: str
    problem: str
    #: PDDL name -> original name, for objects, predicates and actions.
    names: dict[str, str] = field(default_factory=dict)
    #: action name -> the capability parameters it has, in PDDL order.
    parameters: dict[str, list[str]] = field(default_factory=dict)


class _Names:
    """Reversible renaming of long_tamp names to valid PDDL names."""

    def __init__(self) -> None:
        self.to_pddl: dict[str, str] = {}
        self.back: dict[str, str] = {}

    def __call__(self, name: str) -> str:
        if name in self.to_pddl:
            return self.to_pddl[name]
        base = name if _VALID.fullmatch(name) else _sanitize(name)
        candidate, n = base, 1
        while candidate.lower() in {k.lower() for k in self.back}:
            n += 1
            candidate = f"{base}-{n}"
        self.to_pddl[name] = candidate
        self.back[candidate] = name
        return candidate


def _sanitize(name: str) -> str:
    out = re.sub(r"[^A-Za-z0-9_-]", "__", name)
    return out if out[:1].isalpha() else f"o_{out}"


def _literals(texts: Iterable[str]) -> list[Literal]:
    return [Literal.parse(text) for text in texts]


def _used_parameters(descriptor: CapabilityDescriptor) -> list[str]:
    used: list[str] = []
    for literal in (*descriptor.precondition_literals, *descriptor.effect_literals):
        for arg in literal.atom.args:
            if arg.startswith("?") and arg[1:] not in used:
                used.append(arg[1:])
    return used


def _term(arg: str, names: _Names) -> str:
    return f"?{arg[1:]}" if arg.startswith("?") else names(arg)


def _atom(atom: Atom, names: _Names, wildcards: list[str]) -> str:
    args = []
    for arg in atom.args:
        if arg == WILDCARD:
            var = f"?w{len(wildcards)}"
            wildcards.append(var)
            args.append(var)
        else:
            args.append(_term(arg, names))
    return f"({' '.join([names(atom.name), *args])})"


def _condition(literal: Literal, names: _Names) -> tuple[str, set[str]]:
    wildcards: list[str] = []
    body = _atom(literal.atom, names, wildcards)
    needs = set()
    if wildcards:
        body = f"(exists ({' '.join(wildcards)}) {body})"
        needs.add(":existential-preconditions")
    if not literal.positive:
        body = f"(not {body})"
        needs.add(":negative-preconditions")
    return body, needs


def _effect(literal: Literal, names: _Names) -> tuple[str, set[str]]:
    wildcards: list[str] = []
    body = _atom(literal.atom, names, wildcards)
    if literal.positive:
        return body, set()
    body = f"(not {body})"
    if wildcards:  # delete every matching fact
        return f"(forall ({' '.join(wildcards)}) {body})", {":conditional-effects"}
    return body, set()


def _conjunction(parts: list[str]) -> str:
    if not parts:
        return "(and)"
    return parts[0] if len(parts) == 1 else f"(and {' '.join(parts)})"


def to_pddl(
    descriptors: Mapping[str, CapabilityDescriptor] | Iterable[CapabilityDescriptor],
    init: Iterable[str | Atom],
    goal: Iterable[str],
    domain_name: str = "long-tamp",
    problem_name: str = "mission",
    objects: Iterable[str] = (),
    static_preconditions: Mapping[str, Iterable[str]] | None = None,
) -> PddlExport:
    """A PDDL domain for ``descriptors`` and a problem from ``init`` to ``goal``.

    ``init`` is ground atoms (the world state plus static facts); ``goal`` is
    ground literals (``"screwed(part1, part1/h_hole1)"``, ``"not holds(g, _)"``).
    Capabilities without effects (guard conditions) are not actions. Objects
    are every constant in ``init``, ``goal``, the capabilities' literals and
    ``objects``. ``static_preconditions`` maps capability ids to extra
    preconditions over static facts (see the module docstring).
    """
    static_preconditions = dict(static_preconditions or {})
    items = descriptors.values() if isinstance(descriptors, Mapping) else descriptors
    names = _Names()
    requirements = {":strips"}
    predicates: dict[str, int] = {}
    constants: list[str] = []
    domain_constants: list[str] = []

    def note(atom: Atom, in_domain: bool = False) -> None:
        arity = predicates.setdefault(atom.name, len(atom.args))
        if arity != len(atom.args):
            raise ValueError(
                f"predicate {atom.name} used with arities {arity} and {len(atom.args)}"
            )
        for arg in atom.args:
            if arg.startswith("?") or arg == WILDCARD:
                continue
            target = domain_constants if in_domain else constants
            if arg not in target:
                target.append(arg)

    actions, parameters = [], {}
    for descriptor in sorted(items, key=lambda d: d.capability_id):
        if not descriptor.effects:
            continue
        action = names(descriptor.capability_id)
        static = _literals(static_preconditions.pop(descriptor.capability_id, ()))
        used = _used_parameters(descriptor)
        for literal in static:
            unknown = literal.variables() - set(descriptor.required_parameters)
            if unknown:
                raise ValueError(
                    f"{descriptor.capability_id}: static precondition {literal} "
                    f"uses unknown parameter(s) {sorted(unknown)}"
                )
            for arg in literal.atom.args:
                if arg.startswith("?") and arg[1:] not in used:
                    used.append(arg[1:])
        parameters[action] = used
        pre, eff = [], []
        for literal in (*static, *descriptor.precondition_literals):
            note(literal.atom, in_domain=True)
            text, needs = _condition(literal, names)
            pre.append(text)
            requirements |= needs
        for literal in descriptor.effect_literals:
            note(literal.atom, in_domain=True)
            text, needs = _effect(literal, names)
            eff.append(text)
            requirements |= needs
        actions.append(
            f"  (:action {action}\n"
            f"    :parameters ({' '.join('?' + p for p in used)})\n"
            f"    :precondition {_conjunction(pre)}\n"
            f"    :effect {_conjunction(eff)})"
        )

    if static_preconditions:
        raise ValueError(
            f"static preconditions for unknown or effect-less capabilities: "
            f"{sorted(static_preconditions)}"
        )
    init_atoms = [a if isinstance(a, Atom) else parse_atom(str(a)) for a in init]
    goal_literals = _literals(goal)
    for atom in init_atoms:
        note(atom)
    for literal in goal_literals:
        note(literal.atom)
    for obj in objects:
        if obj not in constants:
            constants.append(obj)

    goal_parts = []
    for literal in goal_literals:
        text, needs = _condition(literal, names)
        goal_parts.append(text)
        requirements |= needs

    predicate_decls = " ".join(
        f"({names(name)} {' '.join(f'?a{i}' for i in range(arity))})".replace(" )", ")")
        for name, arity in sorted(predicates.items())
    )
    order = [":strips", ":negative-preconditions", ":existential-preconditions",
             ":conditional-effects"]  # fmt: skip
    constants_decl = ""
    if domain_constants:
        constants_decl = (
            f"  (:constants {' '.join(names(c) for c in domain_constants)})\n"
        )
    domain = (
        f"(define (domain {domain_name})\n"
        f"  (:requirements {' '.join(r for r in order if r in requirements)})\n"
        + constants_decl
        + f"  (:predicates {predicate_decls})\n"
        + "\n".join(actions)
        + "\n)\n"
    )
    object_names = " ".join(names(c) for c in constants if c not in domain_constants)
    init_text = " ".join(_atom(a, names, []) for a in sorted(set(init_atoms)))
    problem = (
        f"(define (problem {problem_name})\n"
        f"  (:domain {domain_name})\n"
        f"  (:objects {object_names})\n"
        f"  (:init {init_text})\n"
        f"  (:goal {_conjunction(goal_parts)})\n"
        ")\n"
    )
    return PddlExport(domain, problem, dict(names.back), parameters)


def from_pddl_plan(
    export: PddlExport, steps: Iterable[str | tuple[str, ...]]
) -> list[tuple[str, dict[str, Any]]]:
    """Map a PDDL plan back to ``(capability_id, parameters)`` steps.

    ``steps`` are actions as ``"(grasp ur10_left__gripper part1__h_grasp)"``
    or ``("grasp", "ur10_left__gripper", "part1__h_grasp")``; names are
    matched case-insensitively (planners often lower-case them).
    """
    lower = {k.lower(): v for k, v in export.names.items()}
    actions = {k.lower(): k for k in export.parameters}
    result = []
    for step in steps:
        tokens = (
            step.strip().strip("()").split() if isinstance(step, str) else list(step)
        )
        action = actions.get(tokens[0].lower())
        if action is None:
            raise ValueError(f"unknown action in plan step {step!r}")
        params = export.parameters[action]
        if len(tokens) - 1 != len(params):
            raise ValueError(f"{step!r}: expected {len(params)} arguments")
        values = {}
        for name, token in zip(params, tokens[1:]):
            if token.lower() not in lower:
                raise ValueError(f"{step!r}: unknown object {token!r}")
            values[name] = lower[token.lower()]
        result.append((lower[action.lower()], values))
    return result


__all__ = ["PddlExport", "from_pddl_plan", "to_pddl"]
