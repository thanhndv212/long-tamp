"""Predicate language for capability preconditions and effects (ADR-0002).

A literal is an atom, optionally negated::

    holds(?gripper, ?handle)        positive, two parameters
    not holds(?gripper, _)          negative, ``_`` matches any argument
    clamped(part1, fixtures/clamp1) ground

- The predicate name is lower-case (``[a-z][a-z0-9_]*``), PDDL-compatible.
- ``?name`` is bound to the step parameter ``name`` when the literal is grounded.
- ``_`` is a wildcard: a positive literal holds if *some* atom matches, a negative
  one if *none* does; a negative effect deletes every matching atom.
- Anything else is a constant: no whitespace, commas or parentheses, and it may
  not start with ``?`` (``left/gripper`` and ``ball/handle`` are fine).

States are closed-world sets of ground atoms.
"""

from __future__ import annotations

import re
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from typing import Any

WILDCARD = "_"
_NAME = re.compile(r"[a-z][a-z0-9_]*")
_VARIABLE = re.compile(r"\?[A-Za-z_][A-Za-z0-9_]*")
_CONSTANT = re.compile(r"[^\s,()?][^\s,()]*")
_ATOM = re.compile(r"^\s*([^\s(]+)\s*\((.*)\)\s*$")

State = frozenset  # frozenset[Atom]


@dataclass(frozen=True, order=True)
class Atom:
    """A predicate applied to arguments (variables, wildcards or constants)."""

    name: str
    args: tuple[str, ...] = ()

    def __str__(self) -> str:
        return f"{self.name}({', '.join(self.args)})"

    def variables(self) -> set[str]:
        return {arg[1:] for arg in self.args if arg.startswith("?")}

    @property
    def is_ground(self) -> bool:
        """No variables (wildcards allowed)."""
        return not self.variables()

    @property
    def has_wildcard(self) -> bool:
        return WILDCARD in self.args

    def ground(self, parameters: Mapping[str, Any]) -> Atom:
        args = []
        for arg in self.args:
            if not arg.startswith("?"):
                args.append(arg)
                continue
            name = arg[1:]
            if name not in parameters:
                raise ValueError(f"{self}: no value for parameter {name!r}")
            value = parameters[name]
            if not isinstance(value, str) or not _CONSTANT.fullmatch(value):
                raise ValueError(
                    f"{self}: parameter {name!r} must be a string without "
                    f"whitespace, commas or parentheses, got {value!r}"
                )
            args.append(value)
        return Atom(self.name, tuple(args))

    def matches(self, ground: Atom) -> bool:
        """Whether this (wildcard-bearing, variable-free) atom matches ``ground``."""
        return (
            self.name == ground.name
            and len(self.args) == len(ground.args)
            and all(a in (WILDCARD, b) for a, b in zip(self.args, ground.args))
        )


@dataclass(frozen=True)
class Literal:
    """An atom that must hold (``positive``) or must not hold."""

    atom: Atom
    positive: bool = True

    @classmethod
    def parse(cls, text: str) -> Literal:
        if not isinstance(text, str):
            raise ValueError(f"literal must be a string, got {text!r}")
        stripped = text.strip()
        positive = True
        if stripped.startswith("not ") or stripped.startswith("not\t"):
            positive = False
            stripped = stripped[3:]
        return cls(parse_atom(stripped, allow_variables=True), positive)

    def __str__(self) -> str:
        return str(self.atom) if self.positive else f"not {self.atom}"

    def variables(self) -> set[str]:
        return self.atom.variables()

    def ground(self, parameters: Mapping[str, Any]) -> Literal:
        return Literal(self.atom.ground(parameters), self.positive)


def parse_atom(text: str, allow_variables: bool = False) -> Atom:
    """Parse ``name(arg, ...)``; ground unless ``allow_variables``."""
    match = _ATOM.match(text) if isinstance(text, str) else None
    if match is None:
        raise ValueError(f"not an atom: {text!r} (expected name(arg, ...))")
    name, body = match.group(1), match.group(2).strip()
    if not _NAME.fullmatch(name):
        raise ValueError(f"{text!r}: predicate name must match [a-z][a-z0-9_]*")
    args: tuple[str, ...] = ()
    if body:
        args = tuple(arg.strip() for arg in body.split(","))
        for arg in args:
            if arg == WILDCARD or _CONSTANT.fullmatch(arg):
                continue
            if _VARIABLE.fullmatch(arg):
                if not allow_variables:
                    raise ValueError(f"{text!r}: variables are not allowed here")
                continue
            raise ValueError(f"{text!r}: invalid argument {arg!r}")
    return Atom(name, args)


def parse_state(atoms: Iterable[Any]) -> frozenset[Atom]:
    """A state from ground atoms given as strings or :class:`Atom`s."""
    state = set()
    for item in atoms:
        atom = item if isinstance(item, Atom) else parse_atom(item)
        if not atom.is_ground or atom.has_wildcard:
            raise ValueError(f"state atoms must be ground, got {atom}")
        state.add(atom)
    return frozenset(state)


def holds(literal: Literal, state: frozenset[Atom]) -> bool:
    """Evaluate a ground literal (wildcards allowed) in a closed-world state."""
    if literal.variables():
        raise ValueError(f"cannot evaluate non-ground literal {literal}")
    found = any(literal.atom.matches(atom) for atom in state)
    return found if literal.positive else not found


def apply_effects(
    state: frozenset[Atom], effects: Iterable[Literal]
) -> frozenset[Atom]:
    """Apply ground effects: all deletions first, then all additions."""
    effects = list(effects)
    kept = {
        atom
        for atom in state
        if not any(not e.positive and e.atom.matches(atom) for e in effects)
    }
    kept.update(e.atom for e in effects if e.positive)
    return frozenset(kept)


def format_state(state: frozenset[Atom]) -> str:
    return "{" + ", ".join(sorted(str(atom) for atom in state)) + "}"
