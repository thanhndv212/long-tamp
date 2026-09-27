"""Trusted capability descriptors for task-plan validation and dispatch."""

from __future__ import annotations

import re
import warnings
from dataclasses import dataclass
from typing import Any, Callable, Mapping

from .predicates import WILDCARD, Literal

_BARE_TAG = re.compile(r"[A-Za-z_][A-Za-z0-9_.-]*")


@dataclass(frozen=True)
class CapabilityDescriptor:
    """Immutable policy and type contract for one executable capability.

    ``preconditions`` and ``effects`` are literals over the step's parameters
    (see ``predicates``), e.g. ``preconditions=("not holds(?gripper, _)",)`` and
    ``effects=("holds(?gripper, ?handle)",)``. For a capability used as a
    ``condition``, the preconditions are what it tests. ``writes`` lists the
    parts of the state the capability touches (``"grasp_state"``,
    ``"robot_pose"``); before 0.2 those tags were passed as ``effects``, which
    still works with a ``DeprecationWarning``.
    """

    capability_id: str
    version: str
    required_parameters: Mapping[str, type]
    resources: tuple[str, ...] = ()
    effects: tuple[str, ...] = ()
    safety_class: str = "planning-only"
    max_attempts: int = 1
    max_timeout: float = 30.0
    restartable: bool = False
    preconditions: tuple[str, ...] = ()
    writes: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        effects, tags = [], list(self.writes)
        for item in self.effects:
            if isinstance(item, str) and _BARE_TAG.fullmatch(item.strip()):
                tags.append(item.strip())
            else:
                effects.append(item)
        if len(tags) > len(self.writes):
            warnings.warn(
                f"{self.capability_id}: effects={tuple(tags[len(self.writes):])!r} "
                "are state tags, not literals; pass them as writes= instead "
                "(effects now declare literals such as 'holds(?gripper, ?handle)')",
                DeprecationWarning,
                stacklevel=3,
            )
        preconditions = self._literals(self.preconditions, "precondition")
        effect_literals = self._literals(effects, "effect")
        for literal in effect_literals:
            if literal.positive and WILDCARD in literal.atom.args:
                raise ValueError(
                    f"{self.capability_id}: positive effect {literal} cannot use "
                    "a wildcard"
                )
        object.__setattr__(
            self, "preconditions", tuple(str(lit) for lit in preconditions)
        )
        object.__setattr__(self, "effects", tuple(str(lit) for lit in effect_literals))
        object.__setattr__(self, "writes", tuple(tags))

    def _literals(self, texts: Any, kind: str) -> list[Literal]:
        if isinstance(texts, str):
            raise ValueError(
                f"{self.capability_id}: {kind}s must be a tuple of strings"
            )
        literals = []
        for text in texts:
            try:
                literal = Literal.parse(text)
            except ValueError as error:
                raise ValueError(f"{self.capability_id}: {kind} {error}") from error
            unknown = literal.variables() - set(self.required_parameters)
            if unknown:
                raise ValueError(
                    f"{self.capability_id}: {kind} {literal} uses unknown "
                    f"parameter(s) {sorted(unknown)}"
                )
            literals.append(literal)
        return literals

    @property
    def precondition_literals(self) -> tuple[Literal, ...]:
        return tuple(Literal.parse(text) for text in self.preconditions)

    @property
    def effect_literals(self) -> tuple[Literal, ...]:
        return tuple(Literal.parse(text) for text in self.effects)

    def snapshot(self) -> dict[str, Any]:
        return {
            "capability_id": self.capability_id,
            "version": self.version,
            "required_parameters": {
                key: value.__name__
                for key, value in sorted(self.required_parameters.items())
            },
            "resources": list(self.resources),
            "preconditions": list(self.preconditions),
            "effects": list(self.effects),
            "writes": list(self.writes),
            "safety_class": self.safety_class,
            "max_attempts": self.max_attempts,
            "max_timeout": self.max_timeout,
            "restartable": self.restartable,
        }


class CapabilityRegistry:
    """Registry that binds public descriptors to private trusted callables."""

    def __init__(self) -> None:
        self._entries: dict[str, tuple[CapabilityDescriptor, Callable[..., Any]]] = {}
        self._frozen = False

    def register(
        self, descriptor: CapabilityDescriptor, implementation: Callable[..., Any]
    ) -> None:
        if self._frozen:
            raise RuntimeError("capability registry is frozen")
        if descriptor.capability_id in self._entries:
            raise ValueError(
                f"capability already registered: {descriptor.capability_id}"
            )
        self._entries[descriptor.capability_id] = (descriptor, implementation)

    def descriptor(self, capability_id: str) -> CapabilityDescriptor:
        try:
            return self._entries[capability_id][0]
        except KeyError as error:
            raise KeyError(f"unknown capability: {capability_id}") from error

    def implementation(self, capability_id: str) -> Callable[..., Any]:
        try:
            return self._entries[capability_id][1]
        except KeyError as error:
            raise KeyError(f"unknown capability: {capability_id}") from error

    def bind(self, capability_id: str, implementation: Callable[..., Any]) -> None:
        """Replace only the trusted callable while preserving descriptor policy."""
        if self._frozen:
            raise RuntimeError("capability registry is frozen")
        descriptor = self.descriptor(capability_id)
        self._entries[capability_id] = (descriptor, implementation)

    def freeze(self) -> None:
        """Prevent capability registration or rebinding before execution."""
        self._frozen = True

    def snapshot(self) -> list[dict[str, Any]]:
        return [self._entries[key][0].snapshot() for key in sorted(self._entries)]
