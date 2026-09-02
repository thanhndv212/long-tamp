"""Trusted capability descriptors for task-plan validation and dispatch."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping


@dataclass(frozen=True)
class CapabilityDescriptor:
    """Immutable policy and type contract for one executable capability."""

    capability_id: str
    version: str
    required_parameters: Mapping[str, type]
    resources: tuple[str, ...] = ()
    effects: tuple[str, ...] = ()
    safety_class: str = "planning-only"
    max_attempts: int = 1
    max_timeout: float = 30.0
    restartable: bool = False

    def snapshot(self) -> dict[str, Any]:
        return {
            "capability_id": self.capability_id,
            "version": self.version,
            "required_parameters": {
                key: value.__name__
                for key, value in sorted(self.required_parameters.items())
            },
            "resources": list(self.resources),
            "effects": list(self.effects),
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
