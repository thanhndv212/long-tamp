"""Canonical, validated intermediate representation for task plans."""

from __future__ import annotations

import hashlib
import json
import math
import re
import unicodedata
from copy import deepcopy
from dataclasses import dataclass
from typing import Any

from .capabilities import CapabilityDescriptor, CapabilityRegistry
from .predicates import (
    Atom,
    Literal,
    apply_effects,
    format_state,
    holds,
    parse_state,
)

_ID_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_.-]{0,127}$")
_SUPPORTED_NODE_TYPES = {
    "sequence",
    "fallback",
    "retry",
    "condition",
    "operation",
    "transaction",
}


class PlanValidationError(ValueError):
    """Raised when task-plan IR violates its schema or capability policy."""


def _normalize(value: Any) -> Any:
    if isinstance(value, str):
        return unicodedata.normalize("NFC", value)
    if isinstance(value, float) and not math.isfinite(value):
        raise PlanValidationError("numbers must be finite")
    if isinstance(value, list):
        return [_normalize(item) for item in value]
    if isinstance(value, dict):
        return {key: _normalize(value[key]) for key in sorted(value)}
    return value


def _canonical_json(value: Any) -> str:
    return json.dumps(
        _normalize(value),
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    )


@dataclass(frozen=True)
class TaskPlan:
    """Normalized task plan and its semantic fingerprint."""

    _document: dict[str, Any]
    canonical_json: str
    plan_fingerprint: str
    effective_attempts: dict[str, int]

    @property
    def document(self) -> dict[str, Any]:
        """Return a defensive copy of the validated normalized IR."""
        return deepcopy(self._document)

    @classmethod
    def from_dict(
        cls, document: dict[str, Any], registry: CapabilityRegistry
    ) -> TaskPlan:
        normalized = _normalize(document)
        if normalized.get("schema_version") != "1.0":
            raise PlanValidationError("unsupported schema_version")
        cls._validate_id(normalized.get("mission_id"), "mission_id")
        if not isinstance(normalized.get("scene"), dict):
            raise PlanValidationError("scene must be an object")
        if not isinstance(normalized.get("provenance"), dict):
            raise PlanValidationError("provenance must be an object")

        seen: set[str] = set()
        effective_attempts: dict[str, int] = {}
        cls._validate_node(normalized.get("root"), registry, seen, effective_attempts)
        if "initial_state" in normalized:
            try:
                initial = parse_state(normalized["initial_state"])
            except (TypeError, ValueError) as error:
                raise PlanValidationError(f"invalid initial_state: {error}") from error
            simulate_plan(normalized["root"], registry, initial)
        canonical = _canonical_json(normalized)
        fingerprint_input = {
            "domain": "agimus-task-plan-v1",
            "plan": normalized,
            "registry": registry.snapshot(),
        }
        fingerprint = hashlib.sha256(
            _canonical_json(fingerprint_input).encode("utf-8")
        ).hexdigest()
        return cls(normalized, canonical, fingerprint, effective_attempts)

    @staticmethod
    def _validate_id(value: Any, field: str) -> None:
        if not isinstance(value, str) or not _ID_PATTERN.fullmatch(value):
            raise PlanValidationError(f"invalid {field}")

    @classmethod
    def _validate_node(
        cls,
        node: Any,
        registry: CapabilityRegistry,
        seen: set[str],
        effective_attempts: dict[str, int],
    ) -> None:
        if not isinstance(node, dict):
            raise PlanValidationError("plan node must be an object")
        node_type = node.get("type")
        if node_type not in _SUPPORTED_NODE_TYPES:
            raise PlanValidationError(f"unsupported node type: {node_type}")
        node_id = node.get("id")
        cls._validate_id(node_id, "node id")
        if node_id in seen:
            raise PlanValidationError(f"duplicate node id: {node_id}")
        seen.add(node_id)

        if node_type in {"sequence", "fallback", "transaction"}:
            children = node.get("children")
            if not isinstance(children, list):
                raise PlanValidationError(f"{node_type} requires children")
            if node_type == "transaction":
                if not isinstance(node.get("restart_state"), list):
                    raise PlanValidationError("transaction requires restart_state")
                if len(children) != 1 or children[0].get("type") != "operation":
                    raise PlanValidationError(
                        "transaction requires exactly one executable operation"
                    )
            for child in children:
                cls._validate_node(child, registry, seen, effective_attempts)
            if node_type == "transaction":
                child = children[0]
                descriptor = registry.descriptor(child["capability"])
                if not descriptor.restartable:
                    raise PlanValidationError(
                        "transaction capability must be restartable"
                    )
                effective_attempts[node_id] = effective_attempts[child["id"]]
            return

        if node_type == "retry":
            if not isinstance(node.get("child"), dict):
                raise PlanValidationError("retry requires child")
            cls._validate_node(node["child"], registry, seen, effective_attempts)
            max_attempts = node.get("max_attempts")
            if (
                isinstance(max_attempts, bool)
                or not isinstance(max_attempts, int)
                or max_attempts < 1
            ):
                raise PlanValidationError("retry requires positive max_attempts")
            child_attempts = effective_attempts.get(node["child"]["id"])
            if child_attempts is not None and max_attempts > child_attempts:
                raise PlanValidationError("retry exceeds child capability limit")
            effective_attempts[node_id] = max_attempts
            return

        capability_id = node.get("capability")
        try:
            descriptor = registry.descriptor(capability_id)
        except KeyError as error:
            raise PlanValidationError(str(error)) from error
        parameters = node.get("parameters", {})
        if not isinstance(parameters, dict):
            raise PlanValidationError("parameters must be an object")
        for name, expected_type in descriptor.required_parameters.items():
            if name not in parameters or not isinstance(
                parameters[name], expected_type
            ):
                raise PlanValidationError(
                    f"parameter {name} must be {expected_type.__name__}"
                )
        constraints = node.get("constraints", {})
        if not isinstance(constraints, dict):
            raise PlanValidationError("constraints must be an object")
        max_attempts = constraints.get("max_attempts", descriptor.max_attempts)
        if isinstance(max_attempts, bool) or not isinstance(max_attempts, int):
            raise PlanValidationError("max_attempts must be an integer")
        if max_attempts < 1:
            raise PlanValidationError("max_attempts must be positive")
        if max_attempts > descriptor.max_attempts:
            raise PlanValidationError(
                f"max_attempts {max_attempts} exceeds capability limit "
                f"{descriptor.max_attempts}"
            )
        effective_attempts[node_id] = max_attempts
        max_timeout = constraints.get("max_timeout", descriptor.max_timeout)
        if max_timeout > descriptor.max_timeout:
            raise PlanValidationError(
                f"max_timeout {max_timeout} exceeds capability limit "
                f"{descriptor.max_timeout}"
            )
        try:
            grounded_literals(node, descriptor)
        except ValueError as error:
            raise PlanValidationError(f"step {node_id}: {error}") from error


# Fallbacks multiply the reachable states; plans are small, so a cap only
# guards against pathological documents.
_MAX_REACHABLE_STATES = 1024


def grounded_literals(
    node: dict[str, Any], descriptor: CapabilityDescriptor
) -> tuple[list[Literal], list[Literal]]:
    """A condition/operation node's preconditions and effects, grounded."""
    parameters = node.get("parameters", {})
    return (
        [lit.ground(parameters) for lit in descriptor.precondition_literals],
        [lit.ground(parameters) for lit in descriptor.effect_literals],
    )


def simulate_plan(
    root: dict[str, Any],
    registry: CapabilityRegistry,
    initial: frozenset[Atom],
) -> None:
    """Symbolically execute every branch of a plan from ``initial``.

    Raises ``PlanValidationError`` if some operation can be reached in a state
    where one of its preconditions is false, or if the plan can never succeed.
    A step that fails is assumed to leave the state unchanged (the transaction
    contract); a condition whose capability declares preconditions succeeds
    exactly when they hold, otherwise both outcomes are explored.
    """
    successes, _ = _simulate(root, registry, {initial})
    if not successes:
        raise PlanValidationError(
            f"plan can never succeed from initial_state {format_state(initial)}"
        )


def _simulate(
    node: dict[str, Any],
    registry: CapabilityRegistry,
    states: set[frozenset[Atom]],
) -> tuple[set[frozenset[Atom]], set[frozenset[Atom]]]:
    """Return the (success, failure) states reachable after ``node``."""
    if len(states) > _MAX_REACHABLE_STATES:
        raise PlanValidationError(
            f"too many reachable states to verify at {node['id']} "
            f"(> {_MAX_REACHABLE_STATES})"
        )
    node_type = node["type"]
    if node_type == "transaction":
        return _simulate(node["children"][0], registry, states)
    if node_type == "retry":
        return _simulate(node["child"], registry, states)
    if node_type == "sequence":
        current, failures = set(states), set()
        for child in node["children"]:
            if not current:
                break
            current, failed = _simulate(child, registry, current)
            failures |= failed
        return current, failures
    if node_type == "fallback":
        current, successes = set(states), set()
        for child in node["children"]:
            if not current:
                break
            succeeded, current = _simulate(child, registry, current)
            successes |= succeeded
        return successes, current

    descriptor = registry.descriptor(node["capability"])
    preconditions, effects = grounded_literals(node, descriptor)
    if node_type == "condition":
        if not preconditions:
            return set(states), set(states)
        successes = {s for s in states if all(holds(p, s) for p in preconditions)}
        return successes, set(states) - successes

    for state in states:
        for literal in preconditions:
            if not holds(literal, state):
                raise PlanValidationError(
                    f"step {node['id']}: precondition {literal} does not hold in "
                    f"reachable state {format_state(state)}"
                )
    return {apply_effects(s, effects) for s in states}, set(states)
