"""The plan reviewer (ADR-0006): constraints from an instruction, never steps.

An instruction says *what* ("assemble parts 1 and 2") and sometimes *how*
("... but don't use clamp 1", "keep the right arm off part 2"). A goal can't
say how; the task planner's blocked bindings can (``to_pddl(blocked=...)``):
a capability with some of its parameters, which no plan step may match.

``review_constraints`` asks a model for the constraints an instruction
imposes, as blocked bindings, and checks each proposal:

- the capability exists, its parameters are the capability's own (not
  ``block``), and its values are objects of the cell;
- ``replannable(blocked)``: the task planner still reaches the goal with
  them. A constraint that makes the mission impossible goes back to the
  model, and in the end isn't applied.

Constraints only remove options: the planner chooses what remains, and the
refiner checks it. The fallback (no model, or no acceptable answer) is no
constraint at all.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from long_tamp.ai.roles import ModelRole, Rejected, RoleOutcome, refine

from .capabilities import CapabilityDescriptor
from .language import Vocabulary

#: A blocked binding: a capability, and the parameter values it must not take.
Blocked = tuple[str, dict[str, str]]

#: Parameters that name a step, not a choice: never part of a constraint.
_IGNORED_PARAMETERS = frozenset({"block"})


@dataclass(frozen=True)
class ReviewRequest:
    """What the reviewer sees: the instruction, the cell, the capabilities,
    and the plan the planner found without constraints (labels)."""

    instruction: str
    vocabulary: Vocabulary
    capabilities: Mapping[str, CapabilityDescriptor]
    plan: tuple[str, ...] = ()


def check_constraints(
    blocked: Sequence[Blocked],
    request: ReviewRequest,
    replannable: Callable[[list[Blocked]], str | None] | None = None,
) -> list[str]:
    """Why ``blocked`` can't be applied (empty: it can)."""
    errors = []
    objects = set(request.vocabulary.objects)
    for capability, binding in blocked:
        descriptor = request.capabilities.get(capability)
        if descriptor is None:
            errors.append(f"unknown capability {capability!r}")
            continue
        allowed = set(descriptor.required_parameters) - _IGNORED_PARAMETERS
        if not binding:
            errors.append(f"{capability}: a constraint needs at least one parameter")
        for name, value in binding.items():
            if name not in allowed:
                errors.append(
                    f"{capability}: no parameter {name!r} (it has {', '.join(sorted(allowed))})"
                )
            elif value not in objects:
                errors.append(f"{capability}: unknown object {value!r} for {name}")
    if not errors and blocked and replannable is not None:
        why = replannable(list(blocked))
        if why:
            errors.append(f"with these constraints no plan reaches the goal: {why}")
    return errors


_SYSTEM = """\
You review a robot task plan against the operator's instruction. A task planner \
chose the steps; you never write steps. Your only power is to FORBID choices the \
instruction rules out: each constraint names a capability and some of its \
parameters, and no step may use that capability with those values.
Example: "don't use clamp 1" forbids clamp_and_screw with clamp = fixtures/clamp1.
Only forbid what the instruction asks to avoid, explicitly or clearly. If it \
imposes nothing on how the work is done, return no constraint. Use only the listed \
capabilities, their parameter names, and the listed objects, spelled exactly."""

_SCHEMA = {
    "type": "object",
    "properties": {
        "avoid": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "capability": {"type": "string"},
                    "parameters": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "name": {"type": "string"},
                                "value": {"type": "string"},
                            },
                            "required": ["name", "value"],
                            "additionalProperties": False,
                        },
                    },
                },
                "required": ["capability", "parameters"],
                "additionalProperties": False,
            },
        },
        "reason": {"type": "string"},
    },
    "required": ["avoid", "reason"],
    "additionalProperties": False,
}


def _prompt(request: ReviewRequest, feedback: Sequence[str]) -> str:
    lines = ["Capabilities (parameters):"]
    for cap_id, descriptor in request.capabilities.items():
        params = [
            p for p in descriptor.required_parameters if p not in _IGNORED_PARAMETERS
        ]
        lines.append(f"- {cap_id}({', '.join(params)})")
    lines += ["", "Objects:", *[f"- {o}" for o in request.vocabulary.objects]]
    if request.vocabulary.notes:
        lines += ["", "Notes on this domain:", request.vocabulary.notes.strip()]
    if request.plan:
        lines += ["", "The plan found without constraints:"]
        lines += [f"{i}. {step}" for i, step in enumerate(request.plan, 1)]
    if feedback:
        lines += ["", *feedback, "Answer again."]
    lines += ["", f"Instruction: {request.instruction}"]
    return "\n".join(lines)


_EXAMPLE = (
    '{"avoid": [{"capability": "clamp_and_screw", "parameters": '
    '[{"name": "clamp", "value": "fixtures/clamp1"}]}], "reason": "..."}'
)


def _binding(item: dict[str, Any]) -> dict[str, str]:
    """A constraint's parameters, in any of the shapes models write them:
    ``parameters`` as name/value pairs or a mapping, ``binding``, or the
    parameters flattened next to ``capability``."""
    params = item.get("parameters", item.get("binding"))
    if isinstance(params, list):
        return {str(p["name"]): str(p["value"]) for p in params}
    if isinstance(params, dict):
        return {str(k): str(v) for k, v in params.items()}
    return {
        str(k): str(v)
        for k, v in item.items()
        if k not in ("capability", "parameters", "binding", "reason")
    }


def _parse(answer: Any) -> list[Blocked]:
    """The answer's constraints. The shape is checked loosely (gateways
    don't all enforce the schema); the checker checks every name."""
    items = answer.get("avoid") if isinstance(answer, dict) else answer
    if isinstance(items, dict):
        items = [items]
    if not isinstance(items, list):
        raise Rejected(f"answer with a JSON object like {_EXAMPLE}")
    blocked = []
    for item in items:
        if not isinstance(item, dict) or "capability" not in item:
            raise Rejected(f"malformed constraint {item!r}; write it like {_EXAMPLE}")
        try:
            blocked.append((str(item["capability"]), _binding(item)))
        except (KeyError, TypeError) as error:
            raise Rejected(
                f"malformed constraint {item!r} ({error}); write it like {_EXAMPLE}"
            ) from error
    return blocked


#: The plan reviewer as a model role: the request is a ``ReviewRequest``.
REVIEW_ROLE = ModelRole(
    name="review", system=_SYSTEM, schema=_SCHEMA, render=_prompt, parse=_parse
)


def review_constraints(
    request: ReviewRequest,
    client: Any = None,
    replannable: Callable[[list[Blocked]], str | None] | None = None,
    max_rounds: int = 2,
) -> RoleOutcome[list[Blocked]]:
    """The constraints ``request.instruction`` imposes, checked (see the
    module docstring); none without a model or an acceptable answer."""
    if client is None:
        return RoleOutcome([], fallback="no model")
    return refine(
        REVIEW_ROLE.proposer(client, request),
        lambda blocked: check_constraints(blocked, request, replannable),
        max_rounds=max_rounds,
        fallback=lambda why: [],
        name="review",
    )
