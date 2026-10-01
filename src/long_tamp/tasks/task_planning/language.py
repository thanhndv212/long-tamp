"""Goals from natural language (#22): a model writes the goal, never the plan.

``goal_from_instruction(instruction, writer, vocabulary, reachable)`` turns an
instruction ("assemble part 2 and put the driver away") into a goal: ground
literals in long_tamp's syntax (``screwed(part2, part2/h_hole1)``,
``not holds(ur10_left/gripper, _)``), which the task planner then plans and the
refiner refines, like any other goal. The design follows ViLaIn-TAMP (arXiv
2506.03270), with two departures:

- the model writes only the goal. The initial state is what the world state
  observes (grasp tracker, recorded facts), so nothing the model makes up is
  mixed with observation;
- motion failures don't go back to the model: the refiner turns them into
  facts and the planner replans around them (``repair``), deterministically.

Each goal the model writes is checked before it is used: syntax, known
predicates and arities, known objects (``check_goal``), then ``reachable``,
whether the task planner finds a plan to it from the current state. What
fails goes back to the model as feedback, for up to ``max_rounds`` attempts.

``ModelGoalWriter`` writes goals with any model through the AI gateway
(``long_tamp.ai``: Anthropic- or OpenAI-compatible APIs, ``<api>:<model>``).
Any object with ``write_goal`` works. Both the goal writer and the grounder
(``ground_instruction``: which scene objects an instruction refers to) are
roles in the sense of ``long_tamp.ai.roles``: proposed, checked, refined.
"""

from __future__ import annotations

import re
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field, replace
from typing import Any, Protocol

from long_tamp.ai.roles import (
    ModelRole,
    Rejected,
    RoleError,
    RoleOutcome,
    Unsupported,
    refine,
)

from .capabilities import CapabilityDescriptor
from .predicates import WILDCARD, Literal, parse_atom


class GoalError(ValueError):
    """No acceptable goal: the instruction can't be expressed, or every
    attempt failed its checks (``attempts`` has each goal and why)."""

    def __init__(self, message: str, attempts: list[dict[str, Any]] | None = None):
        super().__init__(message)
        self.attempts = attempts or []


@dataclass(frozen=True)
class Vocabulary:
    """What a goal may say: predicates (name -> arity), objects, and the
    current state, with optional notes on what the predicates mean."""

    predicates: Mapping[str, int]
    objects: tuple[str, ...]
    state: tuple[str, ...] = ()
    notes: str = ""

    @classmethod
    def from_domain(
        cls,
        descriptors: (
            Mapping[str, CapabilityDescriptor] | Iterable[CapabilityDescriptor]
        ),
        state: Iterable[str],
        objects: Iterable[str] = (),
        notes: str = "",
    ) -> Vocabulary:
        """Predicates from the capabilities' literals and the state; objects
        from the state, ``objects``, and the capabilities' constants."""
        if isinstance(descriptors, Mapping):
            descriptors = descriptors.values()
        predicates: dict[str, int] = {}
        found: set[str] = set(objects)
        atoms = [parse_atom(a) if isinstance(a, str) else a for a in state]
        for atom in atoms:
            predicates.setdefault(atom.name, len(atom.args))
            found.update(a for a in atom.args if a != WILDCARD)
        for descriptor in descriptors:
            for lit in (*descriptor.precondition_literals, *descriptor.effect_literals):
                predicates.setdefault(lit.atom.name, len(lit.atom.args))
                found.update(
                    a for a in lit.atom.args if a != WILDCARD and not a.startswith("?")
                )
        return cls(
            predicates=dict(sorted(predicates.items())),
            objects=tuple(sorted(found)),
            state=tuple(str(a) for a in atoms),
            notes=notes,
        )


def check_goal(goal: Sequence[str], vocabulary: Vocabulary) -> list[str]:
    """Why ``goal`` can't be used (empty: it can): one message per problem."""
    errors = []
    if not goal:
        return ["the goal is empty"]
    objects = set(vocabulary.objects)
    for text in goal:
        try:
            lit = Literal.parse(text)
        except (TypeError, ValueError) as error:
            errors.append(f"{text!r}: not a literal ({error})")
            continue
        name, args = lit.atom.name, lit.atom.args
        if name not in vocabulary.predicates:
            errors.append(f"{text!r}: unknown predicate {name!r}")
            continue
        if len(args) != vocabulary.predicates[name]:
            errors.append(
                f"{text!r}: {name} takes {vocabulary.predicates[name]} arguments"
            )
            continue
        for arg in args:
            if arg == WILDCARD:
                if lit.positive:
                    errors.append(f"{text!r}: '_' is only allowed in negated literals")
            elif arg.startswith("?"):
                errors.append(f"{text!r}: variables are not allowed ({arg})")
            elif arg not in objects:
                errors.append(f"{text!r}: unknown object {arg!r}")
    return errors


class GoalWriter(Protocol):
    def write_goal(
        self, instruction: str, vocabulary: Vocabulary, feedback: Sequence[str]
    ) -> list[str]:
        """The goal for ``instruction``; ``feedback`` says why the previous
        attempts were rejected. Raises ``GoalError`` when the instruction
        can't be expressed with the vocabulary."""


def goal_from_instruction(
    instruction: str,
    writer: GoalWriter,
    vocabulary: Vocabulary,
    reachable: Callable[[list[str]], str | None] | None = None,
    max_rounds: int = 3,
) -> list[str]:
    """A checked goal for ``instruction`` (see the module docstring).

    ``reachable(goal)`` returns ``None`` when the task planner reaches the
    goal from the current state, or why it can't.
    """

    def check(goal: list[str]) -> list[str]:
        errors = check_goal(goal, vocabulary)
        if not errors and reachable is not None:
            why = reachable(goal)
            if why:
                errors = [f"no plan reaches this goal from the current state: {why}"]
        return errors

    try:
        outcome = refine(
            lambda feedback: list(writer.write_goal(instruction, vocabulary, feedback)),
            check,
            max_rounds=max_rounds,
            name="goal",
        )
    except RoleError as error:
        raise GoalError(
            f"no acceptable goal for {instruction!r}: {error}", error.attempts
        ) from error
    return outcome.value


# -- a model through the AI gateway -------------------------------------------

_SYSTEM = """\
You turn a robot operator's instruction into the GOAL of a task-planning problem.
You never write the plan: a task planner finds the actions, and a motion planner \
checks them.

A goal is a list of ground literals that must all hold at the end:
- a positive literal: predicate(arg1, arg2), e.g. screwed(part1, part1/h_hole1)
- a negated literal: not predicate(arg1, arg2); in negated literals only, "_" \
stands for any object, e.g. not holds(ur10_left/gripper, _) means that gripper \
holds nothing
Use only the listed predicates, with their arity, and the listed objects, \
spelled exactly. Write the goal the instruction asks for, nothing more: leave out \
facts the operator didn't ask about, unless the instruction implies them (read the \
notes). Facts that already hold may be part of the goal.
An instruction may also say HOW to do the work (a tool or fixture not to use, \
something to avoid on the way): leave that out of the goal, a plan reviewer turns \
it into constraints. Only if the result the instruction asks for can't be \
expressed with these predicates, return an empty goal and say why in \
"unsupported"."""

_SCHEMA = {
    "type": "object",
    "properties": {
        "goal": {"type": "array", "items": {"type": "string"}},
        "unsupported": {"type": "string"},
    },
    "required": ["goal", "unsupported"],
    "additionalProperties": False,
}


def _parse_goal(answer: Any) -> list[str]:
    if not isinstance(answer, dict):
        raise Rejected(f"the answer must be a JSON object, not {answer!r}")
    if answer.get("unsupported") and not answer.get("goal"):
        raise Unsupported(answer["unsupported"])
    return [str(literal) for literal in answer.get("goal", [])]


def goal_prompt(
    instruction: str, vocabulary: Vocabulary, feedback: Sequence[str]
) -> str:
    """The user message for one attempt."""
    lines = ["Predicates (name/arity):"]
    lines += [f"- {name}/{arity}" for name, arity in vocabulary.predicates.items()]
    lines += ["", "Objects:", *[f"- {o}" for o in vocabulary.objects]]
    lines += ["", "Current state (true now; everything else is false):"]
    lines += [f"- {a}" for a in vocabulary.state] or ["- (nothing)"]
    if vocabulary.notes:
        lines += ["", "Notes on this domain:", vocabulary.notes.strip()]
    if feedback:
        lines += ["", *feedback, "Write a corrected goal."]
    lines += ["", f"Instruction: {instruction}"]
    return "\n".join(lines)


@dataclass
class ModelGoalWriter:
    """Writes goals with a model through the AI gateway (``long_tamp.ai``).

    ``client`` is a ``ModelClient`` (``make_client("<api>:<model>")``); its
    calls are recorded with role ``"goal"``. API failures become
    ``GoalError`` (no point asking again with feedback).
    """

    client: Any
    #: Every prompt and answer, for logs.
    transcript: list[dict[str, Any]] = field(default_factory=list)

    @property
    def model(self) -> str:
        return str(self.client.spec)

    def write_goal(
        self, instruction: str, vocabulary: Vocabulary, feedback: Sequence[str]
    ) -> list[str]:
        propose = GOAL_ROLE.proposer(self.client, (instruction, vocabulary))
        goal = propose(list(feedback))
        self.transcript.append({"feedback": list(feedback), "goal": goal})
        return goal


def goal_writer(model: str | None = None, **client_options: Any) -> ModelGoalWriter:
    """A ``ModelGoalWriter`` for ``model`` (``"<api>:<model>"``; default
    ``anthropic:claude-opus-5-5``); ``client_options`` go to ``make_client``."""
    from long_tamp.ai import make_client

    return ModelGoalWriter(make_client(model, **client_options))


#: The goal writer as a model role (``long_tamp.ai.roles``): the request is
#: ``(instruction, vocabulary)``.
GOAL_ROLE = ModelRole(
    name="goal",
    system=_SYSTEM,
    schema=_SCHEMA,
    render=lambda request, feedback: goal_prompt(request[0], request[1], feedback),
    parse=_parse_goal,
)


# -- grounding: which scene objects an instruction refers to --------------------


@dataclass(frozen=True)
class Grounding:
    """The scene objects an instruction refers to, and the phrases that
    matched none."""

    objects: tuple[str, ...]
    unmatched: tuple[str, ...] = ()


def _spoken_names(objects: Sequence[str]) -> dict[str, list[str]]:
    """How an operator may name objects -> the objects meant: a top-level
    object by its name, a fixture by its own (``fixtures/clamp2`` ->
    ``clamp2``), and an object listed only through its handles by its name
    (``driver`` -> ``driver/h_grip``, ``driver/h_rack``)."""
    names: dict[str, list[str]] = {}
    bare = {o for o in objects if "/" not in o}
    for obj in objects:
        head, _, tail = obj.partition("/")
        if not tail:
            names.setdefault(obj, []).append(obj)
        elif head == "fixtures" and "/" not in tail:
            names.setdefault(tail, []).append(obj)
        elif head not in bare:
            names.setdefault(head, []).append(obj)
    return names


def lexical_grounding(instruction: str, vocabulary: Vocabulary) -> Grounding:
    """The fallback grounder: objects whose name appears in the instruction,
    spaces and case aside ("part 2" -> ``part2``, "screwdriver" -> the
    driver's handles)."""
    text = re.sub(r"[\s_\-]+", "", instruction.lower())
    found: list[str] = []
    for name, meant in _spoken_names(vocabulary.objects).items():
        if re.sub(r"[_\-]", "", name.lower()) in text:
            found += [o for o in meant if o not in found]
    return Grounding(tuple(found))


def check_grounding(grounding: Grounding, vocabulary: Vocabulary) -> list[str]:
    """Why ``grounding`` can't be used (empty: it can)."""
    known = set(vocabulary.objects)
    return [f"unknown object {o!r}" for o in grounding.objects if o not in known]


_GROUND_SYSTEM = """\
You find which objects of a robot cell an operator's instruction refers to.
Return the names of the objects it mentions or clearly implies, spelled exactly as \
listed (prefer whole objects, e.g. part2, over their handles), and the phrases \
that refer to something not in the list."""

_GROUND_SCHEMA = {
    "type": "object",
    "properties": {
        "objects": {"type": "array", "items": {"type": "string"}},
        "unmatched": {"type": "array", "items": {"type": "string"}},
    },
    "required": ["objects", "unmatched"],
    "additionalProperties": False,
}


def _ground_prompt(request: tuple[str, Vocabulary], feedback: Sequence[str]) -> str:
    instruction, vocabulary = request
    lines = ["Objects:", *[f"- {o}" for o in vocabulary.objects]]
    if vocabulary.notes:
        lines += ["", "Notes on this domain:", vocabulary.notes.strip()]
    if feedback:
        lines += ["", *feedback, "Answer again."]
    lines += ["", f"Instruction: {instruction}"]
    return "\n".join(lines)


def _parse_grounding(answer: Any) -> Grounding:
    if not isinstance(answer, dict):
        raise Rejected(f"the answer must be a JSON object, not {answer!r}")
    return Grounding(
        tuple(str(o) for o in answer.get("objects", [])),
        tuple(str(p) for p in answer.get("unmatched", [])),
    )


#: The grounder as a model role: the request is ``(instruction, vocabulary)``.
GROUND_ROLE = ModelRole(
    name="ground",
    system=_GROUND_SYSTEM,
    schema=_GROUND_SCHEMA,
    render=_ground_prompt,
    parse=_parse_grounding,
)


def ground_instruction(
    instruction: str, vocabulary: Vocabulary, client: Any = None, max_rounds: int = 2
) -> RoleOutcome[Grounding]:
    """The objects ``instruction`` refers to: from ``client``'s model, checked
    against the vocabulary, or from ``lexical_grounding`` (no model, or the
    model failed)."""
    fallback = lambda why: lexical_grounding(instruction, vocabulary)  # noqa: E731
    if client is None:
        return RoleOutcome(fallback("no model"), fallback="no model")
    return refine(
        GROUND_ROLE.proposer(client, (instruction, vocabulary)),
        lambda g: check_grounding(g, vocabulary),
        max_rounds=max_rounds,
        fallback=fallback,
        name="ground",
    )


def with_grounding(vocabulary: Vocabulary, grounding: Grounding) -> Vocabulary:
    """``vocabulary`` with a note on what the instruction refers to (for the
    goal writer)."""
    if not grounding.objects and not grounding.unmatched:
        return vocabulary
    note = ""
    if grounding.objects:
        note += "The instruction refers to: " + ", ".join(grounding.objects) + "."
    if grounding.unmatched:
        note += (
            " These phrases match no object in the cell: "
            + ", ".join(grounding.unmatched)
            + "."
        )
    return replace(vocabulary, notes=(vocabulary.notes.rstrip() + "\n" + note).strip())
