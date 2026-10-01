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

``ClaudeGoalWriter`` writes goals with Claude (``pip install long-tamp[language]``);
any object with ``write_goal`` works.
"""

from __future__ import annotations

import json
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, Protocol

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
    feedback: list[str] = []
    attempts: list[dict[str, Any]] = []
    for _ in range(max_rounds):
        goal = list(writer.write_goal(instruction, vocabulary, feedback))
        errors = check_goal(goal, vocabulary)
        if not errors and reachable is not None:
            why = reachable(goal)
            if why:
                errors = [f"no plan reaches this goal from the current state: {why}"]
        attempts.append({"goal": goal, "errors": errors})
        if not errors:
            return goal
        feedback = [f"Your goal {goal} was rejected:", *errors]
    raise GoalError(
        f"no acceptable goal for {instruction!r} in {max_rounds} attempts", attempts
    )


# -- Claude ----------------------------------------------------------------

#: The model, unless the caller names another.
DEFAULT_MODEL = "claude-opus-5-5"

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
If the instruction asks for something these predicates can't express, return an \
empty goal and say why in "unsupported"."""

_SCHEMA = {
    "type": "object",
    "properties": {
        "goal": {"type": "array", "items": {"type": "string"}},
        "unsupported": {"type": "string"},
    },
    "required": ["goal", "unsupported"],
    "additionalProperties": False,
}


@dataclass
class ClaudeGoalWriter:
    """Writes goals with Claude through the Anthropic API.

    Needs ``pip install long-tamp[language]`` and credentials the SDK can
    find (``ANTHROPIC_API_KEY``, or an ``ant auth login`` profile). Requests
    use structured output (a JSON goal) and the API's server-side fallback
    when the model declines (``fallbacks="default"``).
    """

    model: str = DEFAULT_MODEL
    effort: str = "medium"
    max_tokens: int = 16000
    client: Any = None
    #: Every request and response text, for logs.
    transcript: list[dict[str, Any]] = field(default_factory=list)

    def _client(self) -> Any:
        if self.client is None:
            try:
                import anthropic
            except ImportError as error:  # pragma: no cover - depends on the extra
                raise ImportError(
                    "ClaudeGoalWriter needs the Anthropic SDK: "
                    "pip install long-tamp[language]"
                ) from error
            self.client = anthropic.Anthropic()
        return self.client

    @staticmethod
    def prompt(
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

    def write_goal(
        self, instruction: str, vocabulary: Vocabulary, feedback: Sequence[str]
    ) -> list[str]:
        content = self.prompt(instruction, vocabulary, feedback)
        response = self._client().beta.messages.create(
            model=self.model,
            max_tokens=self.max_tokens,
            system=_SYSTEM,
            messages=[{"role": "user", "content": content}],
            output_config={
                "effort": self.effort,
                "format": {"type": "json_schema", "schema": _SCHEMA},
            },
            betas=["server-side-fallback-2026-07-01"],
            fallbacks="default",
        )
        if response.stop_reason == "refusal":
            details = getattr(response, "stop_details", None)
            raise GoalError(
                "the model declined the instruction"
                + (f" ({details.category})" if details and details.category else "")
            )
        if response.stop_reason == "max_tokens":
            raise GoalError("the model's answer was cut off (max_tokens)")
        text = next(b.text for b in response.content if b.type == "text")
        self.transcript.append({"prompt": content, "response": text})
        answer = json.loads(text)
        if answer.get("unsupported") and not answer.get("goal"):
            raise GoalError(
                f"the instruction can't be expressed: {answer['unsupported']}"
            )
        return [str(literal) for literal in answer["goal"]]
