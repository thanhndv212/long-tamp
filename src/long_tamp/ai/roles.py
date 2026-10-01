"""Typed model roles (ADR-0006): a model proposes, long_tamp checks.

Every role has the same shape:

- an **input** (the request: an instruction, a vocabulary, a plan, …);
- an **output contract**: a JSON schema the model answers in, and a parser
  that turns the answer into a typed value (or rejects it);
- a deterministic **checker**: why a proposal can't be used (empty: it can);
- a **fallback** that needs no model, used when the model can't help (it
  can't be reached or refuses, the request can't be expressed, or every
  round is rejected); a role without one fails instead. An unusable answer
  (not JSON, cut off, malformed) is rejected like a failed check: it costs a
  round, with feedback.

``refine`` runs the loop: propose, check, and send the checker's reasons back
to the proposer, for at most ``max_rounds`` rounds. ``ModelRole`` makes a
proposer from a model client (``long_tamp.ai``) for any role; scripted
proposers (tests, replays) plug in the same way.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Any, Generic, TypeVar

from .client import AIError, AIOutputError

T = TypeVar("T")


class RoleError(RuntimeError):
    """A role produced no usable result and has no fallback."""

    def __init__(self, message: str, attempts: list[dict[str, Any]] | None = None):
        super().__init__(message)
        self.attempts = attempts or []


class Unsupported(Exception):
    """The proposer says the request can't be expressed (no point retrying)."""


class Rejected(ValueError):
    """A proposer's answer can't be parsed into the role's output type; the
    message is fed back like a checker's reasons."""


@dataclass
class RoleOutcome(Generic[T]):
    """A role's result, how many attempts it took, and whether (and why) the
    fallback was used instead of a proposal."""

    value: T
    attempts: list[dict[str, Any]] = field(default_factory=list)
    fallback: str | None = None


def refine(
    propose: Callable[[list[str]], T],
    check: Callable[[T], list[str]],
    max_rounds: int = 3,
    fallback: Callable[[str], T] | None = None,
    name: str = "role",
) -> RoleOutcome[T]:
    """Propose, check, and feed the checker's reasons back, at most
    ``max_rounds`` times (see the module docstring)."""
    feedback: list[str] = []
    attempts: list[dict[str, Any]] = []

    def give_up(reason: str, cause: Exception | None = None) -> RoleOutcome[T]:
        if fallback is None:
            error = RoleError(f"{name}: {reason}", attempts)
            if cause is not None:
                raise error from cause
            raise error
        return RoleOutcome(fallback(reason), attempts, reason)

    for _ in range(max_rounds):
        try:
            proposal = propose(feedback)
        except Unsupported as why:
            attempts.append({"unsupported": str(why)})
            return give_up(f"can't be expressed: {why}", why)
        except (Rejected, AIOutputError) as why:
            # An answer we can't use (not JSON, cut off, wrong shape): say why
            # and ask again; the model was reached, so another round may do.
            attempts.append({"errors": [str(why)]})
            feedback = [
                "Your answer was rejected:",
                str(why),
                "Answer with the JSON object only.",
            ]
            continue
        except AIError as error:
            attempts.append({"error": f"{type(error).__name__}: {error}"})
            return give_up(f"{type(error).__name__}: {error}", error)
        errors = check(proposal)
        attempts.append({"proposal": proposal, "errors": errors})
        if not errors:
            return RoleOutcome(proposal, attempts)
        feedback = [f"Your answer {proposal!r} was rejected:", *errors]
    return give_up(f"no acceptable answer in {max_rounds} attempts")


@dataclass
class ModelRole(Generic[T]):
    """A role answered by a model: its instructions, its answer's JSON
    schema, how a request (plus feedback) becomes the user message, and how
    an answer becomes the output (raising ``Rejected`` or ``Unsupported``)."""

    name: str
    system: str
    schema: dict[str, Any]
    render: Callable[[Any, Sequence[str]], str]
    parse: Callable[[Any], T]

    def proposer(self, client: Any, request: Any) -> Callable[[list[str]], T]:
        """A proposer for ``refine``: one model call per round."""

        def propose(feedback: list[str]) -> T:
            answer = client.complete_json(
                self.system, self.render(request, feedback), self.schema, role=self.name
            )
            return self.parse(answer)

        return propose
