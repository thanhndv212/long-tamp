"""An operator's chat with a model that acts through gated tools (ADR-0006).

``ChatSession(client, tools, intro)`` holds a conversation. On each operator
message (``turn``), the model answers with what to say and which tools to
call, as one JSON object::

    {"say": "Planning part 2 first.",
     "actions": [{"tool": "plan", "arguments": {"first": ["screwed(part2, ...)"]}}],
     "done": false}

The session runs the actions in order through their ``Tool`` handlers, and
gives the results back to the model, which may call more tools or answer,
up to ``max_steps`` model calls per operator message.

Tools are the only way the model changes anything, and each one checks its
arguments the way the Python API does: a handler raises ``ToolRejected``
with the reason, which goes back to the model like any result. Tools as
JSON actions, rather than provider-specific function calling, work the same
on every endpoint the gateway reaches, including those that don't enforce
schemas.

``on_tool(call)`` receives every tool call (tool, arguments, ok, result or
error) for the event stream.
"""

from __future__ import annotations

import json
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Any

from .client import AIError, AIOutputError


class ToolRejected(Exception):
    """A tool refused its arguments; the message says why (sent to the model)."""


@dataclass(frozen=True)
class Tool:
    """A tool the model can call: its name, what it does, its arguments
    (name -> description), and the handler that checks and runs it."""

    name: str
    description: str
    handler: Callable[..., Any]
    arguments: Mapping[str, str] = field(default_factory=dict)

    def doc(self) -> str:
        args = "".join(f"\n    - {k}: {v}" for k, v in self.arguments.items())
        return f"- {self.name}: {self.description}" + (args or "\n    (no arguments)")


@dataclass
class ToolCall:
    tool: str
    arguments: dict[str, Any]
    ok: bool
    result: Any = None
    error: str = ""

    def as_text(self) -> str:
        outcome = (
            json.dumps(self.result, default=str)
            if self.ok
            else f"REJECTED: {self.error}"
        )
        return f"{self.tool}({json.dumps(self.arguments, default=str)}) -> {outcome}"


@dataclass
class ChatTurn:
    """One operator message and what came of it."""

    user: str
    say: str
    calls: list[ToolCall] = field(default_factory=list)
    error: str = ""


_SCHEMA = {
    "type": "object",
    "properties": {
        "say": {"type": "string"},
        "actions": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "tool": {"type": "string"},
                    "arguments": {"type": "object"},
                },
                "required": ["tool", "arguments"],
            },
        },
        "done": {"type": "boolean"},
    },
    "required": ["say", "actions", "done"],
}

_RULES = """\
You talk with a robot operator and act only through the tools below. Each answer \
is one JSON object: {"say": text for the operator, "actions": [{"tool": name, \
"arguments": {...}}], "done": true or false}.
- Call tools to do what the operator asks; their results come back to you, and you \
can call more tools or answer. Set "done" to true when you have nothing more to \
call for this message.
- A tool may reject its arguments with a reason: fix them, or tell the operator why \
it can't be done. Never claim something happened unless a tool result says so.
- You never write plan steps: the plan tool asks the task planner.
- Tools that make the robot move (run) are called only when the operator asks to run, \
execute or start in that message: otherwise show the plan and ask.
Tools:
"""


class ChatSession:
    """A conversation with a model acting through ``tools`` (see the module
    docstring)."""

    def __init__(
        self,
        client: Any,
        tools: list[Tool],
        intro: str = "",
        max_steps: int = 4,
        history_turns: int = 6,
        on_tool: Callable[[ToolCall], None] | None = None,
    ) -> None:
        self.client = client
        self.tools = {t.name: t for t in tools}
        self.intro = intro
        self.max_steps = max_steps
        self.history_turns = history_turns
        self.on_tool = on_tool
        self.turns: list[ChatTurn] = []
        self.system = (
            (intro.strip() + "\n\n" if intro else "")
            + _RULES
            + "\n".join(t.doc() for t in self.tools.values())
        )

    def _prompt(self, user: str, results: list[ToolCall]) -> str:
        lines = []
        for past in self.turns[-self.history_turns :]:
            lines.append(f"Operator: {past.user}")
            lines += [f"  tool: {c.as_text()}" for c in past.calls]
            lines.append(f"You: {past.say}")
        lines.append(f"Operator: {user}")
        if results:
            lines.append("Tool results so far for this message:")
            lines += [f"  {c.as_text()}" for c in results]
        return "\n".join(lines)

    def call(self, name: str, arguments: Any) -> ToolCall:
        """Run one tool call through its handler (checked there)."""
        args = dict(arguments) if isinstance(arguments, Mapping) else {}
        tool = self.tools.get(name)
        if tool is None:
            call = ToolCall(
                name,
                args,
                False,
                error=f"no tool {name!r} (tools: {', '.join(self.tools)})",
            )
        else:
            try:
                call = ToolCall(name, args, True, result=tool.handler(**args))
            except ToolRejected as why:
                call = ToolCall(name, args, False, error=str(why))
            except TypeError as error:  # wrong or missing arguments
                call = ToolCall(name, args, False, error=f"bad arguments: {error}")
        if self.on_tool is not None:
            self.on_tool(call)
        return call

    def turn(self, user: str) -> ChatTurn:
        """Answer one operator message (see the module docstring)."""
        turn = ChatTurn(user=user, say="")
        for _ in range(self.max_steps):
            try:
                answer = self.client.complete_json(
                    self.system, self._prompt(user, turn.calls), _SCHEMA, role="chat"
                )
            except AIOutputError as error:
                turn.error = f"unusable answer: {error}"
                continue
            except AIError as error:
                turn.error = f"{type(error).__name__}: {error}"
                break
            if not isinstance(answer, dict):
                turn.error = "the answer was not a JSON object"
                continue
            turn.say = str(answer.get("say", "")) or turn.say
            actions = answer.get("actions") or []
            if isinstance(actions, dict):
                actions = [actions]
            for action in actions if isinstance(actions, list) else []:
                if isinstance(action, dict):
                    turn.calls.append(
                        self.call(
                            str(action.get("tool", "")), action.get("arguments") or {}
                        )
                    )
            if answer.get("done", not actions) or not actions:
                turn.error = ""
                break
        self.turns.append(turn)
        return turn
