"""The chat panel's back end (#90): a ``ChatSession`` driven from the viewer.

``ChatBridge(session).attach(server)`` adds two routes to a ``ViewerServer``:

- ``POST /api/chat`` ``{"message": text}`` starts a turn and returns at once
  (a turn may run the robot for minutes); ``"quit"`` or ``"exit"`` ends the
  chat;
- ``GET /api/chat?since=N`` the transcript from entry ``N``, and whether a
  turn is running.

- ``POST /api/action`` ``{"name": name}`` runs an operator action
  (``add_action``): a button on the page that acts without the model, such
  as "Start mission" running the plan the chat built (#104).

One turn (or action) runs at a time, whoever sends it: a terminal front end
calls ``turn()`` on the same bridge, and the page shows its turns too. A
tool result with a ``plan`` list of step labels also shows as a plan card. The tools'
calls reach the event stream through the session's ``on_tool``, as in the
terminal chat, so their results show in the event list and, for a new plan,
in the plan tree (a ``plan`` event).
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

QUIT = ("quit", "exit")


@dataclass
class Action:
    """An operator action: a page button that runs ``run()`` without the
    model, when ``enabled()``. ``run`` may return a ``ToolCall`` (shown as
    one), or anything JSON-like (shown as text)."""

    name: str
    label: str
    run: Callable[[], Any]
    enabled: Callable[[], bool]


class ChatBridge:
    def __init__(self, session: Any) -> None:
        self.session = session
        self.ended = threading.Event()
        self._turn_lock = threading.Lock()
        self._log_lock = threading.Lock()
        self._log: list[dict[str, Any]] = []
        self._worker: threading.Thread | None = None
        self.actions: dict[str, Action] = {}
        # A ChatSession reports each message and tool call as it comes: the
        # page shows the turn as it unfolds, not only once it is over.
        self._live = hasattr(session, "on_tool") and hasattr(session, "on_message")
        if self._live:
            tool, message = session.on_tool, session.on_message

            def on_tool(call: Any) -> None:
                if tool is not None:
                    tool(call)
                self._tool(call)

            def on_message(say: str) -> None:
                if message is not None:
                    message(say)
                self._add("model", say)

            session.on_tool, session.on_message = on_tool, on_message

    @property
    def busy(self) -> bool:
        return self._turn_lock.locked()

    def attach(self, server: Any) -> ChatBridge:
        server.route("POST", "/api/chat", self._post)
        server.route("GET", "/api/chat", self._get)
        server.route("POST", "/api/action", self._post_action)
        return self

    def add_action(
        self,
        name: str,
        label: str,
        run: Callable[[], Any],
        enabled: Callable[[], bool] | None = None,
    ) -> ChatBridge:
        self.actions[name] = Action(name, label, run, enabled or (lambda: True))
        return self

    def act(self, name: str, source: str = "web") -> None:
        """Start action ``name`` in the background; ``ValueError`` if it is
        unknown, not available now, or a turn is running."""
        action = self.actions.get(name)
        if action is None:
            raise ValueError(f"no action {name!r}")
        if self.ended.is_set():
            raise ValueError("the chat has ended")
        if not self._turn_lock.acquire(blocking=False):
            raise ValueError("wait: a message or an action is still running")
        if not action.enabled():
            self._turn_lock.release()
            raise ValueError(f"{action.label}: not available now")

        def work() -> None:
            try:
                self._add("operator", action.label, source=source, action=name)
                try:
                    result = action.run()
                except Exception as error:  # noqa: BLE001 - shown
                    self._add("error", f"{type(error).__name__}: {error}")
                    return
                if hasattr(result, "as_text"):
                    if not self._live:  # else on_tool showed it
                        self._tool(result)
                elif result is not None:
                    self._add("system", str(result))
            finally:
                self._turn_lock.release()

        self._worker = threading.Thread(target=work, name="chat-action", daemon=True)
        self._worker.start()

    def turn(self, text: str, source: str = "terminal") -> Any:
        """Run one operator message to its end (blocks while another runs)."""
        with self._turn_lock:
            return self._turn(text, source)

    def submit(self, text: str, source: str = "web") -> None:
        """Start a turn in the background; ``ValueError`` while one runs."""
        text = text.strip()
        if not text:
            raise ValueError("the message is empty")
        if text.lower() in QUIT:
            self._add("system", "chat ended", source=source)
            self.ended.set()
            return
        if self.ended.is_set():
            raise ValueError("the chat has ended")
        if not self._turn_lock.acquire(blocking=False):
            raise ValueError("the model is still answering the last message")

        def work() -> None:
            try:
                self._turn(text, source)
            finally:
                self._turn_lock.release()

        self._worker = threading.Thread(target=work, name="chat-turn", daemon=True)
        self._worker.start()

    def wait(self, timeout: float | None = None) -> bool:
        """Wait for the background turn, if any; ``False`` on timeout."""
        worker = self._worker
        if worker is not None:
            worker.join(timeout)
            return not worker.is_alive()
        return True

    def transcript(self, since: int = 0) -> list[dict[str, Any]]:
        with self._log_lock:
            return list(self._log[max(since, 0) :])

    # -- internals -----------------------------------------------------------

    def _add(self, who: str, text: str, **extra: Any) -> None:
        with self._log_lock:
            self._log.append(
                {
                    "i": len(self._log),
                    "t": time.time(),
                    "who": who,
                    "text": text,
                    **extra,
                }
            )

    def _turn(self, text: str, source: str) -> Any:
        self._add("operator", text, source=source)
        try:
            turn = self.session.turn(text)
        except Exception as error:  # noqa: BLE001 - shown, the chat goes on
            self._add("error", f"{type(error).__name__}: {error}")
            return None
        if not self._live:
            for call in turn.calls:
                self._tool(call)
        if turn.error:
            self._add("error", turn.error)
        if not self._live or not turn.say:
            self._add("model", turn.say)
        return turn

    def _tool(self, call: Any) -> None:
        self._add("tool", call.as_text(), tool=call.tool, ok=call.ok)
        result = call.result if call.ok else None
        if isinstance(result, dict) and isinstance(result.get("plan"), list):
            self._add("plan", "", steps=[str(step) for step in result["plan"]])

    def _post_action(self, body: Any, _query: dict[str, list[str]]) -> Any:
        name = body.get("name") if isinstance(body, dict) else None
        if not isinstance(name, str):
            raise ValueError('send {"name": action}')
        self.act(name)
        return {"accepted": True}

    def _post(self, body: Any, _query: dict[str, list[str]]) -> Any:
        message = (body or {}).get("message") if isinstance(body, dict) else None
        if not isinstance(message, str):
            raise ValueError('send {"message": text}')
        self.submit(message)
        return {"accepted": True, "ended": self.ended.is_set()}

    def _get(self, _body: Any, query: dict[str, list[str]]) -> Any:
        try:
            since = int(query.get("since", ["0"])[0])
        except ValueError as error:
            raise ValueError("since must be an integer") from error
        return {
            "since": since,
            "entries": self.transcript(since),
            "busy": self.busy,
            "ended": self.ended.is_set(),
            "actions": [
                {"name": a.name, "label": a.label, "enabled": _safe(a.enabled)}
                for a in self.actions.values()
            ],
        }


def _safe(enabled: Callable[[], bool]) -> bool:
    try:
        return bool(enabled())
    except Exception:  # noqa: BLE001 - a broken predicate disables its button
        return False
