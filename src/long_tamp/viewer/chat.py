"""The chat panel's back end (#90): a ``ChatSession`` driven from the viewer.

``ChatBridge(session).attach(server)`` adds two routes to a ``ViewerServer``:

- ``POST /api/chat`` ``{"message": text}`` starts a turn and returns at once
  (a turn may run the robot for minutes); ``"quit"`` or ``"exit"`` ends the
  chat;
- ``GET /api/chat?since=N`` the transcript from entry ``N``, and whether a
  turn is running.

One turn runs at a time, whoever sends it: a terminal front end calls
``turn()`` on the same bridge, and the page shows its turns too. The tools'
calls reach the event stream through the session's ``on_tool``, as in the
terminal chat, so their results show in the event list and, for a new plan,
in the plan tree (a ``plan`` event).
"""

from __future__ import annotations

import threading
import time
from typing import Any

QUIT = ("quit", "exit")


class ChatBridge:
    def __init__(self, session: Any) -> None:
        self.session = session
        self.ended = threading.Event()
        self._turn_lock = threading.Lock()
        self._log_lock = threading.Lock()
        self._log: list[dict[str, Any]] = []
        self._worker: threading.Thread | None = None

    @property
    def busy(self) -> bool:
        return self._turn_lock.locked()

    def attach(self, server: Any) -> ChatBridge:
        server.route("POST", "/api/chat", self._post)
        server.route("GET", "/api/chat", self._get)
        return self

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
        for call in turn.calls:
            self._add("tool", call.as_text(), tool=call.tool, ok=call.ok)
        if turn.error:
            self._add("error", turn.error)
        self._add("model", turn.say)
        return turn

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
        }
