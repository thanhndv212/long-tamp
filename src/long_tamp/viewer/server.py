"""Serve the viewer live, next to a running mission (standard library only).

``ViewerServer`` follows a mission's ``events.jsonl`` as it grows and serves
the viewer page, which polls for new events. Routes (all JSON):

- ``GET /`` the page; ``GET /api/events?since=N`` the events from index ``N``;
- ``POST /api/control`` ``{"action": "pause" | "resume" | "stop"}``, with an
  ``ExecutionControl``;
- more with ``route(method, path, handler)``.

It binds to 127.0.0.1 by default: control routes can pause or stop a robot,
so expose it beyond the machine on purpose (``host="0.0.0.0"``), not by
accident.
"""

from __future__ import annotations

import json
import threading
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

from .config import ViewerConfig
from .page import render_html

#: A route handler: the request's JSON body (``None`` for GET) and query
#: parameters in, a JSON-serializable response out. Raise ``ValueError`` for
#: a bad request (400).
Handler = Callable[[Any, dict[str, list[str]]], Any]


class EventTail:
    """The events of a JSONL file that may still be growing (thread-safe).

    A partly written last line is left for the next read.
    """

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self._events: list[dict[str, Any]] = []
        self._offset = 0
        self._lock = threading.Lock()

    def since(self, index: int) -> list[dict[str, Any]]:
        with self._lock:
            self._read_new()
            return self._events[max(index, 0) :]

    def _read_new(self) -> None:
        if not self.path.exists():
            return
        with open(self.path, "rb") as f:
            f.seek(self._offset)
            data = f.read()
        end = data.rfind(b"\n") + 1
        for line in data[:end].splitlines():
            if line.strip():
                self._events.append(json.loads(line))
        self._offset += end


class ViewerServer:
    """The live viewer of the mission writing ``events``.

    ``control`` (an ``ExecutionControl``) adds pause, resume and stop
    buttons. ``start()`` serves from a daemon thread and returns the URL.
    """

    def __init__(
        self,
        events: str | Path,
        config: ViewerConfig | None = None,
        port: int = 8090,
        host: str = "127.0.0.1",
        control: Any = None,
    ) -> None:
        self.tail = EventTail(events)
        self.config = config or ViewerConfig()
        self.control = control
        self.host, self.port = host, port
        self._routes: dict[tuple[str, str], Handler] = {
            ("GET", "/api/events"): self._events,
        }
        if control is not None:
            self.route("POST", "/api/control", self._control)
        self._httpd: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None

    def route(self, method: str, path: str, handler: Handler) -> None:
        """Serve ``handler`` at ``method path`` (an ``/api/...`` path)."""
        self._routes[(method.upper(), path)] = handler

    def page(self) -> str:
        return render_html(
            config=self.config,
            live=True,
            chat=("POST", "/api/chat") in self._routes,
            control=self.control is not None,
        )

    @property
    def url(self) -> str:
        host = "localhost" if self.host in ("127.0.0.1", "0.0.0.0") else self.host
        return f"http://{host}:{self.port}/"

    def start(self) -> str:
        self._httpd = ThreadingHTTPServer((self.host, self.port), _handler(self))
        self.port = self._httpd.server_address[1]  # port 0: the one picked
        self._thread = threading.Thread(
            target=self._httpd.serve_forever, name="viewer", daemon=True
        )
        self._thread.start()
        return self.url

    def close(self) -> None:
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None

    def __enter__(self) -> ViewerServer:
        self.start()
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    # -- routes ------------------------------------------------------------

    def _events(self, _body: Any, query: dict[str, list[str]]) -> Any:
        try:
            since = int(query.get("since", ["0"])[0])
        except ValueError as error:
            raise ValueError("since must be an integer") from error
        events = self.tail.since(since)
        return {"since": since, "events": events}

    def _control(self, body: Any, _query: dict[str, list[str]]) -> Any:
        action = (body or {}).get("action")
        if action not in ("pause", "resume", "stop"):
            raise ValueError("action must be pause, resume or stop")
        getattr(self.control, action)()
        return {
            "paused": self.control.paused,
            "stopped": self.control.stopped,
            "waiting_at": self.control.waiting_at,
        }


def _handler(server: ViewerServer) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args: Any) -> None:  # quiet: polled often
            pass

        def _send(self, code: int, body: bytes, kind: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code: int, value: Any) -> None:
            self._send(code, json.dumps(value).encode(), "application/json")

        def _dispatch(self, method: str) -> None:
            url = urlparse(self.path)
            if method == "GET" and url.path in ("/", "/index.html"):
                self._send(200, server.page().encode(), "text/html; charset=utf-8")
                return
            handler = server._routes.get((method, url.path))
            if handler is None:
                self._json(404, {"error": f"no route {method} {url.path}"})
                return
            body = None
            if method == "POST":
                length = int(self.headers.get("Content-Length", 0))
                try:
                    body = json.loads(self.rfile.read(length) or b"null")
                except json.JSONDecodeError:
                    self._json(400, {"error": "the body is not JSON"})
                    return
            try:
                self._json(200, handler(body, parse_qs(url.query)))
            except ValueError as error:
                self._json(400, {"error": str(error)})
            except Exception as error:  # noqa: BLE001 - report, keep serving
                self._json(500, {"error": f"{type(error).__name__}: {error}"})

        def do_GET(self) -> None:  # noqa: N802 - http.server's naming
            self._dispatch("GET")

        def do_POST(self) -> None:  # noqa: N802
            self._dispatch("POST")

    return Handler
