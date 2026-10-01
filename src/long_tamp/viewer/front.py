"""The viewer's front process (``ViewerServer(separate_process=True)``).

Serves the page and the events itself, reading the event stream, so it
answers at once however busy the mission's process is; forwards every other
``/api/`` request to the mission's process. Reads its settings as JSON on
stdin, prints ``PORT <n>`` once listening, and exits with its parent.
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from typing import Any

from .config import ViewerConfig
from .page import render_html
from .server import EventTail, _handler


class Front:
    def __init__(self, settings: dict[str, Any]) -> None:
        self.tail = EventTail(settings["events"])
        self.config = ViewerConfig.from_dict(settings["config"])
        self.backend = settings["backend"].rstrip("/")
        self._routes = {("GET", "/api/events"): self._events}

    def page(self) -> str:
        # control and chat are asked of the mission process (/api/features)
        return render_html(config=self.config, live=True)

    def _events(self, _body: Any, query: dict[str, list[str]]) -> Any:
        try:
            since = int(query.get("since", ["0"])[0])
        except ValueError as error:
            raise ValueError("since must be an integer") from error
        return {"since": since, "events": self.tail.since(since)}

    def proxy(self, method: str, path: str, body: bytes | None) -> tuple[int, bytes]:
        request = urllib.request.Request(
            self.backend + path,
            data=body if method == "POST" else None,
            headers={"Content-Type": "application/json"},
            method=method,
        )
        try:
            with urllib.request.urlopen(request, timeout=300) as r:
                return r.status, r.read()
        except urllib.error.HTTPError as error:
            return error.code, error.read()
        except OSError as error:
            message = f"the mission process did not answer: {error}"
            return 502, json.dumps({"error": message}).encode()


def _exit_with(parent: int) -> None:
    while True:
        if os.getppid() != parent:
            os._exit(0)
        time.sleep(1.0)


def main() -> None:
    settings = json.loads(sys.stdin.read())
    front = Front(settings)
    httpd = ThreadingHTTPServer((settings["host"], settings["port"]), _handler(front))
    threading.Thread(target=_exit_with, args=(settings["parent"],), daemon=True).start()
    print(f"PORT {httpd.server_address[1]}", flush=True)
    httpd.serve_forever()


if __name__ == "__main__":
    main()
