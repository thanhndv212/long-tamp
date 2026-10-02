"""The scene process (see ``long_tamp.visualization.scene``).

Reads its settings as JSON on stdin, loads the pickled pinocchio models,
serves them with ``pyhpp_viser`` and takes commands (JSON lines) on a local
TCP port. Prints ``PORTS <http> <control>`` once both listen; exits with its
parent.
"""

from __future__ import annotations

import collections
import json
import os
import pickle
import socketserver
import sys
import threading
import time
from typing import Any

#: Queued frames beyond this many seconds of playback are thinned (played
#: faster), so the scene stays near the mission: a simulation runs several
#: times faster than real time (live: ~1300 states in 11 s for 44 s of motion).
MAX_LAG = 20.0


class _Robot:
    """What ``pyhpp_viser.Viewer`` reads off an HPP device."""

    geomModel = None  # not callable: no collision geometry unless sent

    def __init__(self, models: dict[str, Any]) -> None:
        self._model, self._visual = models["model"], models["visual"]
        if "collision" in models:
            collision = models["collision"]
            self.geomModel = lambda: collision

    def model(self):
        return self._model

    def visualModel(self):  # noqa: N802 - HPP's naming
        return self._visual


class Player:
    """Shows queued frames at their pace, in its own thread."""

    def __init__(self, display) -> None:
        self.display = display
        self.queue: collections.deque = collections.deque()  # (dt, q)
        self.cond = threading.Condition()
        self._last_t: float | None = None  # stream time of the last frame
        #: bumped by show/clear: a frame taken before then is dropped
        self._generation = 0
        self._draw = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def show(self, q) -> None:
        with self.cond:
            self.queue.clear()
            self._last_t = None
            self._generation += 1
        with self._draw:
            self.display(q)

    def play(self, frames, dt: float) -> None:
        with self.cond:
            self.queue.extend((dt, q) for q in frames)
            self._last_t = None
            self._thin(dt)
            self.cond.notify()

    def frame(self, t: float, q) -> None:
        with self.cond:
            dt = 0.0 if self._last_t is None else min(max(t - self._last_t, 0.0), 0.5)
            self._last_t = t
            self.queue.append((dt, q))
            self._thin(dt or 1 / 30)
            self.cond.notify()

    def clear(self) -> None:
        with self.cond:
            self.queue.clear()
            self._last_t = None
            self._generation += 1

    def _thin(self, dt: float) -> None:
        if dt > 0 and len(self.queue) * dt > MAX_LAG:
            kept = list(self.queue)[::2]
            self.queue.clear()
            self.queue.extend((d * 2, q) for d, q in kept)

    def _run(self) -> None:
        due = time.monotonic()  # when the next frame is due (wall clock)
        while True:
            with self.cond:
                while not self.queue:
                    self.cond.wait()
                    due = time.monotonic()  # after a pause, start afresh
                dt, q = self.queue.popleft()
                generation = self._generation
            # pace against the clock, so drawing time doesn't slow playback
            due = max(due + dt, time.monotonic() - 0.25)
            delay = due - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            with self._draw:
                if generation != self._generation:
                    continue  # a show or clear came after it was taken
                try:
                    self.display(q)
                except Exception:  # noqa: BLE001 - a bad frame must not stop playback
                    pass


def main() -> None:
    settings = json.loads(sys.stdin.read())
    import numpy as np
    from pyhpp_viser import Viewer

    with open(settings["models"], "rb") as f:
        models = pickle.load(f)
    viewer = Viewer(_Robot(models))
    viewer.start(host=settings["host"], port=settings["port"], open=False)
    camera = settings.get("camera")

    @viewer.viewer.on_client_connect
    def aim(client):
        if camera:
            client.camera.position, client.camera.look_at = camera
        client.camera.up_direction = (0.0, 0.0, 1.0)

    player = Player(lambda q: viewer(np.asarray(q, dtype=float)))

    class Handler(socketserver.StreamRequestHandler):
        def handle(self) -> None:
            for line in self.rfile:
                try:
                    message = json.loads(line)
                except ValueError:
                    continue
                cmd = message.get("cmd")
                if cmd == "show":
                    player.show(message["q"])
                elif cmd == "play":
                    player.play(message["frames"], float(message.get("dt", 1 / 30)))
                elif cmd == "frame":
                    player.frame(float(message["t"]), message["q"])
                elif cmd == "clear":
                    player.clear()
                elif cmd == "status":
                    reply = {
                        "clients": len(viewer.viewer.get_clients()),
                        "queued": len(player.queue),
                    }
                    self.wfile.write((json.dumps(reply) + "\n").encode())

    class Server(socketserver.ThreadingTCPServer):
        daemon_threads = True
        allow_reuse_address = True

    control = Server(("127.0.0.1", 0), Handler)
    threading.Thread(target=control.serve_forever, daemon=True).start()
    http_port = (
        viewer.viewer.get_port()
        if hasattr(viewer.viewer, "get_port")
        else settings["port"]
    )
    print(f"PORTS {http_port} {control.server_address[1]}", flush=True)

    parent = settings["parent"]
    while os.getppid() == parent:
        time.sleep(1.0)
    os._exit(0)


if __name__ == "__main__":
    main()
