"""The 3D scene in its own process (#106).

HPP planning holds the interpreter lock for seconds at a time, so a Viser
server inside the mission's process freezes while a step plans: the page
loads slowly and the robot stops moving. ``SceneProcess`` runs the scene in a
child process instead (``scene_server``), built from the robot's pinocchio
models (which pickle), and takes configurations over a local socket:

- ``show(q)``: display ``q`` now (drops anything queued);
- ``play(frames, dt)``: queue a sequence (a path, a gripper closing), played
  at its own pace; the caller doesn't wait for it;
- ``frame(t, q)``: stream states stamped with their (simulated) time, e.g.
  from the MuJoCo process, played back in real time.

``SceneClient`` is the sending end: it pickles to just an address, so another
process (the simulation's) can send frames too. Sending never raises: a
scene that is gone only stops showing.
"""

from __future__ import annotations

import json
import os
import pickle
import socket
import subprocess
import sys
import tempfile
import threading
from collections.abc import Sequence
from typing import Any


def _floats(q: Any) -> list[float]:
    return [float(v) for v in q]


class SceneClient:
    """Sends configurations to a scene server at ``host:port``."""

    def __init__(self, port: int, host: str = "127.0.0.1") -> None:
        self.host, self.port = host, port
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()

    def __getstate__(self) -> dict[str, Any]:  # an address, not a socket
        return {"host": self.host, "port": self.port}

    def __setstate__(self, state: dict[str, Any]) -> None:
        self.__init__(state["port"], state["host"])

    def _send(self, message: dict[str, Any], reply: bool = False) -> Any:
        line = (json.dumps(message) + "\n").encode()
        with self._lock:
            for _ in range(2):  # reconnect once
                try:
                    if self._sock is None:
                        self._sock = socket.create_connection(
                            (self.host, self.port), timeout=2.0
                        )
                    self._sock.sendall(line)
                    if not reply:
                        return None
                    data = b""
                    while not data.endswith(b"\n"):
                        chunk = self._sock.recv(65536)
                        if not chunk:
                            raise OSError("closed")
                        data += chunk
                    return json.loads(data)
                except (OSError, ValueError):
                    if self._sock is not None:
                        self._sock.close()
                    self._sock = None
        return None

    def show(self, q: Any) -> None:
        self._send({"cmd": "show", "q": _floats(q)})

    def play(self, frames: Sequence[Any], dt: float) -> None:
        if len(frames):
            self._send(
                {"cmd": "play", "dt": float(dt), "frames": [_floats(q) for q in frames]}
            )

    def frame(self, t: float, q: Any) -> None:
        self._send({"cmd": "frame", "t": float(t), "q": _floats(q)})

    def clear(self) -> None:
        self._send({"cmd": "clear"})

    def status(self) -> dict[str, Any] | None:
        """``{"clients": n, "queued": n}``, or ``None`` if the scene is gone."""
        return self._send({"cmd": "status"}, reply=True)

    def __call__(self, q: Any) -> None:  # usable as a display callback
        self.show(q)


class SceneProcess:
    """A Viser scene of ``robot`` (anything with ``model()`` and
    ``visualModel()``: an HPP device) served by a child process on ``port``.

    ``camera`` is ``(position, look_at)`` for new clients. ``collisions``
    also sends the collision model (large; the viewer can then show it).
    """

    def __init__(
        self,
        robot: Any,
        port: int = 8081,
        host: str = "0.0.0.0",
        camera: tuple[Sequence[float], Sequence[float]] | None = None,
        collisions: bool = False,
    ) -> None:
        models = {"model": robot.model(), "visual": robot.visualModel()}
        if collisions:
            models["collision"] = robot.geomModel()
        fd, self._models_path = tempfile.mkstemp(prefix="lt-scene-", suffix=".pkl")
        with os.fdopen(fd, "wb") as f:
            pickle.dump(models, f)
        settings = {
            "models": self._models_path,
            "host": host,
            "port": port,
            "camera": [list(camera[0]), list(camera[1])] if camera else None,
            "parent": os.getpid(),
        }
        self._child = subprocess.Popen(
            [sys.executable, "-m", "long_tamp.visualization.scene_server"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
        )
        self._child.stdin.write(json.dumps(settings))
        self._child.stdin.close()
        line = ""
        for line in self._child.stdout:  # the child may log before it is ready
            if line.startswith("PORTS "):
                break
        if not line.startswith("PORTS "):
            self.close()
            raise RuntimeError("the scene process did not start")
        _, http_port, control_port = line.split()
        self.port = int(http_port)
        self.client = SceneClient(int(control_port))
        # keep reading its output, so it never blocks on a full pipe
        threading.Thread(target=self._drain, daemon=True).start()

    def _drain(self) -> None:
        for _ in self._child.stdout:
            pass

    @property
    def url(self) -> str:
        return f"http://localhost:{self.port}"

    def show(self, q: Any) -> None:
        self.client.show(q)

    def play(self, frames: Sequence[Any], dt: float) -> None:
        self.client.play(frames, dt)

    def status(self) -> dict[str, Any] | None:
        return self.client.status()

    def close(self) -> None:
        if self._child.poll() is None:
            self._child.terminate()
            try:
                self._child.wait(5)
            except subprocess.TimeoutExpired:
                self._child.kill()
        try:
            os.unlink(self._models_path)
        except OSError:
            pass


__all__ = ["SceneClient", "SceneProcess"]
