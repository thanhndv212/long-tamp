"""A client for the RoboCerebra benchmark env server (issue #156, R1).

The benchmark (a LIBERO fork on robosuite 1.4, Python 3.10, ``numpy<2``)
runs in its own interpreter, served by ``script/robocerebra/env_server.py``;
this is long_tamp's side. It speaks the wire of
:mod:`long_tamp.execution.remote` (length-prefixed JSON frames, the tagged
codec, the ``hello`` handshake) but not the execution-backend contract: the
env steps synchronously, one ``OSC_POSE`` action (or a chunk of them) per
call, and success is the benchmark's own stateful checker.

::

    with RoboCerebraClient.spawn(["python", "env_server.py", "--bench", bench]) as env:
        info = env.load("Ideal/case1")
        env.reset(step=0)
        result = env.step(actions)          # (N, 7)
        result["total_completed"], result["all_done"]
"""

from __future__ import annotations

import os
import socket
import subprocess
from collections.abc import Sequence
from typing import Any

import numpy as np

from long_tamp.execution.remote import (
    PROTOCOL_VERSION,
    decode,
    encode,
    recv_frame,
    send_frame,
)

SERVICE = "robocerebra"


class RoboCerebraClient:
    """One connection to a RoboCerebra env server at ``host:port``.

    ``call_timeout``: seconds to wait for a reply (loading a case builds the
    MuJoCo model; a long action chunk on a slow host takes a while).
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 7071,
        *,
        connect_timeout: float = 5.0,
        call_timeout: float = 300.0,
    ) -> None:
        self._sock = socket.create_connection((host, port), timeout=connect_timeout)
        self._sock.settimeout(call_timeout)
        self._process: subprocess.Popen[str] | None = None
        try:
            self.server_info = self._call("hello", version=PROTOCOL_VERSION)
        except Exception:
            self._sock.close()
            raise
        if self.server_info.get("service") != SERVICE:
            self._sock.close()
            raise ConnectionError(
                f"not a {SERVICE} env server: {self.server_info.get('service')!r}"
            )

    @classmethod
    def spawn(
        cls,
        command: Sequence[str],
        *,
        env: dict[str, str] | None = None,
        startup_timeout: float = 120.0,
        **kwargs: Any,
    ) -> RoboCerebraClient:
        """Start ``command`` (an env server; ``--port 0`` is appended so it
        picks a free port) and connect once it prints ``READY <port>``. The
        server is stopped on :meth:`close`."""
        process = subprocess.Popen(
            [*command, "--port", "0", "--once"],
            stdout=subprocess.PIPE,
            text=True,
            env={**os.environ, **(env or {})},
        )
        port = _wait_ready(process, startup_timeout)
        try:
            client = cls("127.0.0.1", port, **kwargs)
        except Exception:
            process.kill()
            process.wait()
            raise
        client._process = process
        return client

    # -- transport --

    def _call(self, op: str, *args: Any, **kwargs: Any) -> Any:
        message: dict[str, Any] = {"op": op, "args": encode(list(args))}
        if op == "hello":
            message.update(kwargs)
        elif kwargs:
            message["kwargs"] = encode(kwargs)
        send_frame(self._sock, message)
        reply = recv_frame(self._sock)
        if not reply.get("ok"):
            raise RuntimeError(f"robocerebra env: {op}: {reply.get('error')}")
        return decode(reply.get("value"))

    def close(self) -> None:
        try:
            send_frame(self._sock, {"op": "bye"})
        except OSError:
            pass
        self._sock.close()
        if self._process is not None:
            try:
                self._process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()
            self._process = None

    def __enter__(self) -> RoboCerebraClient:
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    # -- operations (see env_server.py for their semantics) --

    def ping(self) -> None:
        self._call("ping")

    def cases(self) -> list[str]:
        """Every case, ``"<task type>/case<N>"``."""
        return self._call("cases")

    def load(self, case: str, model: str = "bddl") -> dict[str, Any]:
        """Build the env for ``case``: its task, step descriptions and demo
        spans, goal, objects and regions. ``model="bddl"`` is the benchmark's
        scene (fixtures resampled each reset); ``"demo"`` the demo's own
        recorded scene, for replays."""
        return self._call("load", case, model=model)

    def reset(
        self, step: int = 0, settle: int | None = None, restore_demo: bool = False
    ) -> dict[str, Any]:
        """Start an episode at benchmark step ``step``: step 0 as the
        benchmark does; ``k > 0`` fresh from the demo's state at its start.
        ``restore_demo``: continue the demo's controller and gripper
        commands (for replays)."""
        kwargs: dict[str, Any] = {"step": step, "settle": settle}
        if restore_demo:
            kwargs["restore_demo"] = True
        return self._call("reset", **kwargs)

    def resume(self, step: int) -> dict[str, Any]:
        """The benchmark's mid-episode resume to the start of ``step``."""
        return self._call("resume", step)

    def step(self, actions: Any) -> dict[str, Any]:
        """Apply one action (7,) or a chunk (N, 7) of ``OSC_POSE`` actions."""
        return self._call("step", np.asarray(actions, dtype=float))

    def check(self) -> dict[str, Any]:
        return self._call("check")

    def get_state(self) -> dict[str, Any]:
        """``sim_state``, plus the ``controller`` goal and gripper command
        that the sim state leaves out."""
        return self._call("get_state")

    def set_state(
        self, sim_state: Any, controller: dict | None = None
    ) -> dict[str, Any]:
        """Set ``sim_state`` (and ``controller`` from :meth:`get_state`)."""
        if controller is None:
            return self._call("set_state", np.asarray(sim_state, dtype=float))
        return self._call(
            "set_state", np.asarray(sim_state, dtype=float), controller=controller
        )

    def render(
        self, cameras: Sequence[str] | None = None, size: int = 256
    ) -> dict[str, np.ndarray]:
        return self._call(
            "render", cameras=list(cameras) if cameras else None, size=size
        )

    def demo(self) -> dict[str, np.ndarray]:
        """The loaded case's demonstration ``states`` and ``actions``."""
        return self._call("demo")

    def export_scene(self, out_dir: str | os.PathLike[str] | None = None) -> Any:
        """The scene as it stands, as URDF/SRDF texts and a manifest
        (``{"files", "manifest"}``); with ``out_dir``, written there and
        returned as a :class:`~long_tamp.eval.robocerebra_scene.ExportedScene`."""
        export = self._call("export_scene")
        if out_dir is None:
            return export
        from .robocerebra_scene import write_export

        return write_export(export, out_dir)


def _wait_ready(process: subprocess.Popen[str], timeout: float) -> int:
    import selectors
    import time

    assert process.stdout is not None
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            if not selector.select(timeout=0.5):
                if process.poll() is not None:
                    break
                continue
            line = process.stdout.readline()
            if not line:
                break
            if line.startswith("READY "):
                return int(line.split()[1])
    finally:
        selector.close()
    process.kill()
    process.wait()
    raise RuntimeError(f"env server did not start (exit code {process.returncode})")
