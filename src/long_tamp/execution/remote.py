"""Run an execution backend on another host, over TCP (ADR-0007).

A robot is usually driven from somewhere other than the planner: the planner
runs in a container, the arm's servos are on the host's USB port, or the
robot has its own controller PC. ``serve_backend(backend, port=...)`` serves a
backend on the robot's side; ``RemoteBackend(host, port)`` is the planner's
side, an :class:`ExecutionBackend` that forwards ``start``, ``poll``,
``cancel`` and the optional methods the served backend has (``start_error``,
``observed_config``, ``disturb``, and a hand's ``open_gripper``,
``close_gripper`` and ``gripper_state``).

The wire is length-prefixed JSON frames, **never pickle**: a socket is not a
trust boundary pickle can cross. Commands carry their paths as sampled arrays
(``SampledPath``, sampled in the caller's thread as for ``ProcessBackend``);
anything the codec doesn't know is refused, not guessed.

Safety: the server serves one client at a time and cancels the running
command (the backend holds where it is) when that client disconnects, or when
it sends nothing for ``timeout`` seconds while the command runs (the client's
polls are the heartbeat; an idle client may be silent, e.g. while it plans).
A lost connection is a ``FAILURE`` whose message starts with ``disconnected``.

There is no authentication: bind to localhost (the default) and reach a
remote robot through an SSH tunnel or a trusted network.

``RemoteBackend.spawn(factory, *args)`` serves a backend from a local child
process, like ``ProcessBackend`` but over the same wire as a remote robot.
``python -m long_tamp.execution.remote module:factory`` serves one from the
command line.
"""

from __future__ import annotations

import argparse
import base64
import importlib
import json
import logging
import multiprocessing as mp
import selectors
import socket
import struct
import threading
import time
from collections.abc import Callable
from typing import Any

import numpy as np

from .contract import ExecutionCommand, ExecutionStatus, Feedback
from .gripper import GripperState
from .process import OPTIONAL
from .sampled import SampledPath, sampled

log = logging.getLogger(__name__)

#: Wire protocol version; a client and a server must agree on it.
PROTOCOL_VERSION = 1
#: Largest frame accepted [bytes] (a minute of 30-joint samples at 5 ms is ~3 MB).
MAX_FRAME = 64 * 1024 * 1024
_HEADER = struct.Struct(">I")


# -- codec ------------------------------------------------------------------


def encode(value: Any) -> Any:
    """``value`` as plain JSON data. Tagged objects are one-key dicts whose
    key starts with ``$``; unknown types raise ``TypeError``."""
    from long_tamp.tasks.task_planning.skills import SkillCommand

    if isinstance(value, ExecutionStatus):  # a str: before the primitives
        return {"$status": value.value}
    if value is None or isinstance(value, bool | int | float | str):
        return value
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        data = np.ascontiguousarray(value)
        return {
            "$nd": {
                "dtype": data.dtype.str,
                "shape": list(data.shape),
                "data": base64.b64encode(data.tobytes()).decode("ascii"),
            }
        }
    if isinstance(value, SampledPath):
        return {
            "$path": {"times": encode(value.times), "q": encode(value.configurations)}
        }
    if isinstance(value, SkillCommand):
        spec = value.spec
        return {
            "$skill": {
                "spec": {
                    "name": spec.name,
                    "parameters": list(spec.parameters),
                    "preconditions": list(spec.preconditions),
                    "postconditions": list(spec.postconditions),
                    "failures": list(spec.failures),
                    "start": _pose(spec.start),
                    "end": _pose(spec.end),
                    "description": spec.description,
                },
                "parameters": encode(dict(value.parameters)),
                "approach": encode(value.approach),
                "options": encode(dict(value.options)),
            }
        }
    if isinstance(value, ExecutionCommand):
        return {
            "$command": {
                "step_id": value.step_id,
                "duration": value.duration,
                "payload": encode(value.payload),
            }
        }
    if isinstance(value, Feedback):
        return {
            "$feedback": {
                "progress": value.progress,
                "message": value.message,
                "metrics": encode(value.metrics),
                "facts": list(value.facts),
            }
        }
    if isinstance(value, GripperState):
        return {"$gripper": {"width": value.width, "effort": value.effort}}
    if isinstance(value, tuple):
        return {"$tuple": [encode(v) for v in value]}
    if isinstance(value, list):
        return [encode(v) for v in value]
    if isinstance(value, dict):
        out = {}
        for key, item in value.items():
            if not isinstance(key, str) or key.startswith("$"):
                raise TypeError(f"cannot send dict key {key!r}")
            out[key] = encode(item)
        return out
    raise TypeError(f"cannot send a {type(value).__name__} to a remote backend")


def _pose(pose: Any) -> dict[str, Any] | None:
    if pose is None:
        return None
    return {"frame": pose.frame, "target": pose.target, "offset": pose.offset}


def decode(value: Any) -> Any:
    """The inverse of :func:`encode`."""
    if isinstance(value, list):
        return [decode(v) for v in value]
    if not isinstance(value, dict):
        return value
    if len(value) == 1:
        ((tag, body),) = value.items()
        if tag.startswith("$"):
            return _decode_tagged(tag, body)
    return {key: decode(item) for key, item in value.items()}


def _decode_tagged(tag: str, body: Any) -> Any:
    from long_tamp.tasks.task_planning.skills import SkillCommand, SkillPose, SkillSpec

    if tag == "$status":
        return ExecutionStatus(body)
    if tag == "$nd":
        data = base64.b64decode(body["data"])
        return (
            np.frombuffer(data, dtype=np.dtype(body["dtype"]))
            .reshape(body["shape"])
            .copy()
        )
    if tag == "$path":
        return SampledPath.from_samples(decode(body["times"]), decode(body["q"]))
    if tag == "$skill":
        spec = dict(body["spec"])
        for key in ("parameters", "preconditions", "postconditions", "failures"):
            spec[key] = tuple(spec[key])
        for key in ("start", "end"):
            spec[key] = SkillPose(**spec[key]) if spec[key] is not None else None
        return SkillCommand(
            SkillSpec(**spec),
            decode(body["parameters"]),
            decode(body["approach"]),
            decode(body["options"]),
        )
    if tag == "$command":
        return ExecutionCommand(
            body["step_id"], body["duration"], decode(body["payload"])
        )
    if tag == "$feedback":
        return Feedback(
            progress=body["progress"],
            message=body["message"],
            metrics=decode(body["metrics"]),
            facts=tuple(body["facts"]),
        )
    if tag == "$gripper":
        return GripperState(body["width"], body["effort"])
    if tag == "$tuple":
        return tuple(decode(v) for v in body)
    raise ValueError(f"unknown tag {tag!r}")


def send_frame(sock: socket.socket, message: Any) -> None:
    """Send ``message`` (already encoded) as one frame."""
    data = json.dumps(message, separators=(",", ":")).encode()
    if len(data) > MAX_FRAME:
        raise ValueError(f"frame of {len(data)} bytes exceeds {MAX_FRAME}")
    sock.sendall(_HEADER.pack(len(data)) + data)


def recv_frame(sock: socket.socket) -> Any:
    """Receive one frame; ``ConnectionError`` when the peer is gone."""
    (size,) = _HEADER.unpack(_recv_exact(sock, _HEADER.size))
    if size > MAX_FRAME:
        raise ConnectionError(f"frame of {size} bytes exceeds {MAX_FRAME}")
    return json.loads(_recv_exact(sock, size))


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks, remaining = [], size
    while remaining:
        chunk = sock.recv(min(remaining, 1 << 20))
        if not chunk:
            raise ConnectionError("connection closed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


# -- server -----------------------------------------------------------------


def serve_backend(
    backend: Any,
    host: str = "127.0.0.1",
    port: int = 0,
    *,
    timeout: float = 2.0,
    ready: Callable[[int], None] | None = None,
    stop: threading.Event | None = None,
    once: bool = False,
) -> None:
    """Serve ``backend`` on ``host:port`` until ``stop`` is set (or, with
    ``once``, until the first client leaves).

    One client at a time; others are refused. When the client disconnects or
    sends nothing for ``timeout`` seconds while a command runs, the command is
    cancelled. ``ready(port)`` is called once listening (``port=0`` picks a
    free one).
    """
    optional = [name for name in OPTIONAL if hasattr(backend, name)]
    server = socket.create_server((host, port))
    server.setblocking(False)
    selector = selectors.DefaultSelector()
    selector.register(server, selectors.EVENT_READ)
    if ready is not None:
        ready(server.getsockname()[1])
    log.info(
        "serving %s on %s:%d", type(backend).__name__, host, server.getsockname()[1]
    )

    client: socket.socket | None = None
    last_seen = 0.0
    running = False

    def drop(why: str) -> None:
        nonlocal client, running
        if running:
            log.warning("client %s: cancelling the running command", why)
            try:
                backend.cancel()
            except Exception:  # noqa: BLE001 - the connection is gone anyway
                log.exception("cancel failed")
        running = False
        if client is not None:
            selector.unregister(client)
            client.close()
            client = None

    try:
        while stop is None or not stop.is_set():
            for key, _ in selector.select(timeout=min(0.05, timeout / 4)):
                if key.fileobj is server:
                    conn, _addr = server.accept()
                    if client is not None:
                        _refuse(conn, "busy: another client is connected")
                        continue
                    conn.setblocking(True)
                    conn.settimeout(timeout)
                    client, last_seen = conn, time.monotonic()
                    selector.register(conn, selectors.EVENT_READ)
                    continue
                try:
                    request = recv_frame(client)
                except (OSError, ValueError) as error:
                    drop(f"disconnected ({error})")
                    if once:
                        return
                    continue
                last_seen = time.monotonic()
                op = request.get("op")
                if op == "bye":
                    drop("left")
                    if once:
                        return
                    continue
                reply, running = _handle(backend, optional, request, running)
                try:
                    send_frame(client, reply)
                except OSError as error:
                    drop(f"disconnected ({error})")
                    if once:
                        return
            if running and time.monotonic() - last_seen > timeout:
                # Keep the connection: the client may only be slow (a planner
                # holding the interpreter lock); its next poll sees the cancel.
                log.warning("client silent for %g s: cancelling the command", timeout)
                running = False
                try:
                    backend.cancel()
                except Exception:  # noqa: BLE001 - keep serving
                    log.exception("cancel failed")
    finally:
        drop("server stopping")
        selector.close()
        server.close()


def _refuse(conn: socket.socket, error: str) -> None:
    try:
        conn.settimeout(1.0)
        send_frame(conn, {"ok": False, "error": error})
    except OSError:
        pass
    conn.close()


def _handle(
    backend: Any, optional: list[str], request: dict[str, Any], running: bool
) -> tuple[dict[str, Any], bool]:
    """One request: the reply and whether a command is now running."""
    op, args = request.get("op"), request.get("args", [])
    if op == "hello":
        if request.get("version") != PROTOCOL_VERSION:
            error = f"protocol {request.get('version')} != {PROTOCOL_VERSION}"
            return {"ok": False, "error": error}, running
        return {
            "ok": True,
            "value": {"version": PROTOCOL_VERSION, "optional": optional},
        }, running
    if op == "ping":
        return {"ok": True, "value": None}, running
    if op not in ("start", "poll", "cancel", *optional):
        return {"ok": False, "error": f"unknown operation {op!r}"}, running
    try:
        value = getattr(backend, op)(*decode(args))
        if op == "start":
            running = value is ExecutionStatus.RUNNING
        elif op == "poll":
            running = (
                value[0] in (ExecutionStatus.RUNNING, ExecutionStatus.BUSY) and running
            )
        elif op == "cancel":
            running = False
        return {"ok": True, "value": encode(value)}, running
    except Exception as error:  # noqa: BLE001 - reported to the client
        return {"ok": False, "error": f"{type(error).__name__}: {error}"}, running


# -- client -----------------------------------------------------------------


class RemoteBackend:
    """An execution backend served by :func:`serve_backend` at ``host:port``.

    ``keepalive``: seconds between pings while no call is made (keep it well
    under the server's ``timeout``). ``call_timeout``: seconds to wait for a
    reply. A lost connection makes ``start`` and ``poll`` report ``FAILURE``
    with a message starting with ``disconnected``; ``cancel`` is then a no-op.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 7070,
        *,
        keepalive: float = 0.5,
        connect_timeout: float = 5.0,
        call_timeout: float = 30.0,
    ) -> None:
        self._sock = socket.create_connection((host, port), timeout=connect_timeout)
        self._sock.settimeout(call_timeout)
        self._lock = threading.Lock()
        self._closed = threading.Event()
        self._lost: str | None = None
        self._last_sent = time.monotonic()
        self._process: Any = None
        hello = self._call("hello", version=PROTOCOL_VERSION)
        self._optional = set(hello["optional"])
        self._keepalive = threading.Thread(
            target=self._ping, args=(keepalive,), daemon=True, name="remote-keepalive"
        )
        self._keepalive.start()

    @classmethod
    def spawn(
        cls,
        factory: Callable[..., Any],
        *args: Any,
        timeout: float = 2.0,
        **kwargs: Any,
    ) -> RemoteBackend:
        """``factory(*args, **kwargs)``'s backend served from a local child
        process (spawned, so the factory and arguments must be picklable, as
        for ``ProcessBackend``; only the commands cross the wire)."""
        context = mp.get_context("spawn")
        parent, child = context.Pipe()
        process = context.Process(
            target=_serve_child,
            args=(child, factory, args, kwargs, timeout),
            daemon=True,
        )
        process.start()
        ok, value = parent.recv()
        if not ok:
            process.join()
            raise RuntimeError(f"backend process failed to start: {value}")
        backend = cls("127.0.0.1", value, keepalive=timeout / 4)
        backend._process = process
        return backend

    # -- transport --

    def _call(self, op: str, *args: Any, **fields: Any) -> Any:
        with self._lock:
            if self._lost is not None:
                raise ConnectionError(self._lost)
            try:
                send_frame(self._sock, {"op": op, "args": encode(list(args)), **fields})
                self._last_sent = time.monotonic()
                reply = recv_frame(self._sock)
            except (OSError, ValueError) as error:
                self._lost = f"disconnected: {error}"
                self._sock.close()
                raise ConnectionError(self._lost) from error
        if not reply.get("ok"):
            raise RuntimeError(f"remote backend: {op}: {reply.get('error')}")
        return decode(reply.get("value"))

    def _ping(self, period: float) -> None:
        while not self._closed.wait(period / 2):
            if time.monotonic() - self._last_sent < period:
                continue
            try:
                self._call("ping")
            except ConnectionError:
                return
            except RuntimeError:
                log.exception("remote backend: ping refused")
                return

    @property
    def connected(self) -> bool:
        return self._lost is None and not self._closed.is_set()

    # -- the contract --

    def start(self, command: ExecutionCommand) -> ExecutionStatus:
        try:
            return self._call("start", _portable(command))
        except ConnectionError as error:
            log.error("remote backend: %s", error)
            return ExecutionStatus.FAILURE

    def poll(self) -> tuple[ExecutionStatus, Feedback | None]:
        try:
            status, feedback = self._call("poll")
        except ConnectionError as error:
            return ExecutionStatus.FAILURE, Feedback(message=str(error))
        return status, feedback

    def cancel(self) -> None:
        try:
            self._call("cancel")
        except ConnectionError:
            pass  # the server cancels on disconnect

    def _start_error(self, command: ExecutionCommand) -> float | None:
        try:
            return self._call("start_error", _portable(command))
        except ConnectionError:
            return None  # unknown; ``start`` then fails as disconnected

    def __getattr__(self, name: str) -> Any:
        if name.startswith("_") or name not in self.__dict__.get("_optional", ()):
            raise AttributeError(name)
        if name == "start_error":
            return self._start_error
        return lambda *args: self._call(name, *args)

    def close(self) -> None:
        if self._closed.is_set():
            return
        self._closed.set()
        with self._lock:
            if self._lost is None:
                try:
                    send_frame(self._sock, {"op": "bye"})
                except OSError:
                    pass
            self._sock.close()
        if self._process is not None:
            self._process.join(timeout=5)
            if self._process.is_alive():
                self._process.kill()

    def __enter__(self) -> RemoteBackend:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def __del__(self) -> None:  # pragma: no cover - best effort
        try:
            self.close()
        except Exception:  # noqa: BLE001
            pass


def _portable(command: ExecutionCommand) -> ExecutionCommand:
    return ExecutionCommand(command.step_id, command.duration, sampled(command.payload))


def _serve_child(
    conn: Any, factory: Callable[..., Any], args: tuple, kwargs: dict, timeout: float
) -> None:
    try:
        backend = factory(*args, **kwargs)
    except Exception as error:  # noqa: BLE001 - reported to the parent
        conn.send((False, f"{type(error).__name__}: {error}"))
        return
    serve_backend(
        backend, timeout=timeout, ready=lambda port: conn.send((True, port)), once=True
    )


# -- command line -----------------------------------------------------------


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(
        prog="python -m long_tamp.execution.remote",
        description="Serve an execution backend over TCP (ADR-0007).",
    )
    ap.add_argument("factory", help="module:callable that builds the backend")
    ap.add_argument("--kwargs", default="{}", help="JSON keyword arguments for it")
    ap.add_argument("--host", default="127.0.0.1", help="bind address (no auth!)")
    ap.add_argument("--port", type=int, default=7070)
    ap.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="seconds of client silence before the running command is cancelled",
    )
    args = ap.parse_args(argv)
    module, _, attr = args.factory.partition(":")
    factory = getattr(importlib.import_module(module), attr)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    serve_backend(
        factory(**json.loads(args.kwargs)),
        args.host,
        args.port,
        timeout=args.timeout,
    )


if __name__ == "__main__":
    main()
