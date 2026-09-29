"""Run an execution backend in its own process.

A simulator stepped from a Python thread shares the interpreter lock with the
planner: planning ahead in the executor's worker thread then slows both down
(the screw assembly's real-time MuJoCo motion took 3.6 times its duration
while planning ran next to it). ``ProcessBackend(factory, *args)`` builds the
backend in a separate process, like a robot controller on its own computer,
and forwards the contract (``start``, ``poll``, ``cancel``) and the optional
methods the backend has (``start_error``, ``observed_config``, ``disturb``)
over a pipe.

Commands are pickled, so their paths are sampled into arrays first
(``SampledPath``, in the caller's thread); the factory and its arguments must
be picklable too (a class and plain data).
"""

from __future__ import annotations

import multiprocessing as mp
import threading
from collections.abc import Callable
from typing import Any

from .contract import ExecutionCommand, ExecutionStatus, Feedback
from .sampled import sampled

#: Optional backend methods forwarded when the backend has them.
OPTIONAL = ("start_error", "observed_config", "disturb")


def _serve(conn: Any, factory: Callable[..., Any], args: tuple, kwargs: dict) -> None:
    try:
        backend = factory(*args, **kwargs)
    except Exception as error:  # noqa: BLE001 - reported to the parent
        conn.send((False, f"{type(error).__name__}: {error}"))
        return
    conn.send((True, [name for name in OPTIONAL if hasattr(backend, name)]))
    while True:
        try:
            name, call_args = conn.recv()
        except EOFError:
            return
        if name == "close":
            conn.send((True, None))
            return
        try:
            conn.send((True, getattr(backend, name)(*call_args)))
        except Exception as error:  # noqa: BLE001 - reported to the parent
            conn.send((False, f"{type(error).__name__}: {error}"))


class ProcessBackend:
    """``factory(*args, **kwargs)``'s backend, running in a spawned process."""

    def __init__(self, factory: Callable[..., Any], *args: Any, **kwargs: Any) -> None:
        context = mp.get_context("spawn")
        self._conn, child = context.Pipe()
        self._process = context.Process(
            target=_serve, args=(child, factory, args, kwargs), daemon=True
        )
        self._process.start()
        ok, value = self._conn.recv()
        if not ok:
            self._process.join()
            raise RuntimeError(f"backend process failed to start: {value}")
        self._optional = set(value)
        self._lock = threading.Lock()

    def _call(self, name: str, *args: Any) -> Any:
        with self._lock:
            self._conn.send((name, args))
            ok, value = self._conn.recv()
        if not ok:
            raise RuntimeError(f"backend process: {name}: {value}")
        return value

    @staticmethod
    def _portable(command: ExecutionCommand) -> ExecutionCommand:
        return ExecutionCommand(
            command.step_id, command.duration, sampled(command.payload)
        )

    def start(self, command: ExecutionCommand) -> ExecutionStatus:
        return self._call("start", self._portable(command))

    def poll(self) -> tuple[ExecutionStatus, Feedback | None]:
        return self._call("poll")

    def cancel(self) -> None:
        self._call("cancel")

    def __getattr__(self, name: str) -> Any:
        if name.startswith("_") or name not in self.__dict__.get("_optional", ()):
            raise AttributeError(name)
        if name == "start_error":
            return lambda command: self._call(name, self._portable(command))
        return lambda *args: self._call(name, *args)

    def close(self) -> None:
        if self._process.is_alive():
            try:
                self._call("close")
            except (OSError, EOFError, RuntimeError):
                pass
            self._process.join(timeout=5)
        if self._process.is_alive():
            self._process.kill()

    def __del__(self) -> None:  # pragma: no cover - best effort
        try:
            self.close()
        except Exception:  # noqa: BLE001
            pass
