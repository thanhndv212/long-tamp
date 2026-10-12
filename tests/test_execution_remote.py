"""The remote execution backend (issue #125, ADR-0007): the JSON codec, the
contract over a socket, stop-on-disconnect and the single-client rule."""

import json
import pickle
import queue
import socket
import subprocess
import sys
import textwrap
import threading
import time
from contextlib import contextmanager

import numpy as np
import pytest

from long_tamp.execution import (
    ExecutionCommand,
    ExecutionControl,
    ExecutionPolicy,
    ExecutionStatus,
    Feedback,
    MockBackend,
    PathPlaybackBackend,
    RemoteBackend,
    SampledPath,
    remote,
    run_command,
    serve_backend,
)
from long_tamp.tasks.task_planning.skills import SkillCommand, SkillPose, SkillSpec


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class _Path:
    def length(self):
        return 2.0

    def eval(self, t):
        return np.array([t / 2.0, 1.0 - t / 2.0]), True


@contextmanager
def served(backend, timeout=2.0):
    """``backend`` served from a thread of this process; yields the port."""
    stop, ports = threading.Event(), queue.Queue()
    thread = threading.Thread(
        target=serve_backend,
        args=(backend,),
        kwargs={"timeout": timeout, "ready": ports.put, "stop": stop},
        daemon=True,
    )
    thread.start()
    try:
        yield ports.get(timeout=5)
    finally:
        stop.set()
        thread.join(5)


def _wait_for(predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def _run(backend, duration=2.0, **policy):
    """``backend`` served and run through ``RemoteBackend`` on a shared fake
    clock (the server runs in this process)."""
    clock = FakeClock()
    backend.clock = clock
    with served(backend) as port, RemoteBackend(port=port) as client:
        return run_command(
            client,
            ExecutionCommand("s1", duration=duration),
            ExecutionPolicy(**policy),
            clock=clock,
            sleep=clock.sleep,
        )


# ----------------------------------------------------------------- codec


def _roundtrip(value):
    return remote.decode(json.loads(json.dumps(remote.encode(value))))


def test_the_codec_round_trips_arrays_paths_and_commands():
    for array in (np.arange(6, dtype=np.float32).reshape(2, 3), np.array([1, 2])):
        back = _roundtrip(array)
        assert back.dtype == array.dtype and np.array_equal(back, array)

    path = SampledPath(_Path(), dt=0.5)
    back = _roundtrip(path)
    assert back.length() == 2.0
    assert np.allclose(back.eval(1.0)[0], [0.5, 0.5])

    spec = SkillSpec(
        "screw",
        ("tool", "part"),
        preconditions=("holds(?tool)",),
        postconditions=("screwed(?part)",),
        failures=("screw_misaligned(?tool, ?part)",),
        start=SkillPose("?tool", "?part", -0.02),
    )
    command = ExecutionCommand(
        "s1",
        3.0,
        SkillCommand(spec, {"tool": "driver", "part": "p1"}, path, {"torque": 1.5}),
    )
    back = _roundtrip(command)
    assert (back.step_id, back.duration) == ("s1", 3.0)
    assert back.payload.spec == spec
    assert back.payload.parameters == {"tool": "driver", "part": "p1"}
    assert back.payload.options == {"torque": 1.5}
    assert back.payload.length() == 2.0

    feedback = Feedback(0.5, "ok", {"err": 0.1}, ("screwed(p1)",))
    assert _roundtrip(feedback) == feedback
    assert _roundtrip((ExecutionStatus.SUCCESS, None)) == (
        ExecutionStatus.SUCCESS,
        None,
    )
    assert _roundtrip({"a": [1, (2.0, "x")], "b": np.float64(3)}) == {
        "a": [1, (2.0, "x")],
        "b": 3.0,
    }


def test_the_codec_refuses_what_it_does_not_know():
    with pytest.raises(TypeError):
        remote.encode(object())
    with pytest.raises(TypeError):
        remote.encode({"$nd": 1})  # would read back as a tagged value
    with pytest.raises(TypeError):
        remote.encode({1: "x"})


def test_nothing_is_pickled_on_the_wire(monkeypatch):
    def refuse(*args, **kwargs):
        raise AssertionError("pickle used")

    monkeypatch.setattr(pickle, "dumps", refuse)
    monkeypatch.setattr(pickle, "loads", refuse)
    backend = PathPlaybackBackend(speed=100.0)
    with served(backend) as port, RemoteBackend(port=port) as client:
        result = run_command(
            client,
            ExecutionCommand("s1", duration=2.0, payload=_Path()),
            ExecutionPolicy(poll_interval=0.005),
        )
    assert result.status is ExecutionStatus.SUCCESS
    assert isinstance(backend._path, SampledPath)  # sampled by the client


# ------------------------------------------------------- the contract, remote


def test_a_command_runs_to_success_with_heartbeats():
    result = _run(MockBackend(), duration=2.0)
    assert result.status is ExecutionStatus.SUCCESS
    assert result.feedback_count >= 1


def test_failure_is_passed_through():
    result = _run(MockBackend(fail_at=1.0, message="joint limit"))
    assert result.status is ExecutionStatus.FAILURE
    assert result.reason == "failed" and "joint limit" in result.message


def test_busy_is_retried_not_failed():
    result = _run(MockBackend(busy_starts=3))
    assert result.status is ExecutionStatus.SUCCESS
    assert result.busy_retries == 3


def test_silence_beyond_the_inactivity_timeout_cancels_the_command():
    backend = MockBackend(stall_at=1.0)
    result = _run(backend, duration=10.0, inactivity_timeout=3.0)
    assert result.status is ExecutionStatus.FAILURE
    assert result.reason == "inactivity"
    assert backend.cancelled


def test_a_stop_request_cancels_a_running_command():
    control = ExecutionControl()
    backend = MockBackend()
    clock = FakeClock()
    backend.clock = clock

    def sleep(seconds):
        clock.sleep(seconds)
        if clock.now >= 1.0:
            control.stop()

    with served(backend) as port, RemoteBackend(port=port) as client:
        result = run_command(
            client,
            ExecutionCommand("s1", duration=5.0),
            ExecutionPolicy(),
            control=control,
            clock=clock,
            sleep=sleep,
        )
    assert result.status is ExecutionStatus.FAILURE
    assert result.reason == "stopped"
    assert backend.cancelled


def test_optional_methods_are_forwarded_only_when_served():
    class Observed(MockBackend):
        def observed_config(self):
            return np.array([0.1, 0.2])

        def start_error(self, command):
            return float(command.payload.length())

    with served(Observed()) as port, RemoteBackend(port=port) as client:
        assert np.allclose(client.observed_config(), [0.1, 0.2])
        assert client.start_error(ExecutionCommand("s", payload=_Path())) == 2.0
        assert not hasattr(client, "disturb")
    with served(MockBackend()) as port, RemoteBackend(port=port) as client:
        assert not hasattr(client, "observed_config")


# ------------------------------------------------------------- processes


def test_a_spawned_backend_runs_an_executor_mission():
    from tests.test_execution_executor import _executor

    backend = RemoteBackend.spawn(MockBackend, rtf=1000.0)
    try:
        executor, _ = _executor(set(), backend=backend, motions=2)
        executor.plan_ahead = True
        run = executor.run()
        assert run.success, run.message
        assert len(run.executions) == 3
    finally:
        backend.close()


def test_a_spawned_path_playback_plays_a_sampled_path():
    backend = RemoteBackend.spawn(PathPlaybackBackend, speed=20.0)
    try:
        result = run_command(
            backend,
            ExecutionCommand("s1", duration=2.0, payload=_Path()),
            ExecutionPolicy(poll_interval=0.01),
        )
        assert result.status is ExecutionStatus.SUCCESS
    finally:
        backend.close()


def test_a_failing_factory_is_reported():
    with pytest.raises(RuntimeError, match="failed to start"):
        RemoteBackend.spawn(MockBackend, no_such_argument=1)


# ---------------------------------------------------------------- safety


def test_a_killed_client_cancels_the_running_command():
    backend = MockBackend()
    with served(backend, timeout=0.5) as port:
        script = textwrap.dedent(
            f"""
            import time
            from long_tamp.execution import ExecutionCommand, RemoteBackend
            client = RemoteBackend(port={port})
            print(client.start(ExecutionCommand("s1", duration=600.0)), flush=True)
            time.sleep(600)
            """
        )
        child = subprocess.Popen(
            [sys.executable, "-c", script], stdout=subprocess.PIPE, text=True
        )
        try:
            assert "RUNNING" in child.stdout.readline()
            assert not backend.cancelled
            child.kill()
            killed = time.monotonic()
            assert _wait_for(lambda: backend.cancelled, timeout=0.5)
            assert time.monotonic() - killed < 0.5  # within one heartbeat period
        finally:
            child.kill()
            child.wait()


def test_a_silent_client_cancels_the_running_command():
    backend = MockBackend()
    with served(backend, timeout=0.3) as port:
        with socket.create_connection(("127.0.0.1", port)) as sock:
            remote.send_frame(sock, {"op": "hello", "version": remote.PROTOCOL_VERSION})
            assert remote.recv_frame(sock)["ok"]
            remote.send_frame(
                sock,
                {"op": "start", "args": remote.encode([ExecutionCommand("s", 60.0)])},
            )
            assert remote.recv_frame(sock)["ok"]
            # no poll from here on
            assert _wait_for(lambda: backend.cancelled, timeout=1.0)
            # still connected: a slow client learns of the cancel on its next poll
            remote.send_frame(sock, {"op": "poll", "args": []})
            status, feedback = remote.decode(remote.recv_frame(sock)["value"])
            assert status is ExecutionStatus.FAILURE and feedback.message == "cancelled"


def test_an_idle_client_may_be_silent():
    with served(MockBackend(), timeout=0.2) as port:
        with socket.create_connection(("127.0.0.1", port)) as sock:
            remote.send_frame(sock, {"op": "hello", "version": remote.PROTOCOL_VERSION})
            assert remote.recv_frame(sock)["ok"]
            time.sleep(0.6)  # planning, say
            remote.send_frame(sock, {"op": "ping", "args": []})
            assert remote.recv_frame(sock)["ok"]


def test_an_idle_client_is_kept_alive_by_pings():
    backend = MockBackend()
    with served(backend, timeout=0.3) as port:
        with RemoteBackend(port=port, keepalive=0.1) as client:
            assert client.start(ExecutionCommand("s", 60.0)) is ExecutionStatus.RUNNING
            time.sleep(0.8)
            assert client.connected and not backend.cancelled


def test_a_second_client_is_refused():
    with served(MockBackend()) as port, RemoteBackend(port=port):
        with pytest.raises(RuntimeError, match="busy"):
            RemoteBackend(port=port)


def test_a_lost_server_is_a_disconnected_failure():
    stop, ports = threading.Event(), queue.Queue()

    class Observed(MockBackend):
        def start_error(self, command):
            return 0.0

    thread = threading.Thread(
        target=serve_backend,
        args=(Observed(),),
        kwargs={"ready": ports.put, "stop": stop},
        daemon=True,
    )
    thread.start()
    client = RemoteBackend(port=ports.get(timeout=5))
    try:
        assert client.start(ExecutionCommand("s", 60.0)) is ExecutionStatus.RUNNING
        stop.set()
        thread.join(5)
        status, feedback = client.poll()
        assert status is ExecutionStatus.FAILURE
        assert feedback.message.startswith("disconnected")
        assert client.start(ExecutionCommand("s", 1.0)) is ExecutionStatus.FAILURE
        client.cancel()  # no-op, no error
        assert client.start_error(ExecutionCommand("s", payload=_Path())) is None
    finally:
        client.close()


def test_a_version_mismatch_is_refused():
    with served(MockBackend()) as port:
        with socket.create_connection(("127.0.0.1", port)) as sock:
            remote.send_frame(sock, {"op": "hello", "version": 999})
            reply = remote.recv_frame(sock)
    assert not reply["ok"] and "protocol" in reply["error"]
