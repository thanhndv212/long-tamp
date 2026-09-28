"""The execution contract (issue #8, ADR-0004): statuses, heartbeats, timeouts,
BUSY retries, pause/resume and breakpoints. Pure Python, with a fake clock, so
every timeout is deterministic and instant."""

import threading

import pytest

from long_tamp.execution import (
    ExecutionCommand,
    ExecutionControl,
    ExecutionPolicy,
    ExecutionStatus,
    MockBackend,
    run_command,
)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def _run(backend, duration=2.0, clock=None, **policy):
    clock = clock or FakeClock()
    backend.clock = clock
    return run_command(
        backend,
        ExecutionCommand(step_id="s1", duration=duration),
        ExecutionPolicy(**policy),
        clock=clock,
        sleep=clock.sleep,
    )


def test_a_command_runs_to_success_with_heartbeats():
    result = _run(MockBackend(), duration=2.0)
    assert result.status is ExecutionStatus.SUCCESS
    assert result.feedback_count >= 1
    assert result.elapsed == pytest.approx(2.0, abs=0.2)


def test_failure_is_passed_through():
    result = _run(MockBackend(fail_at=1.0, message="joint limit"))
    assert result.status is ExecutionStatus.FAILURE
    assert "joint limit" in result.message


def test_busy_is_retried_not_failed():
    """The DBT lesson: a busy backend is not a failed command."""
    backend = MockBackend(busy_starts=3)
    result = _run(backend, busy_retries=5, busy_backoff=0.5)
    assert result.status is ExecutionStatus.SUCCESS
    assert result.busy_retries == 3


def test_busy_forever_fails_with_a_clear_reason():
    result = _run(MockBackend(busy_starts=100), busy_retries=4, busy_backoff=0.5)
    assert result.status is ExecutionStatus.FAILURE
    assert result.reason == "busy"
    assert result.busy_retries == 4


def test_silence_beyond_the_inactivity_timeout_cancels_the_command():
    backend = MockBackend(stall_at=1.0)  # stops sending feedback, never finishes
    result = _run(backend, duration=10.0, inactivity_timeout=3.0)
    assert result.status is ExecutionStatus.FAILURE
    assert result.reason == "inactivity"
    assert backend.cancelled
    assert result.elapsed == pytest.approx(4.0, abs=0.3)  # 1 s running + 3 s silence


def test_heartbeats_keep_a_long_command_alive():
    """A long trajectory that keeps reporting progress must not time out on
    inactivity, only on its duration-scaled deadline."""
    result = _run(MockBackend(), duration=60.0, inactivity_timeout=3.0)
    assert result.status is ExecutionStatus.SUCCESS


def test_the_deadline_scales_with_the_trajectory_duration():
    """The JTC lesson: a slow simulator (real-time factor 0.4) legitimately
    takes 2.5x the trajectory's duration."""
    slow = MockBackend(rtf=0.4)
    policy = dict(min_real_time_factor=0.3, deadline_margin=5.0)
    assert _run(slow, duration=10.0, **policy).status is ExecutionStatus.SUCCESS
    too_slow = MockBackend(rtf=0.2)
    result = _run(too_slow, duration=10.0, **policy)
    assert result.status is ExecutionStatus.FAILURE
    assert result.reason == "deadline"
    assert too_slow.cancelled
    assert result.elapsed == pytest.approx(10.0 / 0.3 + 5.0, abs=0.3)


def test_the_deadline_is_reported_by_the_policy():
    policy = ExecutionPolicy(min_real_time_factor=0.5, deadline_margin=10.0)
    assert policy.deadline(ExecutionCommand("s", duration=20.0)) == 50.0
    assert policy.deadline(ExecutionCommand("s", duration=None)) is None


def test_a_stop_request_cancels_a_running_command():
    control = ExecutionControl()
    backend = MockBackend()
    clock = FakeClock()
    backend.clock = clock

    def sleep(seconds):
        clock.sleep(seconds)
        if clock.now >= 1.0:
            control.stop()

    result = run_command(
        backend,
        ExecutionCommand("s1", duration=5.0),
        ExecutionPolicy(),
        control=control,
        clock=clock,
        sleep=sleep,
    )
    assert result.status is ExecutionStatus.FAILURE
    assert result.reason == "stopped"
    assert backend.cancelled


# ------------------------------------------------------------- step boundaries


def test_pause_blocks_at_the_next_step_boundary_until_resume():
    control = ExecutionControl()
    control.pause()
    passed = threading.Event()

    def executor():
        control.checkpoint("s2", "before")  # blocks while paused
        passed.set()

    thread = threading.Thread(target=executor)
    thread.start()
    assert not passed.wait(0.2)
    assert control.waiting_at == ("s2", "before")
    control.resume()
    assert passed.wait(2.0)
    thread.join()
    assert control.waiting_at is None


def test_breakpoints_pause_before_or_after_a_given_step():
    control = ExecutionControl()
    control.add_breakpoint("s2", "after")
    control.checkpoint("s1", "after")  # no breakpoint: returns immediately
    control.checkpoint("s2", "before")  # breakpoint is "after", not "before"
    reached = threading.Event()

    def executor():
        control.checkpoint("s2", "after")
        reached.set()

    thread = threading.Thread(target=executor)
    thread.start()
    assert not reached.wait(0.2)
    assert control.paused and control.waiting_at == ("s2", "after")
    control.resume()
    assert reached.wait(2.0)
    thread.join()


def test_stop_releases_a_paused_executor_and_reports_it():
    control = ExecutionControl()
    control.pause()
    outcome = []

    def executor():
        outcome.append(control.checkpoint("s3", "before"))

    thread = threading.Thread(target=executor)
    thread.start()
    control.stop()
    thread.join(2.0)
    assert outcome == [False]  # False: stop, don't start the step
    assert control.stopped


def test_mock_backend_covers_every_status():
    seen = set()
    for backend in (
        MockBackend(),
        MockBackend(fail_at=0.5),
        MockBackend(busy_starts=1),
    ):
        clock = FakeClock()
        backend.clock = clock
        seen.add(backend.start(ExecutionCommand("s", duration=1.0)))
        status, _ = backend.poll()
        seen.add(status)
        clock.now = 2.0
        seen.add(backend.poll()[0])
    assert seen >= {
        ExecutionStatus.RUNNING,
        ExecutionStatus.SUCCESS,
        ExecutionStatus.FAILURE,
        ExecutionStatus.BUSY,
    }


# ----------------------------------------------------------- path playback


class _Path:
    """A 1-D straight path over 2 s of path time."""

    def length(self):
        return 2.0

    def eval(self, t):
        return [t / 2.0], True


def test_path_playback_plays_the_path_over_its_duration():
    from long_tamp.execution.playback import PathPlaybackBackend

    shown = []
    clock = FakeClock()
    backend = PathPlaybackBackend(display=shown.append, speed=1.0, clock=clock)
    result = run_command(
        backend,
        ExecutionCommand("s1", duration=2.0, payload=_Path()),
        ExecutionPolicy(poll_interval=0.5),
        clock=clock,
        sleep=clock.sleep,
    )
    assert result.status is ExecutionStatus.SUCCESS
    assert shown[0] == [0.0] and shown[-1] == [1.0]
    assert result.elapsed == pytest.approx(2.0, abs=0.5)


def test_path_playback_resolves_ids_and_rejects_unknown_ones():
    from long_tamp.execution.playback import PathPlaybackBackend

    paths = {7: _Path()}
    backend = PathPlaybackBackend(get_path=paths.get, speed=100.0)
    assert backend.start(ExecutionCommand("s", payload=7)) is ExecutionStatus.RUNNING
    assert backend.start(ExecutionCommand("s", payload=8)) is ExecutionStatus.FAILURE
