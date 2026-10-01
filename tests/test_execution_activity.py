"""Progress from a step's planning and operator interventions mid-step (#108):
``long_tamp.execution.activity`` and the in-step requests of
``ExecutionControl``."""

import threading
import time
from types import SimpleNamespace

import pytest

from long_tamp.execution import ExecutionControl, MockBackend, activity
from long_tamp.execution.executor import PlanExecutor
from long_tamp.tasks.block_recovery import (
    make_lookahead_hints_factory,
    run_block_with_recovery,
)
from long_tamp.tasks.task_planning.host import create_fake_session


@pytest.fixture(autouse=True)
def _no_step():
    activity.end()
    yield
    activity.end()


def test_outside_a_step_everything_is_a_no_op():
    activity.progress("nothing listens")
    activity.checkpoint("search")
    with activity.searching("x"):
        activity.checkpoint("search")
    assert activity.current_step() is None


def test_progress_names_the_step_and_its_search():
    events, control = [], ExecutionControl()
    activity.begin("s1", "Step one", events.append, control)
    activity.progress("phase 1/5", phase=1)
    with activity.searching("a clamp pose"):
        activity.progress("12 rejected", rejected=12)
    assert [(e["ir_id"], e["role"], e["name"], e["status"]) for e in events] == [
        ("s1", "progress", "Step one", "RUNNING")
    ] * 2
    assert events[0]["metrics"]["phase"] == 1 and "search" not in events[0]["metrics"]
    assert events[1]["metrics"]["search"] == "a clamp pose"
    assert events[1]["metrics"]["elapsed"] >= 0


def test_requests_act_at_the_right_checkpoints():
    control = ExecutionControl()
    activity.begin("s", control=control)
    control.request("skip")
    activity.checkpoint("step")  # not a search: a skip waits
    activity.checkpoint("search")  # nor outside one
    assert control.pending == ("skip", "operator")
    with activity.searching("x"):
        with pytest.raises(activity.SearchSkipped):
            activity.checkpoint("search")
    assert control.pending is None  # taken
    control.request("abort_step", "watchdog")
    with pytest.raises(activity.StepAborted) as aborted:
        activity.checkpoint("step")
    assert aborted.value.reason == "watchdog"
    # it gets through the planner's "except Exception" retries
    assert not isinstance(aborted.value, Exception)
    with pytest.raises(ValueError):
        control.request("jump")
    control.request("skip")
    control.reset()
    assert control.pending is None


def _executor(session, control, events, plan_step):
    """An executor whose capability for ``move-flaky`` runs ``plan_step``."""
    execute_step = session.execute_step

    def execute(step_id):
        if step_id == "move-flaky":
            plan_step()
        return execute_step(step_id)

    session.execute_step = execute
    return PlanExecutor(
        session,
        backend=MockBackend(rtf=1000.0),
        control=control,
        on_event=events.append,
    )


def test_an_aborted_step_fails_once_and_stops_the_run():
    session = create_fake_session('{"shape": "composite"}')
    control, events = ExecutionControl(), []
    calls = []

    def plan_step():  # a long planning loop, aborted from outside
        calls.append(1)
        activity.progress("searching")
        control.request("abort_step")
        for _ in range(3):
            activity.checkpoint("step")
            time.sleep(0.01)
        raise AssertionError("not aborted")

    run = _executor(session, control, events, plan_step).run()
    assert not run.success and run.failed_step == "move-flaky"
    assert len(calls) == 1  # neither the attempts nor the retry node re-plan it
    executes = [
        e for e in events if e["ir_id"] == "move-flaky" and e["role"] == "execute"
    ]
    assert [e["status"] for e in executes] == ["FAILURE"]
    assert executes[0]["message"] == "aborted (abort_step by operator)"
    assert [e["message"] for e in events if e["role"] == "progress"] == ["searching"]
    assert control.stopped and control.pending is None
    assert activity.current_step() is None


def test_a_request_made_between_steps_is_dropped():
    session = create_fake_session("{}")
    control = ExecutionControl()
    control.request("abort_step")  # nothing was planning
    run = PlanExecutor(session, backend=MockBackend(rtf=1000.0), control=control).run()
    assert run.success


def test_a_skip_ends_the_lookahead_and_the_block_plans_unhinted():
    control = ExecutionControl()
    events = []
    activity.begin("b", sink=events.append, control=control)
    searched = []

    def find(**kw):
        searched.append(1)
        with activity.searching("a target"):
            control.request("skip")
            activity.checkpoint("search")
        return None

    planner = SimpleNamespace(find_feasible_phase_target=find)
    factory = make_lookahead_hints_factory(
        planner, [("g", "h1"), ("g", "h2")], [0.0], max_rounds=5, verbose=False
    )
    assert factory() is None and len(searched) == 1  # no further rounds
    assert events[-1]["message"] == "lookahead skipped: planning without a hint"


class _Planner:
    def __init__(self, on_plan):
        self.on_plan, self.resets = on_plan, 0
        self.invalidated_phase_hints = set()

    def plan_sequence(self, **kw):
        return self.on_plan()

    def reset_grasp_tracker_to_call_start(self):
        self.resets += 1


def test_an_abort_mid_block_rolls_its_grasps_back():
    def abort():
        raise activity.StepAborted("operator")

    planner = _Planner(abort)
    with pytest.raises(activity.StepAborted):
        run_block_with_recovery(planner, [("g", "h")], [0.0], verbose=False)
    assert planner.resets == 1


def test_an_abort_before_the_block_planned_leaves_the_tracker_alone():
    def hints():
        raise activity.StepAborted("operator")

    planner = _Planner(lambda: {"success": True, "final_config": [0.0]})
    with pytest.raises(activity.StepAborted):
        run_block_with_recovery(
            planner, [("g", "h")], [0.0], hints_factory=hints, verbose=False
        )
    assert planner.resets == 0  # its "call start" is an earlier block's


def test_requests_from_another_thread_reach_a_planning_loop():
    control, seen = ExecutionControl(), []
    activity.begin("s", control=control)
    threading.Timer(0.05, control.request, args=("abort_step", "operator")).start()
    with pytest.raises(activity.StepAborted):
        for _ in range(200):
            activity.checkpoint("step")
            seen.append(1)
            time.sleep(0.01)
    assert 0 < len(seen) < 200
