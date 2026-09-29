"""The mission event stream (issue #11): IR ids and roles stamped on compiled
BT elements, and the Python runner's events in the schema the C++ host
writes. The C++ side is compared against these by the ``taskplan_bt_events``
CTest (``examples/behaviortree/check_events.py``)."""

import io
import json
import xml.etree.ElementTree as ET

import pytest

from long_tamp.execution import ExecutionCommand, ExecutionPolicy, MockBackend
from long_tamp.execution.executor import PlanExecutor
from long_tamp.tasks.task_planning.compiler import COMPILER_VERSION
from long_tamp.tasks.task_planning.events import (
    SCHEMA,
    TRANSACTION_ROLES,
    JsonlEventWriter,
    read_events,
)
from long_tamp.tasks.task_planning.host import create_fake_session
from long_tamp.tasks.task_planning.runner import run_plan

UNSTAMPED = {"root", "BehaviorTree", "SetupTaskPlan", "FinalizeTaskPlan"}


def _elements(session):
    return list(ET.fromstring(session.get_behavior_tree_xml()).iter())


def test_compiler_stamps_every_plan_element_with_its_ir_id_and_role():
    session = create_fake_session('{"shape": "composite"}')
    assert COMPILER_VERSION == "1.1"
    stamped = {}
    for element in _elements(session):
        if element.tag in UNSTAMPED or element.get("name") == "task-plan-root":
            assert "_ir_id" not in element.attrib
            continue
        key = (element.get("_ir_id"), element.get("_ir_role"))
        assert None not in key, element.tag
        assert key not in stamped, f"{key} stamped twice"
        stamped[key] = element.get("name")
    roles = {role for _, role in stamped}
    assert roles == set(TRANSACTION_ROLES) | {
        "sequence",
        "fallback",
        "retry",
        "condition",
    }
    assert stamped[("move-a", "attempts")] == "Move a retry"
    assert stamped[("retry-flaky", "retry")] == "retry-flaky"


def test_a_transactions_elements_all_carry_the_transaction_id():
    session = create_fake_session("{}")
    ids = {e.get("_ir_id") for e in _elements(session) if "_ir_id" in e.attrib}
    assert ids == {"move-home"}  # the operation inside is not a BT element


@pytest.mark.parametrize(
    "options",
    ["{}", '{"fault": "capability_raises"}', '{"shape": "composite"}'],
)
def test_python_events_name_the_compiled_elements(options):
    session = create_fake_session(options)
    names = {
        (e.get("_ir_id"), e.get("_ir_role")): e.get("name")
        for e in _elements(session)
        if "_ir_id" in e.attrib
    }
    events = []
    run_plan(session, on_event=events.append)
    assert events
    for event in events:
        assert event["schema"] == SCHEMA and event["source"] == "python"
        assert names[(event["ir_id"], event["role"])] == event["name"]


def test_composites_run_then_finish_and_leaves_just_finish():
    events = []
    run = run_plan(
        create_fake_session('{"shape": "composite"}'), on_event=events.append
    )
    assert run.success
    seen = [(e["ir_id"], e["role"], e["status"], e["previous"]) for e in events]
    assert seen[:3] == [
        ("mission", "sequence", "RUNNING", "IDLE"),
        ("reach-a", "fallback", "RUNNING", "IDLE"),
        ("at-a", "condition", "FAILURE", "IDLE"),
    ]
    assert seen[-1] == ("mission", "sequence", "SUCCESS", "RUNNING")
    executes = [
        e for e in events if e["ir_id"] == "move-flaky" and e["role"] == "execute"
    ]
    assert [e["status"] for e in executes] == ["FAILURE", "SUCCESS"]
    assert executes[0]["message"] == "synthetic first-attempt failure"
    assert [e["metrics"]["attempt"] for e in executes] == [1, 2]


def test_a_complete_step_reports_why_it_was_skipped():
    session = create_fake_session("{}")
    run_plan(session)  # completes move-home in this run
    events = []
    run_plan(session, on_event=events.append)
    assert [(e["role"], e["status"]) for e in events] == [
        ("transaction", "RUNNING"),
        ("complete", "SUCCESS"),
        ("transaction", "SUCCESS"),
    ]
    assert events[1]["message"] == "completed_this_run"


def test_jsonl_writer_round_trips(tmp_path):
    path = tmp_path / "events.jsonl"
    with JsonlEventWriter(path) as write:
        run_plan(create_fake_session("{}"), on_event=write)
        assert len(read_events(path)) == 9  # flushed per event, before close
    events = read_events(path)
    assert events[0]["name"] == "Move home transaction"
    stream = io.StringIO()
    JsonlEventWriter(stream)(events[0])
    assert json.loads(stream.getvalue()) == events[0]


def test_the_executor_adds_motion_events_with_metrics():
    session = create_fake_session("{}")
    events = []
    executor = PlanExecutor(
        session,
        backend=MockBackend(rtf=1000.0),
        policy=ExecutionPolicy(poll_interval=0.001),
        on_event=events.append,
    )
    execute_step = session.execute_step

    def execute_and_submit(step_id):  # a capability submitting its motion
        executor.submit(ExecutionCommand(step_id, duration=1.0))
        return execute_step(step_id)

    session.execute_step = execute_and_submit
    run = executor.run()
    assert run.success
    roles = [(e["role"], e["status"]) for e in events]
    i = roles.index(("motion", "RUNNING"))
    assert roles[i + 1] == ("motion", "SUCCESS")
    assert roles[i + 2] == ("execute", "SUCCESS")  # motion is part of execute
    metrics = events[i + 1]["metrics"]
    assert metrics["duration"] == 1.0 and metrics["feedback_count"] >= 1
