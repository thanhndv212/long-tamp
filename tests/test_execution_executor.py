"""The Python TaskPlan executor (issue #9): plan steps, then execute their motion
on a backend under the execution contract, with step-boundary control."""

import threading

from long_tamp.execution import (
    ExecutionCommand,
    ExecutionControl,
    ExecutionPolicy,
    MockBackend,
)
from long_tamp.execution.executor import PlanExecutor
from long_tamp.tasks.task_planning import (
    CapabilityDescriptor,
    CapabilityRegistry,
    TaskPlan,
    TaskPlanningSession,
)


def _transaction(node_id, capability, **parameters):
    return {
        "type": "transaction",
        "id": node_id,
        "label": node_id,
        "restart_state": ["q_current"],
        "children": [
            {
                "type": "operation",
                "id": f"{node_id}.execute",
                "capability": capability,
                "parameters": parameters,
            }
        ],
    }


def _executor(world, backend=None, control=None, motions=1, fail_plan=False):
    """grasp/release capabilities that 'plan' by updating ``world`` and submit
    ``motions`` commands each to the executor."""
    holder = {}
    planned = []

    def grasp(p):
        if fail_plan:
            raise RuntimeError("no path")
        planned.append(("grasp", p["handle"]))
        world.add(f"holds({p['gripper']}, {p['handle']})")
        for i in range(motions):
            holder["executor"].submit(ExecutionCommand(f"grasp-{i}", duration=1.0))
        return {}

    def release(p):
        planned.append(("release", p["gripper"]))
        world.difference_update(
            {a for a in world if a.startswith(f"holds({p['gripper']},")}
        )
        holder["executor"].submit(ExecutionCommand("release", duration=1.0))
        return {}

    registry = CapabilityRegistry()
    registry.register(
        CapabilityDescriptor(
            "grasp",
            "1.0",
            {"gripper": str, "handle": str},
            preconditions=("not holds(?gripper, _)",),
            effects=("holds(?gripper, ?handle)",),
            restartable=True,
        ),
        grasp,
    )
    registry.register(
        CapabilityDescriptor(
            "release",
            "1.0",
            {"gripper": str},
            preconditions=("holds(?gripper, _)",),
            effects=("not holds(?gripper, _)",),
            restartable=True,
        ),
        release,
    )
    document = {
        "schema_version": "1.0",
        "mission_id": "exec-demo",
        "scene": {"id": "fake"},
        "provenance": {"kind": "human", "generator": "test"},
        "root": {
            "type": "sequence",
            "id": "root",
            "children": [
                _transaction("grasp-1", "grasp", gripper="left", handle="ball"),
                _transaction("release-1", "release", gripper="left"),
            ],
        },
    }
    plan = TaskPlan.from_dict(document, registry)
    session = TaskPlanningSession(plan, registry, world_state=lambda: set(world))
    executor = PlanExecutor(
        session,
        backend=backend,
        policy=ExecutionPolicy(poll_interval=0.001, busy_backoff=0.001),
        control=control,
    )
    holder["executor"] = executor
    return executor, planned


def test_without_a_backend_it_only_plans():
    executor, planned = _executor(set())
    run = executor.run()
    assert run.success
    assert planned == [("grasp", "ball"), ("release", "left")]
    assert run.executions == []  # submitted commands are dropped, not run


def test_each_step_executes_its_motion_on_the_backend():
    backend = MockBackend(rtf=1000.0)  # fast: 1 s of motion in 1 ms
    executor, _ = _executor(set(), backend=backend, motions=2)
    run = executor.run()
    assert run.success
    assert [e.step_id for e in run.executions] == ["grasp-1", "grasp-1", "release-1"]
    assert all(e.result.status == "success" for e in run.executions)


def test_a_failed_execution_fails_its_step_and_stops_the_plan():
    backend = MockBackend(rtf=1.0, fail_at=0.0, message="collision detected")
    executor, planned = _executor(set(), backend=backend)
    run = executor.run()
    assert not run.success
    assert run.failed_step == "grasp-1"
    assert "collision detected" in run.message
    assert planned == [("grasp", "ball")]  # release never planned


def test_a_skipped_step_executes_nothing():
    backend = MockBackend(rtf=1000.0)
    executor, planned = _executor({"holds(left, ball)"}, backend=backend)
    run = executor.run()
    assert run.success and run.skipped == ["grasp-1"]
    assert [e.step_id for e in run.executions] == ["release-1"]


def test_commands_from_a_failed_planning_attempt_are_discarded():
    backend = MockBackend(rtf=1000.0)
    executor, _ = _executor(set(), backend=backend, fail_plan=True)
    run = executor.run()
    assert not run.success and run.executions == []


def test_breakpoint_pauses_between_steps_and_resume_continues():
    control = ExecutionControl()
    control.add_breakpoint("release-1", "before")
    backend = MockBackend(rtf=1000.0)
    executor, planned = _executor(set(), backend=backend, control=control)
    result = {}
    thread = threading.Thread(target=lambda: result.setdefault("run", executor.run()))
    thread.start()
    for _ in range(200):
        if control.waiting_at == ("release-1", "before"):
            break
        thread.join(0.01)
    assert control.waiting_at == ("release-1", "before")
    assert planned == [("grasp", "ball")]  # paused before release was planned
    control.resume()
    thread.join(5.0)
    assert result["run"].success
    assert planned == [("grasp", "ball"), ("release", "left")]


def test_stop_at_a_boundary_ends_the_run():
    control = ExecutionControl()
    control.add_breakpoint("release-1", "before")
    executor, planned = _executor(
        set(), backend=MockBackend(rtf=1000.0), control=control
    )
    result = {}
    thread = threading.Thread(target=lambda: result.setdefault("run", executor.run()))
    thread.start()
    for _ in range(200):
        if control.waiting_at is not None:
            break
        thread.join(0.01)
    control.stop()
    thread.join(5.0)
    run = result["run"]
    assert not run.success and run.failed_step == "release-1"
    assert run.message == "stopped"
    assert planned == [("grasp", "ball")]
