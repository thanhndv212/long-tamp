"""Concurrent execution of a parallel group's lanes (#21): merged motion."""

import numpy as np
import pytest

from long_tamp.execution import ExecutionCommand, ExecutionPolicy, MockBackend
from long_tamp.execution.concurrent import merge_lanes
from long_tamp.execution.executor import PlanExecutor
from long_tamp.tasks.task_planning import (
    CapabilityDescriptor,
    CapabilityRegistry,
    TaskPlan,
    TaskPlanningSession,
    parallelize,
)
from long_tamp.tasks.task_planning.skills import SkillCommand, SkillSpec


class Line:
    """Configuration ``a`` to ``b`` over ``T`` seconds."""

    def __init__(self, a, b, T=1.0):
        self.a, self.b, self.T = np.array(a, float), np.array(b, float), T

    def length(self):
        return self.T

    def eval(self, t):
        u = min(max(t / self.T, 0.0), 1.0)
        return self.a + (self.b - self.a) * u, True


def _cmd(name, a, b, T=1.0):
    return ExecutionCommand(name, T, Line(a, b, T))


# Configuration: [left arm, right arm, left's object, unmoved]. The left lane
# is planned first, so the right lane's paths see the left arm at its end.
LEFT = [_cmd("l1", [0, 0, 5, 9], [1, 0, 6, 9]), _cmd("l2", [1, 0, 6, 9], [2, 0, 6, 9])]
RIGHT = [_cmd("r1", [2, 0, 6, 9], [2, 3, 6, 9], T=2.0)]


def test_lanes_merge_entry_by_entry():
    merged = merge_lanes([LEFT, RIGHT], step_id="g")
    assert [c.step_id for c in merged] == ["g#1", "g#2"]
    assert merged[0].duration == 2.0  # the longest of the round
    first = merged[0].payload
    np.testing.assert_allclose(first.eval(0.5)[0], [0.5, 0.75, 5.5, 9])
    np.testing.assert_allclose(first.eval(2.0)[0], [1, 3, 6, 9])
    # the right lane is done: its arm holds where it ended
    np.testing.assert_allclose(merged[1].payload.eval(0.5)[0], [1.5, 3, 6, 9])


def test_lanes_moving_the_same_entry_are_not_merged():
    clash = [_cmd("r1", [2, 0, 6, 9], [2, 3, 7, 9])]  # moves left's object
    assert merge_lanes([LEFT, clash]) is None


def test_a_skill_runs_alone():
    spec = SkillSpec("screw", ("tool",))
    skill = ExecutionCommand(
        "s", 1.0, SkillCommand(spec, {"tool": "d"}, RIGHT[0].payload)
    )
    assert merge_lanes([LEFT, [skill]]) is None


def test_a_collision_along_the_merged_motion_refuses_the_merge():
    # both arms past 0.8 at once: in collision
    assert (
        merge_lanes([LEFT, RIGHT], validate=lambda q: not (q[0] > 0.8 and q[1] > 0.8))
        is None
    )
    assert merge_lanes([LEFT, RIGHT], validate=lambda q: True) is not None


# -- in the executor ---------------------------------------------------------


def _mission(backend, validate=None, plan_ahead=False, concurrent=True):
    """Left arm: grasp then move; right arm: move. The right arm's move is
    independent of the left arm's steps."""
    q = np.array([0.0, 0.0, 5.0, 9.0])
    holder, planned = {}, []

    def move(entry, target, name):
        def run(p):
            a = q.copy()
            q[entry] = target
            if entry == 0 and p.get("carry"):
                q[2] += 1.0
            planned.append(name)
            holder["executor"].submit(ExecutionCommand(name, 1.0, Line(a, q.copy())))

        return run

    registry = CapabilityRegistry()
    for name, arm, entry, target in (
        ("left_a", "left", 0, 1.0),
        ("left_b", "left", 0, 2.0),
        ("right_a", "right", 1, 3.0),
    ):
        registry.register(
            CapabilityDescriptor(
                name, "1.0", {"arm": str}, resources=("arm",), restartable=True
            ),
            move(entry, target, name),
        )
    steps = [
        ("s1", "left_a", "left"),
        ("s2", "right_a", "right"),
        ("s3", "left_b", "left"),
    ]
    root = {
        "type": "sequence",
        "id": "root",
        "children": [
            {
                "type": "transaction",
                "id": sid,
                "label": sid,
                "restart_state": ["q_current"],
                "children": [
                    {
                        "type": "operation",
                        "id": f"{sid}.execute",
                        "capability": cap,
                        "parameters": {"arm": arm},
                    }
                ],
            }
            for sid, cap, arm in steps
        ],
    }
    document = {
        "schema_version": "1.0",
        "mission_id": "concurrent-demo",
        "scene": {"id": "fake"},
        "provenance": {"kind": "planner", "generator": "test"},
        "root": root,
    }
    plan = TaskPlan.from_dict(parallelize(document, registry), registry)
    session = TaskPlanningSession(plan, registry)
    executor = PlanExecutor(
        session,
        backend=backend,
        policy=ExecutionPolicy(poll_interval=0.001, busy_backoff=0.001),
        plan_ahead=plan_ahead,
        concurrent=concurrent,
        validate_config=validate,
    )
    holder["executor"] = executor
    return executor, planned


@pytest.mark.parametrize("plan_ahead", [False, True])
def test_a_parallel_group_runs_as_merged_motion(plan_ahead):
    executor, planned = _mission(MockBackend(rtf=1000.0), plan_ahead=plan_ahead)
    run = executor.run()
    assert run.success, run.message
    # planned lane by lane: the left lane (s1, s3), then the right (s2)
    assert planned == ["left_a", "left_b", "right_a"]
    # two merged commands (the left lane has two, the right one) instead of three
    assert [e.command.step_id.split("#")[-1] for e in run.executions] == ["1", "2"]
    assert run.timing["merged_groups"] == 1


def test_a_group_that_would_collide_runs_one_step_after_another():
    executor, _ = _mission(MockBackend(rtf=1000.0), validate=lambda q: q[0] < 0.5)
    run = executor.run()
    assert run.success, run.message
    assert [e.command.step_id for e in run.executions] == [
        "left_a",
        "left_b",
        "right_a",
    ]
    assert run.timing["sequential_groups"] == 1


def test_without_concurrency_steps_run_in_plan_order_one_by_one():
    executor, _ = _mission(MockBackend(rtf=1000.0), concurrent=False)
    run = executor.run()
    assert run.success
    assert len(run.executions) == 3 and run.timing["merged_groups"] == 0
