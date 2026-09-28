"""run_plan: the compiled BT's semantics, run synchronously from Python."""

from long_tamp.tasks.task_planning import (
    CapabilityDescriptor,
    CapabilityRegistry,
    TaskPlan,
    TaskPlanningSession,
)
from long_tamp.tasks.task_planning.runner import run_plan


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


def _session(world, root, failures=None):
    """Grasp/release over a mutable ``world`` set; ``failures`` counts down."""
    ran = []
    failures = failures or {}

    def grasp(p):
        ran.append(("grasp", p["handle"]))
        if failures.get("grasp", 0) > 0:
            failures["grasp"] -= 1
            raise RuntimeError("draw failed")
        world.add(f"holds({p['gripper']}, {p['handle']})")

    def release(p):
        ran.append(("release", p["gripper"]))
        world.difference_update(
            {a for a in world if a.startswith(f"holds({p['gripper']},")}
        )

    registry = CapabilityRegistry()
    registry.register(
        CapabilityDescriptor(
            "grasp",
            "1.0",
            {"gripper": str, "handle": str},
            preconditions=("not holds(?gripper, _)",),
            effects=("holds(?gripper, ?handle)",),
            max_attempts=3,
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
    registry.register(
        CapabilityDescriptor(
            "empty", "1.0", {"gripper": str}, preconditions=("not holds(?gripper, _)",)
        ),
        lambda p: not any(a.startswith(f"holds({p['gripper']},") for a in world),
    )
    document = {
        "schema_version": "1.0",
        "mission_id": "runner-demo",
        "scene": {"id": "fake"},
        "provenance": {"kind": "human", "generator": "test"},
        "root": root,
    }
    plan = TaskPlan.from_dict(document, registry)
    return TaskPlanningSession(plan, registry, world_state=lambda: set(world)), ran


def _regrasp():
    return {
        "type": "sequence",
        "id": "root",
        "children": [
            _transaction("grasp-1", "grasp", gripper="left", handle="ball"),
            {
                "type": "fallback",
                "id": "cycled",
                "label": "cycled",
                "children": [
                    {
                        "type": "condition",
                        "id": "empty",
                        "label": "gripper empty",
                        "capability": "empty",
                        "parameters": {"gripper": "left"},
                    },
                    {
                        "type": "sequence",
                        "id": "cycle",
                        "children": [
                            _transaction("release-1", "release", gripper="left"),
                            _transaction(
                                "grasp-2", "grasp", gripper="left", handle="cup"
                            ),
                        ],
                    },
                ],
            },
        ],
    }


def test_nominal_run_follows_the_tree():
    session, ran = _session(set(), _regrasp())
    run = run_plan(session)
    assert run.success and run.skipped == []
    assert ran == [("grasp", "ball"), ("release", "left"), ("grasp", "cup")]


def test_work_whose_effect_holds_is_skipped_and_reported():
    skips = []
    session, ran = _session({"holds(left, ball)"}, _regrasp())
    run = run_plan(session, on_skip=lambda label, why: skips.append((label, why)))
    assert run.success
    assert run.skipped == ["grasp-1"]
    assert skips == [("grasp-1", "effect_holds")]
    assert ran == [("release", "left"), ("grasp", "cup")]


def test_a_transaction_is_retried_up_to_its_attempts():
    session, ran = _session(set(), _regrasp(), failures={"grasp": 2})
    assert run_plan(session).success
    assert ran[:3] == [("grasp", "ball")] * 3


def test_exhausted_attempts_stop_the_run_with_the_step():
    session, ran = _session(set(), _regrasp(), failures={"grasp": 3})
    run = run_plan(session)
    assert not run.success
    assert run.failed_step == "grasp-1"
    assert "draw failed" in run.message


def test_unready_step_stops_the_run_with_the_unsatisfied_literals():
    root = {
        "type": "sequence",
        "id": "root",
        "children": [_transaction("release-1", "release", gripper="left")],
    }
    # release's effect (empty gripper) holds on an empty gripper: skipped.
    session, _ = _session(set(), root)
    assert run_plan(session).skipped == ["release-1"]
    # grasping with the gripper full (and not already done): not ready.
    root["children"] = [_transaction("grasp-2", "grasp", gripper="left", handle="cup")]
    session, ran = _session({"holds(left, ball)"}, root)
    run = run_plan(session)
    assert not run.success and ran == []
    assert run.message == "not ready: not holds(left, _)"
