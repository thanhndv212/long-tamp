"""Effect-based completion: a step is done when its effect holds (issue #4, ADR-0002).

With a world state, "is this transaction complete?" is answered from the
world, not from an in-memory set: a step whose effect already holds is
skipped without running, and one whose effect was undone runs again.
"""

import json

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


def _session(world, use_world=True, grasp_effects=("holds(?gripper, ?handle)",)):
    """A grasp/release session over a mutable ``world`` (a set of atom strings)."""
    calls = []

    def grasp(parameters):
        calls.append(("grasp", parameters["handle"]))
        world.add(f"holds({parameters['gripper']}, {parameters['handle']})")
        return {}

    def release(parameters):
        calls.append(("release", parameters["gripper"]))
        world.difference_update(
            {a for a in world if a.startswith(f"holds({parameters['gripper']},")}
        )
        return {}

    registry = CapabilityRegistry()
    registry.register(
        CapabilityDescriptor(
            "grasp",
            "1.0",
            {"gripper": str, "handle": str},
            effects=grasp_effects,
            restartable=True,
        ),
        grasp,
    )
    registry.register(
        CapabilityDescriptor(
            "release",
            "1.0",
            {"gripper": str},
            effects=("not holds(?gripper, _)",),
            restartable=True,
        ),
        release,
    )
    document = {
        "schema_version": "1.0",
        "mission_id": "guard-demo",
        "scene": {"id": "fake"},
        "provenance": {"kind": "human", "generator": "test"},
        "root": {
            "type": "sequence",
            "id": "root",
            "children": [
                _transaction("grasp-ball", "grasp", gripper="left", handle="ball"),
                _transaction("release-left", "release", gripper="left"),
            ],
        },
    }
    plan = TaskPlan.from_dict(document, registry)
    session = TaskPlanningSession(
        plan, registry, world_state=(lambda: set(world)) if use_world else None
    )
    return session, calls


def _complete(session, step_id):
    return json.loads(session.is_step_complete(step_id))


def test_a_step_whose_effect_already_holds_is_skipped_without_running():
    world = {"holds(left, ball)"}
    session, calls = _session(world)
    assert _complete(session, "grasp-ball") == {
        "status": "success",
        "complete": True,
        "reason": "effect_holds",
    }
    result = json.loads(session.execute_step("grasp-ball"))
    assert result["status"] == "skipped"
    assert result["message"] == "effect already holds"
    assert calls == []


def test_a_step_runs_when_its_effect_does_not_hold_then_counts_as_done():
    world = set()
    session, calls = _session(world)
    assert _complete(session, "grasp-ball")["complete"] is False
    assert json.loads(session.execute_step("grasp-ball"))["status"] == "success"
    assert calls == [("grasp", "ball")]
    assert _complete(session, "grasp-ball")["complete"] is True


def test_a_step_whose_effect_was_undone_runs_again():
    world = set()
    session, calls = _session(world)
    session.execute_step("grasp-ball")
    world.clear()  # e.g. the part slipped out of the gripper
    assert _complete(session, "grasp-ball")["complete"] is False
    assert json.loads(session.execute_step("grasp-ball"))["status"] == "success"
    assert calls == [("grasp", "ball"), ("grasp", "ball")]


def test_negative_effects_count_as_holding_when_nothing_matches():
    world = set()  # the left gripper already holds nothing
    session, calls = _session(world)
    assert json.loads(session.execute_step("release-left"))["status"] == "skipped"
    assert calls == []


def test_without_a_world_state_completion_stays_in_memory():
    world = {"holds(left, ball)"}
    session, calls = _session(world, use_world=False)
    assert _complete(session, "grasp-ball") == {
        "status": "success",
        "complete": False,
        "reason": "not_completed",
    }
    session.execute_step("grasp-ball")
    assert _complete(session, "grasp-ball")["reason"] == "completed_this_run"
    assert json.loads(session.execute_step("grasp-ball"))["status"] == "skipped"
    assert calls == [("grasp", "ball")]


def test_a_step_without_declared_effects_falls_back_to_memory():
    world = {"holds(left, ball)"}
    session, calls = _session(world, grasp_effects=())
    assert _complete(session, "grasp-ball")["complete"] is False
    session.execute_step("grasp-ball")
    assert _complete(session, "grasp-ball")["reason"] == "completed_this_run"


def test_a_failing_world_state_never_runs_the_step():
    session, calls = _session(set())

    def broken():
        raise RuntimeError("tracker unavailable")

    session.world_state = broken
    complete = _complete(session, "grasp-ball")
    assert complete["status"] == "failure"
    assert complete["complete"] is False
    result = json.loads(session.execute_step("grasp-ball"))
    assert result["status"] == "failure"
    assert "tracker unavailable" in result["message"]
    assert calls == []
