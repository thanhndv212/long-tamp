import pytest

from agimus_spacelab.tasks.task_planning import (
    CapabilityDescriptor,
    CapabilityRegistry,
    PlanValidationError,
    TaskPlan,
)


def _plan(operation=None):
    operation = operation or {
        "type": "operation",
        "id": "pick",
        "capability": "plan_grasp",
        "parameters": {"gripper": "robot/gripper", "handle": "part/handle"},
    }
    return {
        "schema_version": "1.0",
        "mission_id": "pick-part",
        "scene": {"id": "test-scene", "config_size": 2},
        "root": {"type": "sequence", "id": "root", "children": [operation]},
        "provenance": {"kind": "human", "generator": "test"},
    }


def _registry():
    registry = CapabilityRegistry()
    registry.register(
        CapabilityDescriptor(
            capability_id="plan_grasp",
            version="1.0",
            required_parameters={"gripper": str, "handle": str},
            resources=("robot",),
            effects=("grasp_state",),
            safety_class="planning-only",
            max_attempts=10,
            max_timeout=30.0,
            restartable=True,
        ),
        lambda parameters: parameters,
    )
    return registry


def test_task_plan_normalization_and_fingerprint_are_deterministic():
    first = TaskPlan.from_dict(_plan(), _registry())
    reordered = {
        "root": _plan()["root"],
        "provenance": {"generator": "test", "kind": "human"},
        "scene": {"config_size": 2, "id": "test-scene"},
        "mission_id": "pick-part",
        "schema_version": "1.0",
    }
    second = TaskPlan.from_dict(reordered, _registry())

    assert first.canonical_json == second.canonical_json
    assert first.plan_fingerprint == second.plan_fingerprint


def test_duplicate_node_ids_are_rejected():
    document = _plan()
    document["root"]["children"].append(
        {
            "type": "condition",
            "id": "pick",
            "capability": "plan_grasp",
            "parameters": {"gripper": "robot/gripper", "handle": "part/handle"},
        }
    )

    with pytest.raises(PlanValidationError, match="duplicate node id"):
        TaskPlan.from_dict(document, _registry())


def test_parallel_is_reserved_but_not_initially_supported():
    document = _plan()
    document["root"] = {"type": "parallel", "id": "root", "children": []}

    with pytest.raises(PlanValidationError, match=r"unsupported node type.*parallel"):
        TaskPlan.from_dict(document, _registry())


def test_plan_cannot_expand_descriptor_retry_limit():
    operation = _plan()["root"]["children"][0]
    operation["constraints"] = {"max_attempts": 11}

    with pytest.raises(PlanValidationError, match=r"max_attempts.*exceeds"):
        TaskPlan.from_dict(_plan(operation), _registry())


def test_transaction_requires_explicit_restart_state():
    operation = _plan()["root"]["children"][0]
    transaction = {
        "type": "transaction",
        "id": "pick-transaction",
        "children": [operation],
    }

    with pytest.raises(PlanValidationError, match="restart_state"):
        TaskPlan.from_dict(_plan(transaction), _registry())


def test_transaction_requires_exactly_one_restartable_operation():
    operation = _plan()["root"]["children"][0]
    transaction = {
        "type": "transaction",
        "id": "pick-transaction",
        "restart_state": ["q_current"],
        "children": [operation, dict(operation, id="pick-again")],
    }

    with pytest.raises(PlanValidationError, match="exactly one executable"):
        TaskPlan.from_dict(_plan(transaction), _registry())


def test_generic_retry_requires_positive_attempt_limit():
    retry = {
        "type": "retry",
        "id": "retry-pick",
        "child": _plan()["root"]["children"][0],
    }

    with pytest.raises(PlanValidationError, match="retry requires positive"):
        TaskPlan.from_dict(_plan(retry), _registry())


def test_returned_document_is_a_defensive_copy():
    plan = TaskPlan.from_dict(_plan(), _registry())
    document = plan.document
    document["mission_id"] = "changed"

    assert plan.document["mission_id"] == "pick-part"
    assert '"mission_id":"pick-part"' in plan.canonical_json
