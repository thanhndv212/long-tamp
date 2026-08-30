import json

from agimus_spacelab.tasks.task_planning import (
    CapabilityDescriptor,
    CapabilityRegistry,
    TaskPlan,
    TaskPlanningSession,
)


def _session(implementation=None, node_type="operation", transaction=False):
    calls = []
    registry = CapabilityRegistry()
    registry.register(
        CapabilityDescriptor(
            capability_id="move",
            version="1.0",
            required_parameters={"target": str},
            effects=("robot_pose",),
            restartable=True,
        ),
        implementation
        or (lambda parameters: calls.append(parameters["target"]) or {"distance": 1.0}),
    )
    operation = {
        "type": node_type,
        "id": "move-home",
        "capability": "move",
        "parameters": {"target": "home"},
    }
    child = operation
    if transaction:
        operation["id"] = "move-home-transaction.execute"
        child = {
            "type": "transaction",
            "id": "move-home-transaction",
            "restart_state": ["q_current"],
            "children": [operation],
        }
    document = {
        "schema_version": "1.0",
        "mission_id": "move-demo",
        "scene": {"id": "fake", "config_size": 1},
        "provenance": {"kind": "human", "generator": "test"},
        "root": {
            "type": "sequence",
            "id": "root",
            "children": [child],
        },
    }
    return TaskPlanningSession(TaskPlan.from_dict(document, registry), registry), calls


def test_session_executes_registered_capability_and_checkpoints_success():
    session, calls = _session()

    result = json.loads(session.execute_step("move-home"))

    assert result["status"] == "success"
    assert result["metrics"] == {"distance": 1.0}
    assert json.loads(session.is_step_complete("move-home"))["complete"] is True
    assert calls == ["home"]


def test_completed_step_is_not_executed_twice():
    session, calls = _session()
    session.execute_step("move-home")

    result = json.loads(session.execute_step("move-home"))

    assert result["status"] == "skipped"
    assert calls == ["home"]


def test_transaction_dispatches_child_and_checkpoints_transaction_id():
    session, calls = _session(transaction=True)

    result = json.loads(session.execute_step("move-home-transaction"))

    assert result["status"] == "success"
    assert json.loads(session.is_step_complete("move-home-transaction"))["complete"]
    assert calls == ["home"]


def test_registered_condition_result_is_evaluated():
    session, _ = _session(
        implementation=lambda parameters: False, node_type="condition"
    )

    result = json.loads(session.evaluate_condition("move-home"))

    assert result == {"status": "success", "value": False}
