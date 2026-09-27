"""TaskPlan diagrams (issue #7): Mermaid and Graphviz renderings, pure Python."""

import re

from long_tamp.tasks.task_planning import (
    CapabilityDescriptor,
    CapabilityRegistry,
    TaskPlan,
)
from long_tamp.tasks.task_planning.visualize import to_dot, to_mermaid


def _transaction(node_id, label, capability, **parameters):
    return {
        "type": "transaction",
        "id": node_id,
        "label": label,
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


def _registry():
    registry = CapabilityRegistry()
    registry.register(
        CapabilityDescriptor(
            "grasp",
            "1.0",
            {"gripper": str, "handle": str},
            effects=("holds(?gripper, ?handle)",),
            max_attempts=3,
            restartable=True,
        ),
        lambda p: {},
    )
    registry.register(
        CapabilityDescriptor(
            "release",
            "1.0",
            {"gripper": str},
            effects=("not holds(?gripper, _)",),
            restartable=True,
        ),
        lambda p: {},
    )
    registry.register(
        CapabilityDescriptor(
            "empty",
            "1.0",
            {"gripper": str},
            preconditions=("not holds(?gripper, _)",),
        ),
        lambda p: True,
    )
    return registry


def _plan():
    """The TWIN regrasp shape: a transaction, then a condition-guarded fallback."""
    document = {
        "schema_version": "1.0",
        "mission_id": "Regrasp",
        "scene": {"id": "fake"},
        "provenance": {"kind": "human", "generator": "test"},
        "root": {
            "type": "sequence",
            "id": "root",
            "label": "Regrasp",
            "children": [
                _transaction(
                    "grasp-1", "Grasp ball", "grasp", gripper="left", handle="ball"
                ),
                {
                    "type": "fallback",
                    "id": "cycled",
                    "label": "Already cycled",
                    "children": [
                        {
                            "type": "condition",
                            "id": "gripper-empty",
                            "label": "Gripper empty",
                            "capability": "empty",
                            "parameters": {"gripper": "left"},
                        },
                        {
                            "type": "sequence",
                            "id": "release-then-regrasp",
                            "label": "Release, regrasp",
                            "children": [
                                _transaction(
                                    "release-1", "Release", "release", gripper="left"
                                ),
                                _transaction(
                                    "grasp-2",
                                    "Regrasp",
                                    "grasp",
                                    gripper="left",
                                    handle="ball",
                                ),
                            ],
                        },
                    ],
                },
            ],
        },
    }
    return TaskPlan.from_dict(document, _registry())


def test_mermaid_renders_every_plan_node_with_its_kind():
    text = to_mermaid(_plan())
    assert text.startswith("flowchart TD")
    # Mermaid ids are sanitized; labels keep the originals.
    assert 'grasp_1["Grasp ball<br/>grasp(left, ball) ×3"]' in text
    assert 'cycled{{"? Already cycled"}}' in text
    assert 'gripper_empty{"Gripper empty?"}' in text
    assert 'root["→ Regrasp"]' in text


def test_fallback_alternatives_are_dashed_else_edges():
    text = to_mermaid(_plan())
    assert "cycled --> gripper_empty" in text
    assert "cycled -.->|else| release_then_regrasp" in text
    assert "root --> grasp_1" in text


def test_effects_can_be_shown():
    text = to_mermaid(_plan(), registry=_registry())
    assert "⇒ holds(left, ball)" in text
    assert "⇒ not holds(left, _)" in text


def test_mermaid_ids_are_unique_and_valid():
    text = to_mermaid(_plan())
    ids = re.findall(r"^\s+([A-Za-z0-9_]+)[\[\{(]", text, re.M)
    assert len(ids) == len(set(ids)) == 7


def test_rendering_is_deterministic():
    assert to_mermaid(_plan()) == to_mermaid(_plan())
    assert to_dot(_plan()) == to_dot(_plan())


def test_dot_uses_shapes_and_dashed_alternatives():
    text = to_dot(_plan())
    assert text.startswith('digraph "Regrasp" {')
    assert '"gripper-empty" [label="Gripper empty?", shape=diamond];' in text
    assert '"cycled" [label="? Already cycled", shape=hexagon];' in text
    assert '"cycled" -> "release-then-regrasp" [style=dashed, label="else"];' in text
    assert text.rstrip().endswith("}")
