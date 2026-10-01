"""Deterministic structured compiler from TaskPlan IR to BehaviorTree.CPP XML."""

from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Any

from .events import element_name
from .model import TaskPlan

# 1.1: every element emitted for an IR node carries ``_ir_id`` and
# ``_ir_role`` (see events.py); BehaviorTree.CPP keeps them as non-port
# attributes. 1.2: ``parallel`` nodes lower to ``Parallel``.
COMPILER_VERSION = "1.2"


@dataclass(frozen=True)
class CompiledBehaviorTree:
    xml: str
    source_map: dict[str, str]
    artifact_fingerprint: str


def compile_behavior_tree(plan: TaskPlan) -> CompiledBehaviorTree:
    """Compile a validated plan using an allowlisted set of BT elements."""

    root = ET.Element(
        "root",
        {"BTCPP_format": "4", "main_tree_to_execute": plan.document["mission_id"]},
    )
    behavior_tree = ET.SubElement(
        root, "BehaviorTree", {"ID": plan.document["mission_id"]}
    )
    mission_sequence = ET.SubElement(
        behavior_tree, "Sequence", {"name": "task-plan-root"}
    )
    ET.SubElement(mission_sequence, "SetupTaskPlan", {"name": "setup-task-plan"})
    source_map: dict[str, str] = {}
    _compile_node(plan, plan.document["root"], mission_sequence, source_map, "mission")
    ET.SubElement(mission_sequence, "FinalizeTaskPlan", {"name": "finalize-task-plan"})

    ET.indent(root, space="  ")
    xml = ET.tostring(root, encoding="unicode", short_empty_elements=True)
    fingerprint_payload = json.dumps(
        {
            "domain": "agimus-bt-artifact-v1",
            "plan_fingerprint": plan.plan_fingerprint,
            "compiler_version": COMPILER_VERSION,
            "xml": xml,
            "source_map": source_map,
        },
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    )
    fingerprint = hashlib.sha256(fingerprint_payload.encode("utf-8")).hexdigest()
    return CompiledBehaviorTree(xml, source_map, fingerprint)


def _compile_node(
    plan: TaskPlan,
    node: dict[str, Any],
    parent: ET.Element,
    source_map: dict[str, str],
    path: str,
) -> None:
    node_id = node["id"]
    node_path = f"{path}/{node_id}"
    source_map[node_id] = node_path
    node_type = node["type"]
    label = node.get("label", node_id)

    def stamp(role: str, **attrib: str) -> dict[str, str]:
        return {**attrib, "_ir_id": node_id, "_ir_role": role}

    if node_type == "transaction":
        fallback = ET.SubElement(
            parent,
            "Fallback",
            stamp("transaction", name=element_name(label, "transaction")),
        )
        ET.SubElement(
            fallback,
            "TaskStepComplete",
            stamp("complete", name=element_name(label, "complete"), step_id=node_id),
        )
        sequence = ET.SubElement(
            fallback, "Sequence", stamp("ready", name=element_name(label, "ready"))
        )
        ET.SubElement(
            sequence,
            "TaskStepReady",
            stamp(
                "precondition",
                name=element_name(label, "precondition"),
                step_id=node_id,
            ),
        )
        retry = ET.SubElement(
            sequence,
            "RetryUntilSuccessful",
            stamp(
                "attempts",
                name=element_name(label, "attempts"),
                num_attempts=str(plan.effective_attempts[node_id]),
            ),
        )
        ET.SubElement(
            retry,
            "ExecuteTaskStep",
            stamp("execute", name=element_name(label, "execute"), step_id=node_id),
        )
        return
    if node_type in {"operation", "condition"}:
        tag = (
            "TaskCapabilityCondition" if node_type == "condition" else "ExecuteTaskStep"
        )
        ET.SubElement(parent, tag, stamp(node_type, name=label, step_id=node_id))
        return
    if node_type == "retry":
        attempts = str(plan.effective_attempts[node_id])
        element = ET.SubElement(
            parent,
            "RetryUntilSuccessful",
            stamp("retry", name=label, num_attempts=attempts),
        )
        _compile_node(plan, node["child"], element, source_map, node_path)
        return

    if node_type == "parallel":
        # Every lane must succeed; the first failure fails the node (and
        # halts the other lanes). Planning stays sequential: a host's
        # ExecuteTaskStep completes within its tick.
        element = ET.SubElement(
            parent,
            "Parallel",
            stamp(
                "parallel",
                name=label,
                success_count=str(len(node["children"])),
                failure_count="1",
            ),
        )
        for child in node["children"]:
            _compile_node(plan, child, element, source_map, node_path)
        return
    tag = "Sequence" if node_type == "sequence" else "Fallback"
    element = ET.SubElement(parent, tag, stamp(node_type, name=label))
    for child in node["children"]:
        _compile_node(plan, child, element, source_map, node_path)
