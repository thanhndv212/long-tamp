"""Deterministic structured compiler from TaskPlan IR to BehaviorTree.CPP XML."""

from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Any

from .model import TaskPlan

COMPILER_VERSION = "1.0"


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

    if node_type == "transaction":
        fallback = ET.SubElement(parent, "Fallback", {"name": f"{label} transaction"})
        ET.SubElement(
            fallback,
            "TaskStepComplete",
            {"name": f"{label} complete", "step_id": node_id},
        )
        sequence = ET.SubElement(fallback, "Sequence", {"name": f"{label} ready"})
        ET.SubElement(
            sequence,
            "TaskStepReady",
            {"name": f"{label} precondition", "step_id": node_id},
        )
        retry = ET.SubElement(
            sequence,
            "RetryUntilSuccessful",
            {
                "name": f"{label} retry",
                "num_attempts": str(plan.effective_attempts[node_id]),
            },
        )
        ET.SubElement(retry, "ExecuteTaskStep", {"name": label, "step_id": node_id})
        return
    if node_type in {"operation", "condition"}:
        tag = (
            "TaskCapabilityCondition" if node_type == "condition" else "ExecuteTaskStep"
        )
        ET.SubElement(parent, tag, {"name": label, "step_id": node_id})
        return
    if node_type == "retry":
        attempts = str(plan.effective_attempts[node_id])
        element = ET.SubElement(
            parent, "RetryUntilSuccessful", {"name": label, "num_attempts": attempts}
        )
        _compile_node(plan, node["child"], element, source_map, node_path)
        return

    tag = "Sequence" if node_type == "sequence" else "Fallback"
    element = ET.SubElement(parent, tag, {"name": label})
    for child in node["children"]:
        _compile_node(plan, child, element, source_map, node_path)
