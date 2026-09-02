import xml.etree.ElementTree as ET

from long_tamp.tasks.task_planning import (
    CapabilityDescriptor,
    CapabilityRegistry,
    TaskPlan,
    compile_behavior_tree,
)


def _compile(label="Move <home> & wait"):
    registry = CapabilityRegistry()
    registry.register(
        CapabilityDescriptor("move", "1.0", {"target": str}, restartable=True),
        lambda parameters: parameters,
    )
    document = {
        "schema_version": "1.0",
        "mission_id": "move-demo",
        "scene": {"id": "fake", "config_size": 1},
        "provenance": {"kind": "human", "generator": "test"},
        "root": {
            "type": "sequence",
            "id": "root",
            "children": [
                {
                    "type": "operation",
                    "id": "move-home",
                    "label": label,
                    "capability": "move",
                    "parameters": {"target": "home"},
                }
            ],
        },
    }
    return compile_behavior_tree(TaskPlan.from_dict(document, registry))


def test_compiler_is_deterministic_and_emits_source_map():
    first = _compile()
    second = _compile()

    assert first.xml == second.xml
    assert first.source_map == second.source_map
    assert first.artifact_fingerprint == second.artifact_fingerprint
    assert first.source_map["move-home"].endswith("/move-home")


def test_compiler_escapes_labels_and_emits_parseable_btcpp4_xml():
    artifact = _compile()
    root = ET.fromstring(artifact.xml)

    assert root.attrib == {
        "BTCPP_format": "4",
        "main_tree_to_execute": "move-demo",
    }
    execute = root.find(".//ExecuteTaskStep")
    assert execute is not None
    assert execute.attrib["name"] == "Move <home> & wait"


def test_transaction_compiles_complete_ready_retry_execute_subtree():
    registry = CapabilityRegistry()
    registry.register(
        CapabilityDescriptor("move", "1.0", {"target": str}, restartable=True),
        lambda parameters: parameters,
    )
    document = {
        "schema_version": "1.0",
        "mission_id": "transaction-demo",
        "scene": {"id": "fake", "config_size": 1},
        "provenance": {"kind": "human", "generator": "test"},
        "root": {
            "type": "transaction",
            "id": "move-home",
            "label": "Move home",
            "restart_state": ["q_current"],
            "children": [
                {
                    "type": "operation",
                    "id": "move-home.execute",
                    "capability": "move",
                    "parameters": {"target": "home"},
                }
            ],
        },
    }

    root = ET.fromstring(
        compile_behavior_tree(TaskPlan.from_dict(document, registry)).xml
    )

    fallback = root.find(".//Fallback")
    assert [child.tag for child in fallback] == ["TaskStepComplete", "Sequence"]
    assert fallback.find("./Sequence/TaskStepReady") is not None
    retry = fallback.find("./Sequence/RetryUntilSuccessful")
    assert retry is not None
    assert retry.attrib["num_attempts"] == "1"
    assert retry.find("./ExecuteTaskStep") is not None
