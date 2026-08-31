"""Allowlisted session factories used by the standalone C++ host."""

from __future__ import annotations

import json

from .capabilities import CapabilityDescriptor, CapabilityRegistry
from .compiler import compile_behavior_tree
from .model import TaskPlan
from .session import TaskPlanningSession


class HostSession(TaskPlanningSession):
    """Task session carrying the deterministic BT artifact consumed by C++."""

    def __init__(self, plan: TaskPlan, registry: CapabilityRegistry) -> None:
        super().__init__(plan, registry)
        self.artifact = compile_behavior_tree(plan)

    def get_behavior_tree_xml(self) -> str:
        return self.artifact.xml


def create_fake_session(options_json: str = "{}") -> HostSession:
    """Create a deterministic non-SpaceLab conformance session.

    Accepts an optional ``fault`` option used exclusively by the C++
    failure-path CTests (``taskplan_bt_fault_*``) to exercise error handling
    in the CPython bridge and BT nodes without touching PyHPP:

    - ``"capability_raises"``: the ``move`` capability raises, exercising
      the exception -> ``retry`` -> exhausted ``RetryUntilSuccessful`` ->
      deterministic BT ``FAILURE`` path (process exit code 1).
    - ``"malformed_json_response"``: ``execute_step`` returns a non-JSON
      string, exercising the C++ ``nlohmann::json::parse`` failure path
      (uncaught in the node, surfaces as process exit code 2).
    - ``"missing_method"``: ``get_report`` is replaced with a
      non-callable, exercising ``PyCallable_Check`` failure in
      ``PythonSession::call`` (process exit code 2).
    """

    options = json.loads(options_json)
    fault = options.get("fault")
    registry = CapabilityRegistry()

    def move_impl(parameters: dict) -> dict:
        if fault == "capability_raises":
            raise RuntimeError("synthetic capability failure for fault-path testing")
        return {"target": parameters["target"], "distance": 1.0}

    registry.register(
        CapabilityDescriptor(
            "move",
            "1.0",
            {"target": str},
            effects=("robot_pose",),
            restartable=True,
        ),
        move_impl,
    )
    document = {
        "schema_version": "1.0",
        "mission_id": "FakeTaskPlan",
        "scene": {"id": "fake-scene", "config_size": 1},
        "provenance": {"kind": "human", "generator": "fake-conformance"},
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
    session = HostSession(TaskPlan.from_dict(document, registry), registry)
    if fault == "malformed_json_response":
        session.execute_step = lambda step_id: "not-json"  # type: ignore[method-assign]
    elif fault == "missing_method":
        session.get_report = None  # type: ignore[method-assign]
    return session


def create_screwdriving_session(options_json: str = "{}") -> HostSession:
    """Create the allowlisted SpaceLab screwdriving planning session.

    The implementation lives outside this generic package, under
    ``script/spacelab/``, and is loaded dynamically so this module carries
    no SpaceLab-specific imports (mirrors the legacy-script loading in
    ``screwdriving_session._load_legacy_module``).
    """

    import importlib.util
    import os
    from pathlib import Path

    root = Path(os.environ.get("AGIMUS_SPACELAB_SOURCE_DIR", Path.cwd()))
    script = root / "script" / "spacelab" / "screwdriving_session.py"
    if not script.exists():
        raise RuntimeError(f"SpaceLab screwdriving runtime not found: {script}")
    spec = importlib.util.spec_from_file_location(
        "agimus_screwdriving_runtime", script
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load SpaceLab screwdriving runtime: {script}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.create_session(options_json)
