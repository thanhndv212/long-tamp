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
    """Create a deterministic, mission-agnostic conformance session.

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


def create_twin_session(options_json: str = "{}") -> HostSession:
    """Real-mission factory: TWIN's bimanual lift-ball scene (§9).

    The adapter itself -- capability registry, ``TaskPlan`` document,
    session construction against a real PyHPP scene -- lives outside this
    generic layer, in ``script/twin/twin_bt_session.py``, per this file's
    own module docstring ("the generic layer knows nothing about any
    specific robot, mission, or gripper"). This factory only imports it,
    lazily: ``script/`` is a dev-checkout path (example/demo code), not
    part of the installed package, so importing it eagerly at module load
    time would break importing ``host.py`` itself in any environment
    that's ``pip install``-only.

    Requires running from a repo checkout with ``script/twin/`` present
    (true of every environment this C++ host is built and run from today
    -- see ``docs/usage/behaviortree-integration.md``); raises ImportError
    otherwise, same as any other missing optional dependency.
    """
    import sys
    from pathlib import Path

    twin_dir = Path(__file__).resolve().parents[4] / "script" / "twin"
    if str(twin_dir) not in sys.path:
        sys.path.insert(0, str(twin_dir))
    from twin_bt_session import build_twin_session

    return build_twin_session(options_json)
