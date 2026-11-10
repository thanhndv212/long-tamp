"""Allowlisted session factories used by the standalone C++ host."""

from __future__ import annotations

import json

from .capabilities import CapabilityDescriptor, CapabilityRegistry
from .compiler import compile_behavior_tree
from .model import TaskPlan
from .session import TaskPlanningSession, WorldState
from .world_state import RecordedFacts


class HostSession(TaskPlanningSession):
    """Task session carrying the deterministic BT artifact consumed by C++."""

    def __init__(
        self,
        plan: TaskPlan,
        registry: CapabilityRegistry,
        world_state: WorldState | None = None,
        recorded: RecordedFacts | None = None,
    ) -> None:
        super().__init__(plan, registry, world_state, recorded)
        self.artifact = compile_behavior_tree(plan)

    def get_behavior_tree_xml(self) -> str:
        return self.artifact.xml

    def get_plan_document(self) -> str:
        """The plan's IR document (JSON), e.g. for a host to start its event
        stream with a ``plan`` event, which viewers draw (#25)."""
        return json.dumps(self.plan.document)


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

    ``"shape": "composite"`` builds a plan using every composite instead of
    a single transaction -- a ``sequence`` of a ``fallback`` guarded by a
    (false) ``condition`` and a ``retry`` around a transaction whose first
    attempt fails -- so the C++ host and the Python runner can be compared
    on every node type (``examples/behaviortree/check_events.py``).
    """

    options = json.loads(options_json)
    fault = options.get("fault")
    registry = CapabilityRegistry()
    calls: dict[str, int] = {}

    def move_impl(parameters: dict) -> dict:
        if fault == "capability_raises":
            raise RuntimeError("synthetic capability failure for fault-path testing")
        return {"target": parameters["target"], "distance": 1.0}

    def flaky_move_impl(parameters: dict) -> dict:
        calls["flaky"] = calls.get("flaky", 0) + 1
        if calls["flaky"] == 1:
            raise RuntimeError("synthetic first-attempt failure")
        return move_impl(parameters)

    registry.register(
        CapabilityDescriptor(
            "move",
            "1.0",
            {"target": str},
            writes=("robot_pose",),
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
    if options.get("shape") == "composite":
        registry.register(
            CapabilityDescriptor("at", "1.0", {"target": str}),
            lambda parameters: False,
        )
        registry.register(
            CapabilityDescriptor(
                "flaky_move",
                "1.0",
                {"target": str},
                writes=("robot_pose",),
                restartable=True,
                max_attempts=2,
            ),
            flaky_move_impl,
        )
        document["root"] = _composite_root()
    session = HostSession(TaskPlan.from_dict(document, registry), registry)
    if fault == "malformed_json_response":
        session.execute_step = lambda step_id: "not-json"  # type: ignore[method-assign]
    elif fault == "missing_method":
        session.get_report = None  # type: ignore[method-assign]
    return session


def _move(
    node_id: str, target: str, attempts: int = 1, capability: str = "move"
) -> dict:
    return {
        "type": "transaction",
        "id": node_id,
        "label": node_id.replace("-", " ").capitalize(),
        "restart_state": ["q_current"],
        "children": [
            {
                "type": "operation",
                "id": f"{node_id}.execute",
                "capability": capability,
                "parameters": {"target": target},
                "constraints": {"max_attempts": attempts},
            }
        ],
    }


def _composite_root() -> dict:
    return {
        "type": "sequence",
        "id": "mission",
        "children": [
            {
                "type": "fallback",
                "id": "reach-a",
                "children": [
                    {
                        "type": "condition",
                        "id": "at-a",
                        "capability": "at",
                        "parameters": {"target": "a"},
                    },
                    _move("move-a", "a"),
                ],
            },
            {
                "type": "retry",
                "id": "retry-flaky",
                "max_attempts": 2,
                "child": _move("move-flaky", "b", attempts=2, capability="flaky_move"),
            },
        ],
    }


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


def create_twin_regrasp_session(options_json: str = "{}") -> HostSession:
    """Real-mission factory: TWIN scene, forced release before regrasp (§11 item 1).

    Same lazy-import rationale as ``create_twin_session`` above. See
    ``script/twin/twin_bt_session.py``'s ``build_twin_regrasp_session()``
    docstring for what this plan document exercises that the flat
    ``create_twin_session`` one does not: a real ``release()`` forced by a
    ``fallback``/``condition`` guard before a second real ``grasp()``.
    """
    import sys
    from pathlib import Path

    twin_dir = Path(__file__).resolve().parents[4] / "script" / "twin"
    if str(twin_dir) not in sys.path:
        sys.path.insert(0, str(twin_dir))
    from twin_bt_session import build_twin_regrasp_session

    return build_twin_regrasp_session(options_json)


def create_screw_session(options_json: str = "{}") -> HostSession:
    """Real-mission factory: the screw-assembly cell (#58).

    A short seeded plan (pick the driver; home it while the left arm grasps
    part 1, in a ``parallel`` node; rack it) built by
    ``script/screw_assembly/screw_bt_session.py``, imported lazily for the
    same reason as ``create_twin_session``. Options: ``seed``.
    """
    import sys
    from pathlib import Path

    screw_dir = Path(__file__).resolve().parents[4] / "script" / "screw_assembly"
    if str(screw_dir) not in sys.path:
        sys.path.insert(0, str(screw_dir))
    from screw_bt_session import build_screw_session

    return build_screw_session(options_json)
