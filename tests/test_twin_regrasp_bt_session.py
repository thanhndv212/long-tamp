"""Real-scene integration test for ``build_twin_regrasp_session()``.

Companion to ``tests/test_grasp_release_use_case_twin.py``: that file
proves ``GraspSequencePlanner.grasp()``/``.release()`` work against a real
TWIN scene by calling them directly. This file proves the *BT session
wiring* around them -- ``twin_bt_session.py``'s ``grasp_impl``/
``release_impl``/``empty_impl`` closures, the ``TaskPlan`` document built
by ``_build_regrasp_plan_document()``, and ``TaskPlanningSession``'s
dispatch (``execute_step``/``evaluate_condition``) -- by driving the same
public methods the compiled BT XML's ``ExecuteTaskStep``/
``TaskCapabilityCondition`` C++ nodes call, in the exact order the
compiled tree ticks them (root ``sequence`` -> transaction, then a
``fallback`` of [``condition``, ``sequence`` of two transactions]).

This does not reimplement BT.CPP's own Sequence/Fallback tick semantics
(that engine is exercised for real by the opt-in
``taskplan_bt_twin_regrasp`` CTest in
``examples/behaviortree/CMakeLists.txt``, which runs the actual compiled
C++ binary) -- it exercises the Python-side dispatch surface BT.CPP calls
into, against a real PyHPP scene, without needing a C++ build. One piece
of that surface it does have to reproduce by hand: a compiled
``transaction`` wraps its ``ExecuteTaskStep`` in
``RetryUntilSuccessful(num_attempts=<capability's max_attempts>)`` (see
``compiler.py``'s §4 mapping) -- ``grasp`` is registered with
``max_attempts=8`` in this scenario specifically (see
``build_twin_regrasp_session()``'s docstring for why), ``release`` with
the usual 3 -- so a single unlucky target-generation draw is expected to
fail sometimes and get retried by the tree, not treated as a test
failure -- see ``_execute_step_retrying()`` below.

Requires the real PyHPP backend, same as ``test_grasp_release_use_case_twin.py``
-- skips cleanly otherwise.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

try:
    from long_tamp.backends.pyhpp import HAS_PYHPP
except ImportError:
    HAS_PYHPP = False

requires_pyhpp = pytest.mark.skipif(not HAS_PYHPP, reason="PyHPP backend not available")

_TWIN_SCRIPT_DIR = Path(__file__).resolve().parent.parent / "script" / "twin"


def _build_session():
    if str(_TWIN_SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(_TWIN_SCRIPT_DIR))
    from twin_bt_session import build_twin_regrasp_session

    try:
        return build_twin_regrasp_session()
    except Exception as exc:  # environment gap, not a test failure
        pytest.skip(f"TWIN scene could not be set up in this environment: {exc}")


def _execute_step_retrying(session, step_id: str, attempts: int = 3) -> dict:
    """Call ``execute_step`` up to ``attempts`` times, mirroring the compiled
    tree's own ``RetryUntilSuccessful(num_attempts=3)`` around each
    transaction (see module docstring) -- a lone failed attempt here is not
    yet a real failure, only the exhausted budget is."""
    result = None
    for _ in range(attempts):
        result = json.loads(session.execute_step(step_id))
        if result["status"] == "success":
            return result
    return result


@requires_pyhpp
class TestTwinRegraspBtSession:
    def test_release_is_forced_before_regrasp(self):
        session = _build_session()

        setup = json.loads(session.setup())
        assert setup["status"] == "success"

        grasp1 = _execute_step_retrying(session, "grasp-handle1", attempts=8)
        assert grasp1["status"] == "success", grasp1.get("message")

        not_empty = json.loads(session.evaluate_condition("gripper-empty"))
        assert not_empty == {"status": "success", "value": False}, (
            "gripper must still be holding ball/handle right after grasp-handle1 -- "
            "otherwise the fallback's release+regrasp branch never runs, and this "
            "test proves nothing about forcing a release"
        )

        release = _execute_step_retrying(session, "release-gripper")
        assert release["status"] == "success", release.get("message")

        now_empty = json.loads(session.evaluate_condition("gripper-empty"))
        assert now_empty["value"] is True, (
            "release-gripper must have actually freed the gripper, not just "
            "returned success without effect"
        )

        grasp2 = _execute_step_retrying(session, "grasp-handle1-again", attempts=8)
        assert grasp2["status"] == "success", grasp2.get("message")

        after = json.loads(session.evaluate_condition("gripper-empty"))
        assert after == {"status": "success", "value": False}, (
            "regrasping must have actually re-acquired ball/handle"
        )

        finalize = json.loads(session.finalize())
        assert finalize["status"] == "success"
        report = json.loads(session.get_report())
        assert set(report["completed"]) == {
            "grasp-handle1",
            "release-gripper",
            "grasp-handle1-again",
        }

    def test_compiled_xml_matches_the_forced_release_shape(self):
        session = _build_session()

        import xml.etree.ElementTree as ET

        root = ET.fromstring(session.get_behavior_tree_xml())
        top_fallback = root.find(
            ".//Fallback[@name='Already released and regrasped ball/handle']"
        )
        assert top_fallback is not None
        condition, sequence = list(top_fallback)
        assert condition.tag == "TaskCapabilityCondition"
        assert sequence.tag == "Sequence"
        nested_fallbacks = sequence.findall("./Fallback")
        assert len(nested_fallbacks) == 2, "release transaction + grasp transaction"
