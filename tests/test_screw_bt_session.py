"""The BehaviorTree.CPP session path on the screw-assembly cell (#58).

``create_screw_session`` is the factory the C++ host (``agimus_taskplan_bt``)
calls. This drives the same session with ``run_plan``, the reference
semantics of the compiled tree, on the real scene: pick the driver, home it
while the left arm grasps part 1 (a ``parallel`` node), rack it. Seeded;
it replaces the TWIN session check, whose random handover made it pass or
fail by chance (#54, #57). The opt-in ``taskplan_bt_screw_cell`` CTest runs
the compiled tree itself.

Needs the PyHPP backend; skips otherwise.
"""

import json
import xml.etree.ElementTree as ET

import pytest

try:
    from long_tamp.backends.pyhpp import HAS_PYHPP
except ImportError:
    HAS_PYHPP = False


@pytest.mark.skipif(not HAS_PYHPP, reason="PyHPP backend not available")
@pytest.mark.slow_planning
def test_the_screw_cell_session_runs_its_plan():
    from long_tamp.tasks.task_planning.host import create_screw_session
    from long_tamp.tasks.task_planning.runner import run_plan

    session = create_screw_session(json.dumps({"seed": 1}))

    # the compiled tree: the home move and the part grasp in a Parallel
    tree = ET.fromstring(session.get_behavior_tree_xml())
    (parallel,) = tree.iter("Parallel")
    assert parallel.attrib["success_count"] == "2"

    run = run_plan(session)
    assert run.success, f"{run.failed_step}: {run.message}"
    labels = [r["label"] for r in session.ctx["records"]]
    assert labels == [
        "bootstrap: pick driver",
        "ur10_right home (bootstrap)",  # the lanes, planned in plan order
        "part1 A0: grasp",
        "return: rack driver",
    ]
    state = {str(atom) for atom in session.world_state()}
    assert "holds(ur10_left/gripper, part1/h_grasp)" in state
    assert "holds(fixtures/rack_hold, driver/h_rack)" in state
