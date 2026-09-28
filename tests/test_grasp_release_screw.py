"""``grasp()`` / ``release()`` on a real scene we own: the screw-assembly cell.

The blocking counterpart of ``test_grasp_release_use_case_twin.py``, whose
borrowed scene fails now and then (#54). Seeded, and built so that every step
is feasible by construction: ur10_right grasps the driver on its dock,
ur10_left grasps part 1 on the staging row, a conflicting grasp is refused
without planning, then both release where they grasped (nothing has moved,
so each release goes back the way its grasp came).

Needs the PyHPP backend; skips otherwise. Uses the scene on disk (any part
count >= 1; ``build_scene.py`` regenerates it).
"""

import sys
from pathlib import Path

import pytest

try:
    from long_tamp.backends.pyhpp import HAS_PYHPP
except ImportError:
    HAS_PYHPP = False

SCREW_DIR = Path(__file__).resolve().parents[1] / "script" / "screw_assembly"
LEFT, RIGHT = "ur10_left/gripper", "ur10_right/gripper"
DRIVER, PART = "driver/h_grip", "part1/h_grasp"


@pytest.mark.skipif(not HAS_PYHPP, reason="PyHPP backend not available")
@pytest.mark.slow_planning
def test_grasp_release_lifecycle_on_the_screw_cell(tmp_path):
    if str(SCREW_DIR) not in sys.path:
        sys.path.insert(0, str(SCREW_DIR))
    import task_screw_assembly as T  # noqa: PLC0415 - path-dependent import

    T.seed_everything(1)
    task, planner = T.setup(log_dir=str(tmp_path))
    tracker = planner.grasp_tracker

    r1 = planner.grasp(RIGHT, DRIVER, task.q_init)
    assert r1["success"], f"driver grasp failed: {r1['message']}"
    r2 = planner.grasp(LEFT, PART, r1["final_config"])
    assert r2["success"], f"part grasp failed: {r2['message']}"
    assert tracker.current_grasps[RIGHT] == DRIVER
    assert tracker.current_grasps[LEFT] == PART

    # A conflicting grasp is refused before any planning, not auto-released.
    r3 = planner.grasp(LEFT, DRIVER, r2["final_config"])
    assert r3["success"] is False and "release(" in r3["message"]
    assert tracker.current_grasps[LEFT] == PART

    r4 = planner.release(LEFT, r2["final_config"])
    assert r4["success"], f"part release failed: {r4['message']}"
    assert tracker.current_grasps[LEFT] is None
    r5 = planner.release(RIGHT, r4["final_config"])
    assert r5["success"], f"driver release failed: {r5['message']}"
    assert tracker.current_grasps[RIGHT] is None

    # Releasing a free gripper is a no-op.
    assert planner.release(RIGHT, r5["final_config"])["skipped"]
