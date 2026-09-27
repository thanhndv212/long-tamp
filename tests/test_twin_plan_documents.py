"""TWIN's real mission documents pass the plan-time precondition check.

Pure Python: builds the same descriptors and documents
``script/twin/twin_bt_session.py`` hands to ``HostSession``, without HPP.
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import pytest

from long_tamp.tasks.task_planning import CapabilityRegistry, PlanValidationError
from long_tamp.tasks.task_planning.model import TaskPlan

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script" / "twin"))
import twin_bt_session  # noqa: E402

GRASP_SEQUENCE = [
    ("panda_left/gripper", "ball/handle"),
    ("panda_right/gripper", "ball/handle2"),
]


def _registry(grasp_attempts=3):
    descriptors = twin_bt_session._descriptors()
    registry = CapabilityRegistry()
    registry.register(
        replace(descriptors["grasp"], max_attempts=grasp_attempts), lambda p: {}
    )
    registry.register(descriptors["release"], lambda p: {})
    registry.register(descriptors["empty"], lambda p: True)
    return registry


def test_lift_ball_plan_is_feasible():
    document = twin_bt_session._build_plan_document(GRASP_SEQUENCE)
    TaskPlan.from_dict(document, _registry())


def test_regrasp_plan_is_feasible():
    document = twin_bt_session._build_regrasp_plan_document()
    TaskPlan.from_dict(document, _registry(grasp_attempts=8))


def test_two_grippers_on_one_handle_are_rejected():
    document = twin_bt_session._build_plan_document(
        [("panda_left/gripper", "ball/handle"), ("panda_right/gripper", "ball/handle")]
    )
    with pytest.raises(PlanValidationError, match=r"not holds\(_, ball/handle\)"):
        TaskPlan.from_dict(document, _registry())
