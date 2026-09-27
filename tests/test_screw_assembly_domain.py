"""The screw-assembly mission as a TaskPlan (issue #5): pure Python, no HPP.

The plan must have exactly the block structure of ``build_mission`` (so the
planner does the same work as before), and its preconditions/effects must
make the nominal mission feasible and the obvious mistakes infeasible.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

from long_tamp.tasks.task_planning import (
    CapabilityRegistry,
    PlanValidationError,
    TaskPlan,
)

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script/screw_assembly"))
import screw_domain  # noqa: E402


def _registry():
    registry = CapabilityRegistry()
    for descriptor in screw_domain.DESCRIPTORS.values():
        registry.register(descriptor, lambda parameters: {})
    return registry


@pytest.mark.parametrize("n_parts", [1, 2, 4])
def test_one_transaction_per_block_in_mission_order(n_parts):
    document = screw_domain.build_plan_document(n_parts)
    steps = document["root"]["children"]
    blocks = screw_domain.build_mission(n_parts)
    assert [s["children"][0]["parameters"]["block"] for s in steps] == [
        b["label"] for b in blocks
    ]
    assert len(steps) == 3 + 4 * n_parts


@pytest.mark.parametrize("n_parts", [1, 4])
def test_nominal_mission_is_feasible(n_parts):
    TaskPlan.from_dict(screw_domain.build_plan_document(n_parts), _registry())


def test_capabilities_declare_the_right_effects():
    steps = screw_domain.build_plan_document(1)["root"]["children"]
    capabilities = [s["children"][0]["capability"] for s in steps]
    assert capabilities == [
        "grasp",  # bootstrap: pick driver
        "home",
        "grasp",  # part1 A0
        "clamp_and_screw",  # part1 A
        "home",
        "release",  # part1 B
        "rack",  # return
    ]
    clamp = steps[3]["children"][0]["parameters"]
    assert clamp["part"] == "part1"
    assert clamp["seat"] == "part1/h_seat"
    assert (clamp["hole1"], clamp["hole2"]) == ("part1/h_hole1", "part1/h_hole2")


def test_screwing_without_the_driver_is_rejected():
    document = screw_domain.build_plan_document(1)
    steps = document["root"]["children"]
    del steps[0]  # no driver pickup
    with pytest.raises(PlanValidationError, match="holds.ur10_right/gripper"):
        TaskPlan.from_dict(document, _registry())


def test_a_mission_started_with_the_driver_in_hand_is_feasible():
    """Same plan, scenario start: the pickup's effect already holds, so it is
    skipped (as the effect guard does at run time) instead of rejected."""
    document = screw_domain.build_plan_document(
        1, initial_state=["holds(ur10_right/gripper, driver/h_grip)"]
    )
    TaskPlan.from_dict(document, _registry())


def test_a_mission_started_with_a_foreign_tool_in_hand_is_rejected():
    document = screw_domain.build_plan_document(
        1, initial_state=["holds(ur10_right/gripper, part2/h_grasp)"]
    )
    with pytest.raises(PlanValidationError, match="b00-grasp"):
        TaskPlan.from_dict(document, _registry())


def test_home_moves_have_no_effects_so_they_always_run():
    assert screw_domain.DESCRIPTORS["home"].effects == ()
    assert screw_domain.RECORDED_PREDICATES == {"screwed"}
