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


def _registry(n_parts=1):
    registry = CapabilityRegistry()
    for descriptor in screw_domain.descriptors(n_parts).values():
        registry.register(descriptor, lambda parameters: {})
    return registry


def _check(n_parts, initial_state=()):
    document = screw_domain.build_plan_document(n_parts, list(initial_state))
    return TaskPlan.from_dict(document, _registry(n_parts))


_PART1_DONE = [
    "holds(fixtures/clamp1, part1/h_seat)",
    "screwed(part1, part1/h_hole1)",
    "screwed(part1, part1/h_hole2)",
]
_DRIVER = "holds(ur10_right/gripper, driver/h_grip)"


@pytest.mark.parametrize("n_parts", [1, 2, 4])
def test_one_transaction_per_block_in_mission_order(n_parts):
    document = screw_domain.build_plan_document(n_parts)
    steps = screw_domain.transactions(document)
    blocks = screw_domain.build_mission(n_parts)
    assert [s["children"][0]["parameters"]["block"] for s in steps] == [
        b["label"] for b in blocks
    ]
    assert len(steps) == 3 + 4 * n_parts


@pytest.mark.parametrize("n_parts", [1, 4])
def test_nominal_mission_is_feasible(n_parts):
    _check(n_parts)


def test_capabilities_declare_the_right_effects():
    steps = screw_domain.transactions(screw_domain.build_plan_document(1))
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
    pick = document["root"]["children"][0]["children"][1]["children"]
    del pick[0]  # no driver pickup
    with pytest.raises(PlanValidationError, match="holds.ur10_right/gripper"):
        TaskPlan.from_dict(document, _registry())


def test_a_mission_started_with_the_driver_in_hand_is_feasible():
    """The pickup's effect already holds: skipped, not rejected."""
    _check(1, [_DRIVER])


def test_a_mission_started_with_a_foreign_tool_in_hand_is_rejected():
    with pytest.raises(PlanValidationError, match="b00-grasp"):
        _check(1, ["holds(ur10_right/gripper, part2/h_grasp)"])


def test_resuming_after_a_part_was_released_skips_that_part():
    """Killed after part 1's release: A0's own effect no longer holds (the part
    was let go). Without the part_done guard A0 would grasp the clamped part
    again and B release it again: pointless planning, but not infeasible."""
    _check(2, [_DRIVER, *_PART1_DONE])


def test_a_finished_mission_does_not_pick_the_driver_again():
    finished = [*_PART1_DONE, "holds(fixtures/rack_hold, driver/h_rack)"]
    _check(1, finished)


def test_a_mission_started_with_the_part_in_hand_is_feasible():
    _check(1, ["holds(ur10_left/gripper, part1/h_grasp)"])


def test_home_moves_have_no_effects_so_they_always_run():
    assert screw_domain.DESCRIPTORS["home"].effects == ()
    assert screw_domain.RECORDED_PREDICATES == {"screwed"}
