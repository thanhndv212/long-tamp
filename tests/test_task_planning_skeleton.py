"""From a goal to a skeleton to a validated TaskPlan (issue #14).

``skeleton_document`` and the screw assembly's expansion are checked without a
planner; the planner round trip (goal -> Fast Downward -> TaskPlan that passes
``TaskPlan.from_dict``, the same gate as a hand-written plan) runs when the
``planning`` extra is installed.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

from long_tamp.tasks.task_planning import (
    CapabilityDescriptor,
    CapabilityRegistry,
    PlanValidationError,
    TaskPlan,
)
from long_tamp.tasks.task_planning.skeleton import NoPlanFound, skeleton_document

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script/screw_assembly"))
import screw_domain  # noqa: E402

MOVE = CapabilityDescriptor(
    "move",
    "1.0",
    {"target": str},
    preconditions=("not at(?target)",),
    effects=("at(?target)",),
    restartable=True,
)


def _registry(descriptors):
    registry = CapabilityRegistry()
    for descriptor in descriptors:
        registry.register(descriptor, lambda parameters: {})
    return registry


def test_a_skeleton_becomes_a_sequence_of_transactions():
    document = skeleton_document(
        [("move", {"target": "a"}), ("move", {"target": "b"})], mission_id="Moves"
    )
    plan = TaskPlan.from_dict(document, _registry([MOVE]))
    root = plan.document["root"]
    assert [child["id"] for child in root["children"]] == ["b00-move", "b01-move"]
    assert root["children"][0]["label"] == "move(a)"
    assert document["provenance"]["kind"] == "planner"


def test_a_generated_plan_goes_through_the_same_validation():
    """Leaving ``a`` before ever reaching it violates ``at(a)``: rejected at
    load time, as a hand-written plan would be."""
    leave = CapabilityDescriptor(
        "leave",
        "1.0",
        {"target": str},
        preconditions=("at(?target)",),
        effects=("left(?target)",),
        restartable=True,
    )
    document = skeleton_document([("leave", {"target": "a"})], mission_id="Bad")
    with pytest.raises(PlanValidationError):
        TaskPlan.from_dict(document, _registry([MOVE, leave]))


# ---------------------------------------------------------------- screw domain


@pytest.mark.parametrize("n_parts", [1, 4])
def test_block_for_rebuilds_every_hand_written_block(n_parts):
    for index, block in enumerate(screw_domain.build_mission(n_parts)):
        assert screw_domain.block_for(*screw_domain._step(index, block)) == block


def test_a_clamp_other_than_the_parts_own_gets_its_block_and_label():
    steps = [
        ("grasp", {"gripper": "ur10_right/gripper", "handle": "driver/h_grip"}),
        ("grasp", {"gripper": "ur10_left/gripper", "handle": "part1/h_grasp"}),
        (
            "clamp_and_screw",
            {
                "holder": "ur10_left/gripper",
                "held": "part1/h_grasp",
                "tool_gripper": "ur10_right/gripper",
                "tool": "driver/h_grip",
                "clamp": "fixtures/clamp2",
                "seat": "part1/h_seat",
                "part": "part1",
                "hole1": "part1/h_hole1",
                "hole2": "part1/h_hole2",
            },
        ),
    ]
    document = screw_domain.planned_document(steps, 1)
    labels = [t["label"] for t in screw_domain.transactions(document)]
    assert labels == [
        "bootstrap: pick driver",
        "ur10_right home (bootstrap)",
        "part1 A0: grasp",
        "part1 A: clamp + screw (in clamp2)",
        "ur10_right home (part1)",
    ]
    clamp = screw_domain.transactions(document)[3]["children"][0]
    block = screw_domain.block_for(clamp["capability"], clamp["parameters"])
    assert block["seq"][0] == ("fixtures/clamp2", "part1/h_seat")


def _hand_skeleton(n_parts):
    """The hand-written mission's order, as a planner would return it."""
    steps = [("grasp", {"gripper": "ur10_right/gripper", "handle": "driver/h_grip"})]
    for i in range(1, n_parts + 1):
        p = f"part{i}"
        steps += [
            ("grasp", {"gripper": "ur10_left/gripper", "handle": f"{p}/h_grasp"}),
            (
                "clamp_and_screw",
                {
                    "holder": "ur10_left/gripper",
                    "held": f"{p}/h_grasp",
                    "tool_gripper": "ur10_right/gripper",
                    "tool": "driver/h_grip",
                    "clamp": f"fixtures/clamp{i}",
                    "seat": f"{p}/h_seat",
                    "part": p,
                    "hole1": f"{p}/h_hole1",
                    "hole2": f"{p}/h_hole2",
                },
            ),
            ("release", {"gripper": "ur10_left/gripper"}),
        ]
    return steps + [
        (
            "rack",
            {
                "gripper": "ur10_right/gripper",
                "dock": "fixtures/rack_hold",
                "dock_handle": "driver/h_rack",
            },
        )
    ]


@pytest.mark.parametrize("n_parts", [1, 4])
def test_the_hand_order_expands_to_the_hand_written_blocks(n_parts):
    document = screw_domain.planned_document(_hand_skeleton(n_parts), n_parts)
    TaskPlan.from_dict(document, _registry(screw_domain.descriptors(n_parts).values()))
    planned = [
        screw_domain.block_for(t["children"][0]["capability"], t["children"][0]["parameters"])
        for t in screw_domain.transactions(document)
    ]
    assert planned == screw_domain.build_mission(n_parts)


# ------------------------------------------------------------------- planner


@pytest.fixture(params=["fast-downward", "pyperplan"])
def engine(request):
    pytest.importorskip("unified_planning")
    module = {"fast-downward": "up_fast_downward", "pyperplan": "up_pyperplan"}
    pytest.importorskip(module[request.param])
    return request.param


def _planner(engine):
    from long_tamp.tasks.task_planning.skeleton import UnifiedPlanningPlanner

    return UnifiedPlanningPlanner(engine)


@pytest.mark.parametrize("n_parts", [1, 2, 4])
def test_an_n_part_goal_is_planned_into_a_valid_task_plan(n_parts, engine):
    steps = _planner(engine).solve(screw_domain.pddl_problem(n_parts))
    document = screw_domain.planned_document(steps, n_parts)
    TaskPlan.from_dict(document, _registry(screw_domain.descriptors(n_parts).values()))
    labels = [t["label"] for t in screw_domain.transactions(document)]
    # Same blocks as the hand-written mission, in an order the planner chose.
    assert sorted(labels) == sorted(b["label"] for b in screw_domain.build_mission(n_parts))


def test_a_partly_done_mission_is_planned_from_its_world_state(engine):
    state = [
        "holds(ur10_right/gripper, driver/h_grip)",
        "holds(fixtures/clamp1, part1/h_seat)",
        "screwed(part1, part1/h_hole1)",
        "screwed(part1, part1/h_hole2)",
    ]
    steps = _planner(engine).solve(screw_domain.pddl_problem(2, state))
    document = screw_domain.planned_document(steps, 2, state)
    TaskPlan.from_dict(document, _registry(screw_domain.descriptors(2).values()))
    labels = [t["label"] for t in screw_domain.transactions(document)]
    # Only part 2's work and the rack, in an order the engine chose (e.g.
    # pyperplan racks the driver before ur10_left lets go of part 2).
    assert sorted(labels) == sorted(
        [
            "part2 A0: grasp",
            "part2 A: clamp + screw",
            "ur10_right home (part2)",
            "part2 B: release",
            "return: rack driver",
        ]
    )
    assert labels.index("part2 A0: grasp") < labels.index("part2 A: clamp + screw")


def test_an_unreachable_goal_raises(engine):
    export = screw_domain.pddl_problem(1)
    export.problem = export.problem.replace(
        "(can_grasp ur10_left__gripper part1__h_grasp)", ""
    )
    with pytest.raises(NoPlanFound):
        _planner(engine).solve(export)
