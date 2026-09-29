"""The repair loop (issue #15): a refinement failure becomes a blocked binding
and the goal is replanned from the current world state.

The loop and the ``blocked`` export are checked with a toy domain (a box that
can go into either of two slots); the end-to-end version plans with Unified
Planning when the ``planning`` extra is installed.
"""

from __future__ import annotations

import pytest

from long_tamp.tasks.task_planning import CapabilityDescriptor
from long_tamp.tasks.task_planning.pddl import to_pddl
from long_tamp.tasks.task_planning.predicates import parse_atom
from long_tamp.tasks.task_planning.repair import (
    block_failed_step,
    plan_execute_repair,
)
from long_tamp.tasks.task_planning.skeleton import skeleton_document

PLACE = CapabilityDescriptor(
    "place",
    "1.0",
    {"obj": str, "slot": str},
    preconditions=("not placed(?obj)", "not full(?slot)"),
    effects=("placed(?obj)", "full(?slot)"),
    restartable=True,
)
STATIC = {"place": ("fits(?obj, ?slot)",)}
INIT = ["fits(box, slot1)", "fits(box, slot2)"]


def _export(blocked=()):
    return to_pddl([PLACE], INIT, ["placed(box)"], static_preconditions=STATIC,
                   blocked=blocked)  # fmt: skip


def test_blocked_bindings_become_a_precondition_and_facts():
    x = _export(blocked=[("place", {"slot": "slot1"})])
    assert "(not (blocked_place__slot ?slot))" in x.domain
    assert "(blocked_place__slot slot1)" in x.problem
    with pytest.raises(ValueError, match="not planning parameters"):
        _export(blocked=[("place", {"colour": "red"})])
    with pytest.raises(ValueError, match="unknown or effect-less"):
        _export(blocked=[("fly", {"slot": "slot1"})])


def _slot_policy(failure):
    """cannot_reach(<slot>, <obj>) -> never place anything into that slot."""
    blocks = []
    for fact in failure["facts"]:
        atom = parse_atom(fact)
        if atom.name == "cannot_reach":
            blocks.append(("place", {"slot": atom.args[0]}))
    return blocks


def _loop(solve, fail_on):
    """Execute by 'placing' the planned steps; the first placement into
    ``fail_on`` fails with cannot_reach."""
    world, executed, injected = set(), [], []

    def plan(blocked):
        steps = solve(_export(blocked), world)
        return skeleton_document(steps, mission_id="Box")

    def execute(document):
        for transaction in document["root"]["children"]:
            op = transaction["children"][0]
            slot = op["parameters"]["slot"]
            if slot == fail_on and not injected:
                injected.append(slot)
                return {
                    "step": transaction["id"],
                    "capability": op["capability"],
                    "parameters": op["parameters"],
                    "facts": [f"cannot_reach({slot}, box)"],
                }
            executed.append(slot)
            world.add(f"placed({op['parameters']['obj']})")
        return None

    return plan_execute_repair(plan, execute, _slot_policy), executed


def _first_fit(export, world):
    """A stand-in planner: the first slot not blocked, unless already placed."""
    if "placed(box)" in world:
        return []
    blocked = {line.split()[-1].rstrip(")") for line in export.problem.split("(")
               if line.startswith("blocked_place__slot")}  # fmt: skip
    slot = next(s for s in ("slot1", "slot2") if s not in blocked)
    return [("place", {"obj": "box", "slot": slot})]


def test_an_injected_cannot_reach_is_replanned_around():
    outcome, executed = _loop(_first_fit, fail_on="slot1")
    assert outcome.success and len(outcome.rounds) == 2
    assert outcome.blocked == [("place", {"slot": "slot1"})]
    assert executed == ["slot2"]
    assert outcome.rounds[0].failure["facts"] == ["cannot_reach(slot1, box)"]


def test_the_loop_stops_when_the_policy_has_nothing_new():
    calls = []

    def execute(document):
        calls.append(document)
        return {"step": "s", "capability": "place", "parameters": {}, "facts": []}

    outcome = plan_execute_repair(lambda blocked: {}, execute, lambda f: [])
    assert not outcome.success and len(calls) == 1
    assert "nothing new" in outcome.message


def test_no_plan_ends_the_loop_with_a_message():
    from long_tamp.tasks.task_planning.skeleton import NoPlanFound

    def plan(blocked):
        if blocked:
            raise NoPlanFound("fast-downward: UNSOLVABLE_PROVEN")
        return {}

    failure = {"step": "s", "capability": "place", "parameters": {"slot": "s1"}}
    outcome = plan_execute_repair(plan, lambda d: failure)
    assert not outcome.success and len(outcome.rounds) == 1
    assert outcome.message.startswith("no plan avoids")


def test_the_loop_is_bounded():
    failing = {"step": "s", "capability": "place", "parameters": {"slot": "x"}}
    n = iter(range(100))

    def execute(document):
        return {**failing, "parameters": {"slot": f"s{next(n)}"}}

    outcome = plan_execute_repair(
        lambda b: {}, execute, block_failed_step, max_rounds=3
    )
    assert not outcome.success and len(outcome.rounds) == 3
    assert "after 3 plans" in outcome.message


def test_end_to_end_with_a_real_planner():
    """Whichever slot the planner picks first fails; the replan picks the
    other one and completes."""
    pytest.importorskip("unified_planning")
    from long_tamp.tasks.task_planning.skeleton import UnifiedPlanningPlanner

    planner = UnifiedPlanningPlanner()
    first = planner.solve(_export())[0][1]["slot"]

    def solve(export, world):
        return [] if "placed(box)" in world else planner.solve(export)

    outcome, executed = _loop(solve, fail_on=first)
    other = {"slot1": "slot2", "slot2": "slot1"}[first]
    assert outcome.success and len(outcome.rounds) == 2
    assert executed == [other]


# --------------------------------------------------------------- screw assembly

import sys  # noqa: E402
from pathlib import Path  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script/screw_assembly"))
import screw_domain  # noqa: E402

CLAMP_STEP = {
    "holder": "ur10_left/gripper",
    "held": "part1/h_grasp",
    "tool_gripper": "ur10_right/gripper",
    "tool": "driver/h_grip",
    "clamp": "fixtures/clamp1",
    "seat": "part1/h_seat",
    "part": "part1",
    "hole1": "part1/h_hole1",
    "hole2": "part1/h_hole2",
}


def test_screw_policy_blocks_a_clamp_that_cannot_reach_a_seat():
    failure = {
        "step": "part1 A: clamp + screw",
        "capability": "clamp_and_screw",
        "parameters": CLAMP_STEP,
        "facts": [
            "refinement_failed(x)",
            "cannot_reach(fixtures/clamp1, part1/h_seat)",
        ],
    }
    assert screw_domain.repair_policy(failure) == [
        ("clamp_and_screw", {"clamp": "fixtures/clamp1", "seat": "part1/h_seat"})
    ]


def test_screw_policy_otherwise_blocks_the_failed_binding():
    failure = {
        "step": "part1 A0: grasp",
        "capability": "grasp",
        "parameters": {"gripper": "ur10_left/gripper", "handle": "part1/h_grasp"},
        "facts": [
            "refinement_failed(x)",
            "cannot_reach(ur10_left/gripper, part1/h_grasp)",
        ],
    }
    assert screw_domain.repair_policy(failure) == [
        ("grasp", {"gripper": "ur10_left/gripper", "handle": "part1/h_grasp"})
    ]


def test_a_blocked_clamp_with_no_alternative_leaves_no_plan():
    """One part, one clamp: blocking it makes the goal unreachable, which the
    repair loop reports instead of replanning forever."""
    pytest.importorskip("unified_planning")
    from long_tamp.tasks.task_planning.skeleton import (
        NoPlanFound,
        UnifiedPlanningPlanner,
    )

    blocked = [
        ("clamp_and_screw", {"clamp": "fixtures/clamp1", "seat": "part1/h_seat"})
    ]
    with pytest.raises(NoPlanFound):
        UnifiedPlanningPlanner().solve(screw_domain.pddl_problem(1, blocked=blocked))
