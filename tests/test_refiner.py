"""The refiner interface (issue #10, ADR-0001): ``GraspSequenceRefiner`` over
the recovery ladder, and the facts a failed refinement reports. Uses the
recovery tests' stub planner; no solver."""

import sys
from pathlib import Path

import pytest

from long_tamp.tasks.refiner import (
    GraspSequenceRefiner,
    Lookahead,
    RefinementStep,
    failure_facts,
)
from long_tamp.tasks.task_planning.predicates import parse_atom
from tests.test_block_recovery import BLOCK, DONE, EDGE, ENTRY, FakePlanner

STEP = RefinementStep(label="part1 A: clamp + screw", sequence=tuple(BLOCK))


def _refine(planner, step=STEP, **kw):
    return GraspSequenceRefiner(planner, verbose=False, **kw).refine(step, ENTRY)


def test_success_returns_the_final_config_and_phases():
    planner = FakePlanner(plan_succeeds=True)
    planner.phase_results = [{"phase": 1}, {"phase": 2}]
    r = _refine(planner)
    assert r.success and r.final_config == DONE
    assert r.phases == [{"phase": 1}, {"phase": 2}]
    assert r.facts == [] and r.replans == 0


def test_recovery_is_the_ladders():
    r = _refine(FakePlanner(resume_succeeds_after=3))
    assert r.success and r.resumes == 3 and r.replans == 0


def test_an_unreachable_phase_is_reported_as_a_fact():
    """Solver-only failures on phase 2: the ladder replans until it gives up,
    and the refiner names the phase the task planner should avoid."""
    r = _refine(FakePlanner(collisions=0), max_replans=1)
    assert not r.success and r.final_config == ENTRY
    assert "unreachable" in r.message
    assert r.facts == [
        "refinement_failed(part1_A:_clamp_+_screw)",
        "ik_unreachable(tool/g_tip, part/h_hole1)",
    ]
    for fact in r.facts:
        parse_atom(fact)  # ground atoms in the TaskPlan predicate language


def test_a_stuck_phase_is_reported_as_cannot_reach():
    r = _refine(FakePlanner(collisions=5), max_replans=0, resume_limit=2)
    assert not r.success
    assert r.facts[-1] == "cannot_reach(tool/g_tip, part/h_hole1)"


def test_recovery_options_reach_the_ladder():
    planner = FakePlanner(collisions=0)
    r = _refine(planner, max_replans=0, unreachable_resumes=0, resume_limit=4)
    assert r.resumes == 4  # the fast path was disabled, so it hit the limit


def test_lookahead_hints_are_drawn_for_each_attempt():
    planner = FakePlanner(broken_hints=(0,), resume_succeeds_after=1)
    chains = iter([[[0.1]], [[0.2]]])
    planner.find_feasible_phase_target = lambda **kw: next(chains)
    step = RefinementStep("s", tuple(BLOCK), lookahead=Lookahead(pair=(0, 1)))
    r = _refine(planner, step=step)
    assert r.success and r.replans == 1
    assert planner.plan_calls == [{0: [[0.1]]}, {0: [[0.2]]}]


@pytest.mark.parametrize(
    "failure, expected",
    [
        (None, []),
        ({"kind": "no_resumable_state", "phase_idx": None, "edge": None}, []),
        (
            {"kind": "hint_chain_broken", "phase_idx": 0, "edge": None},
            ["lookahead_failed(arm/g, part/h_seat)"],
        ),
        ({"kind": "stuck", "phase_idx": 9, "edge": EDGE}, []),  # out of range
    ],
)
def test_failure_facts(failure, expected):
    facts = failure_facts(STEP, failure)
    assert facts[0] == "refinement_failed(part1_A:_clamp_+_screw)"
    assert facts[1:] == expected


def test_a_failed_release_names_the_handle_it_held():
    step = RefinementStep("r", (("tip", "part/h_hole1"), ("tip", None)))
    facts = failure_facts(step, {"kind": "stuck", "phase_idx": 1, "edge": EDGE})
    assert facts[1] == "release_infeasible(tip, part/h_hole1)"
    alone = RefinementStep("r", (("arm/g", None),))
    facts = failure_facts(alone, {"kind": "stuck", "phase_idx": 0, "edge": EDGE})
    assert facts[1] == "release_infeasible(arm/g, none)"


def test_a_collision_is_reported_as_blocks():
    error = (
        "TransitionPlanner.computePath failed for transit edge x: Collision "
        "between object panda_right/panda_link6_0 and ground/ground_base_0"
    )
    failure = {"kind": "stuck", "phase_idx": 0, "edge": EDGE, "error": error}
    facts = failure_facts(STEP, failure)
    assert facts[-1] == "blocks(panda_right/panda_link6, ground/ground_base)"
    for fact in facts:
        parse_atom(fact)


def test_screw_assembly_blocks_become_refinement_steps():
    sys.path.insert(
        0, str(Path(__file__).resolve().parents[1] / "script/screw_assembly")
    )
    import screw_domain

    blocks = [b for b in screw_domain.build_mission(1) if "seq" in b]
    steps = {b["label"]: screw_domain.refinement_step(b) for b in blocks}
    a = steps["part1 A: clamp + screw"]
    assert a.lookahead == Lookahead(pair=(0, 1), also=(3,), verify_paths=True)
    assert a.sequence[0] == ("fixtures/clamp1", "part1/h_seat")
    assert all(s.lookahead is None for label, s in steps.items() if " A:" not in label)
