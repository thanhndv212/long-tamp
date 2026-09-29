"""PDDL export of capabilities, a world state and a goal (issue #13).

The translation is checked on its own; the round trip (export -> a reference
planner -> ``from_pddl_plan`` -> every step checked against long_tamp's own
predicate semantics) runs when Unified Planning and its Fast Downward engine
are installed (``pip install long-tamp[planning]``).
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

from long_tamp.tasks.task_planning import CapabilityDescriptor
from long_tamp.tasks.task_planning.pddl import from_pddl_plan, to_pddl
from long_tamp.tasks.task_planning.predicates import (
    Literal,
    apply_effects,
    holds,
    parse_state,
)

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script/screw_assembly"))
import screw_domain  # noqa: E402

GRASP = CapabilityDescriptor(
    "grasp",
    "1.0",
    {"block": str, "gripper": str, "handle": str},
    preconditions=("not holds(?gripper, _)",),
    effects=("holds(?gripper, ?handle)",),
    restartable=True,
)
RELEASE = CapabilityDescriptor(
    "release",
    "1.0",
    {"gripper": str},
    preconditions=("holds(?gripper, _)",),
    effects=("not holds(?gripper, _)",),
    restartable=True,
)
GUARD = CapabilityDescriptor("done", "1.0", {"x": str}, preconditions=("placed(?x)",))


def test_actions_translate_literals_and_wildcards():
    x = to_pddl([GRASP, RELEASE, GUARD], init=[], goal=["holds(left/g, ball/h)"])
    assert "(:action grasp" in x.domain and "(:action release" in x.domain
    assert "(:action done" not in x.domain  # no effects: a guard, not an action
    # "block" appears in no literal: not a planning parameter.
    assert x.parameters["grasp"] == ["gripper", "handle"]
    assert "(not (exists (?w0) (holds ?gripper ?w0)))" in x.domain
    assert "(forall (?w0) (not (holds ?gripper ?w0)))" in x.domain
    assert ":conditional-effects" in x.domain and ":existential-preconditions" in x.domain
    assert "(holds left__g ball__h)" in x.problem


def test_names_are_valid_pddl_and_map_back():
    x = to_pddl([GRASP], init=["holds(2nd/g, h)"], goal=["holds(left/g, ball/h)"])
    assert x.names["left__g"] == "left/g" and x.names["ball__h"] == "ball/h"
    assert x.names["o_2nd__g"] == "2nd/g"  # a name must start with a letter
    steps = from_pddl_plan(x, ["(GRASP LEFT__G BALL__H)"])  # planners upper-case
    assert steps == [("grasp", {"gripper": "left/g", "handle": "ball/h"})]


def test_static_preconditions_bound_parameters_and_are_checked():
    x = to_pddl(
        [GRASP],
        init=["can_grasp(left/g, ball/h)"],
        goal=["holds(left/g, ball/h)"],
        static_preconditions={"grasp": ("can_grasp(?gripper, ?handle)",)},
    )
    assert "(can_grasp ?gripper ?handle)" in x.domain
    with pytest.raises(ValueError, match="unknown parameter"):
        to_pddl([GRASP], [], [], static_preconditions={"grasp": ("ok(?nope)",)})
    with pytest.raises(ValueError, match="unknown or effect-less"):
        to_pddl([GRASP], [], [], static_preconditions={"fly": ("ok(?x)",)})


def test_inconsistent_arities_are_rejected():
    with pytest.raises(ValueError, match="arities"):
        to_pddl([GRASP], init=["holds(a)"], goal=[])


def test_bad_plan_steps_are_rejected():
    x = to_pddl([GRASP], init=[], goal=["holds(a, b)"])
    with pytest.raises(ValueError, match="unknown action"):
        from_pddl_plan(x, ["(fly a b)"])
    with pytest.raises(ValueError, match="expected 2 arguments"):
        from_pddl_plan(x, ["(grasp a)"])
    with pytest.raises(ValueError, match="unknown object"):
        from_pddl_plan(x, ["(grasp a zzz)"])


@pytest.mark.parametrize("n_parts", [1, 2])
def test_screw_assembly_exports_its_mission(n_parts):
    x = screw_domain.pddl_problem(n_parts)
    assert set(x.parameters) == {"grasp", "clamp_and_screw", "release", "rack"}
    assert "(:action home" not in x.domain  # no effects
    for i in range(1, n_parts + 1):
        assert f"(screwed part{i} part{i}__h_hole2)" in x.problem


# ------------------------------------------------------------------ round trip


def _check_plan(n_parts, steps, state):
    """Replay a skeleton with long_tamp's own semantics: every step's
    preconditions (and static preconditions) hold, and the goal holds at the end."""
    descriptors = screw_domain.descriptors(n_parts)
    state = parse_state([*screw_domain.static_facts(n_parts), *state])
    for capability, parameters in steps:
        descriptor = descriptors[capability]
        static = screw_domain.STATIC_PRECONDITIONS.get(capability, ())
        for literal in (*map(Literal.parse, static), *descriptor.precondition_literals):
            assert holds(literal.ground(parameters), state), (capability, literal)
        effects = [lit.ground(parameters) for lit in descriptor.effect_literals]
        state = apply_effects(state, effects)
    for text in screw_domain.mission_goal(n_parts):
        assert holds(Literal.parse(text), state), text


def _solve(export, tmp_path):
    up = pytest.importorskip("unified_planning")
    pytest.importorskip("up_fast_downward")
    from unified_planning.io import PDDLReader
    from unified_planning.shortcuts import OneshotPlanner

    up.shortcuts.get_environment().credits_stream = None
    (tmp_path / "domain.pddl").write_text(export.domain)
    (tmp_path / "problem.pddl").write_text(export.problem)
    problem = PDDLReader().parse_problem(
        str(tmp_path / "domain.pddl"), str(tmp_path / "problem.pddl")
    )
    with OneshotPlanner(name="fast-downward") as planner:
        result = planner.solve(problem)
    assert result.plan is not None, result.status
    return [
        (a.action.name, *(str(p) for p in a.actual_parameters))
        for a in result.plan.actions
    ]


@pytest.mark.parametrize("n_parts", [1, 2, 4])
def test_round_trip_through_a_reference_planner(n_parts, tmp_path):
    export = screw_domain.pddl_problem(n_parts)
    steps = from_pddl_plan(export, _solve(export, tmp_path))
    _check_plan(n_parts, steps, state=[])
    kinds = [capability for capability, _ in steps]
    assert kinds.count("clamp_and_screw") == n_parts
    assert kinds[-1] == "rack"


def test_round_trip_from_a_partly_done_state(tmp_path):
    """Part 1 done, driver in hand: the plan is only part 2's work and the rack."""
    state = [
        "holds(ur10_right/gripper, driver/h_grip)",
        "holds(fixtures/clamp1, part1/h_seat)",
        "screwed(part1, part1/h_hole1)",
        "screwed(part1, part1/h_hole2)",
    ]
    export = screw_domain.pddl_problem(2, state)
    steps = from_pddl_plan(export, _solve(export, tmp_path))
    _check_plan(2, steps, state)
    assert all("part1" not in str(p) for _, p in steps)
    assert [c for c, _ in steps].count("grasp") == 1  # part 2 only
