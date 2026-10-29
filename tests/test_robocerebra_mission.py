"""RoboCerebra as a mission domain (#186): the symbolic half.

The goals are copies of the benchmark's Ideal cases (``goal.json``), so the
tests need neither the benchmark nor HPP; the planning tests need a task
planner (Fast Downward or Unified Planning). With ``RC_BENCH`` set, every
Ideal case's own ``goal.json`` is planned too.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import pytest

from long_tamp.eval.robocerebra_mission import (
    RoboCerebraDomain,
    chains,
    observed_atoms,
)
from long_tamp.eval.robocerebra_plan import GRIPPER
from long_tamp.mission import check_domain

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_mission_runner import _task_planner_or_skip  # noqa: E402

TABLE = "coffee_table_table_region"
#: Ideal/case3: a plate to place, three pours over it.
CASE3 = {
    "plate_1": [["Pick", "plate_1"], ["In", "plate_1", TABLE]],
    "popcorn_1": [
        ["Pick", "popcorn_1"],
        ["Pour", "popcorn_1", "plate_1"],
        ["In", "popcorn_1", TABLE],
    ],
    "cookies_1": [
        ["Pick", "cookies_1"],
        ["Pour", "cookies_1", "plate_1"],
        ["In", "cookies_1", TABLE],
    ],
    "wine_bottle_1": [["Pick", "wine_bottle_1"], ["Pour", "wine_bottle_1", "plate_1"]],
}
#: Ideal/case6: the butter's chain starts with an unsupported open.
CASE6 = {
    "butter_1": [
        ["Open", "short_cabinet_1"],
        ["Pick", "butter_1"],
        ["In", "butter_1", "short_cabinet_1_middle_region"],
    ],
    "tomato_sauce_1": [
        ["Pick", "tomato_sauce_1"],
        ["Pour", "tomato_sauce_1", "plate_1"],
        ["In", "tomato_sauce_1", TABLE],
    ],
}
#: Ideal/case9: the bowl goes to the table, then (after an open) the microwave.
CASE9 = {
    "white_bowl_1": [
        ["Pick", "white_bowl_1"],
        ["In", "white_bowl_1", TABLE],
        ["Open", "microwave_1"],
        ["Pick", "white_bowl_1"],
        ["In", "white_bowl_1", "microwave_1_heating_region"],
    ],
    "cookies_1": [
        ["Pick", "cookies_1"],
        ["Pour", "cookies_1", "white_bowl_1"],
        ["In", "cookies_1", TABLE],
    ],
}


def plan(domain, state, goal=None):
    from long_tamp.tasks.task_planning.skeleton import default_planner

    _task_planner_or_skip()
    return default_planner().solve(domain.pddl_problem(state, goal=goal))


def labels(domain, steps):
    return [
        domain.expand_step(i, c, dict(p), {})[0][2] for i, (c, p) in enumerate(steps)
    ]


@pytest.mark.parametrize("goal", [CASE3, CASE6, CASE9])
def test_every_case_is_a_consistent_domain(goal):
    assert check_domain(RoboCerebraDomain(goal)) == []


def test_chains_stop_at_the_first_unsupported_goal():
    cut = chains(CASE9)
    assert [s[0] for s in cut["white_bowl_1"]] == ["pick", "in"]
    assert chains(CASE6)["butter_1"] == []
    domain = RoboCerebraDomain(CASE6)
    # Nothing to do for the butter: the checker never gets past its open.
    assert domain.goal() == ["progress(tomato_sauce_1, s3)"]
    assert domain.unsupported() == {"butter_1": CASE6["butter_1"]}
    assert domain.goal(without=["tomato_sauce_1"]) == []


def test_the_static_facts_are_the_chains():
    facts = set(RoboCerebraDomain(CASE3).static_facts())
    assert {
        "goal_pick(popcorn_1, s0)",
        "goal_pour(popcorn_1, s1, plate_1)",
        f"goal_place(popcorn_1, s2, {TABLE})",
        "next(s2, s3)",
        f"gripper({GRIPPER})",
    } <= facts


def test_the_plan_from_the_start_reaches_every_goal_in_order():
    domain = RoboCerebraDomain(CASE3)
    steps = plan(domain, domain.initial_atoms())
    done = labels(domain, steps)
    for obj, specs in domain.chains.items():
        mine = [label for label in done if label.split()[1] == obj]
        assert [m.split()[0] for m in mine] == [
            {"pick": "pick", "in": "place", "pour": "pour"}[s[0]] for s in specs
        ], (obj, done)
    # Nothing put down on the way nor picked up again: every step a goal.
    assert len(done) == sum(len(specs) for specs in domain.chains.values()), done


def test_a_wrong_object_in_hand_is_put_down_first():
    domain = RoboCerebraDomain(CASE3)
    progress = {"plate_1": 2, "popcorn_1": 0, "cookies_1": 3, "wine_bottle_1": 2}
    # The hand still holds the cookies, already placed by the checker's count.
    state = observed_atoms(progress, "cookies_1")
    done = labels(domain, plan(domain, state))
    assert done[0] == "putdown cookies_1"
    assert done[1:] == [
        "pick popcorn_1",
        "pour popcorn_1 plate_1",
        f"place popcorn_1 {TABLE}",
    ]


def test_reached_goals_are_skipped_and_a_dropped_object_regrasped():
    domain = RoboCerebraDomain(CASE9)
    # The cookies were picked (counted) and fell: grasp them again, no pick.
    state = observed_atoms({"white_bowl_1": 2, "cookies_1": 1}, None)
    done = labels(domain, plan(domain, state))
    assert done == [
        "regrasp cookies_1",
        "pour cookies_1 white_bowl_1",
        f"place cookies_1 {TABLE}",
    ]


def test_a_held_object_without_goals_is_put_down():
    domain = RoboCerebraDomain(CASE6, others=["ketchup_1", "tomato_sauce_1"])
    assert domain.others == ("ketchup_1",)
    state = observed_atoms({"butter_1": 0, "tomato_sauce_1": 0}, "ketchup_1")
    done = labels(domain, plan(domain, state))
    assert done[0] == "putdown ketchup_1"
    assert done[1:] == [
        "pick tomato_sauce_1",
        "pour tomato_sauce_1 plate_1",
        f"place tomato_sauce_1 {TABLE}",
    ]


def test_an_object_given_up_on_is_put_down_where_its_chain_stands():
    domain = RoboCerebraDomain(CASE3)
    # The popcorn is held for its pour, which keeps failing: give it up.
    state = observed_atoms(
        {"plate_1": 2, "popcorn_1": 1, "cookies_1": 3, "wine_bottle_1": 0}, "popcorn_1"
    )
    domain.give_up("popcorn_1")
    assert "progress(popcorn_1, s3)" not in domain.goal()
    done = labels(domain, plan(domain, state))
    assert done == [
        "putdown popcorn_1",
        "pick wine_bottle_1",
        "pour wine_bottle_1 plate_1",
    ]


@pytest.mark.parametrize("case", range(1, 11))
def test_every_ideal_case_plans_its_supported_goals(case):
    bench = os.environ.get("RC_BENCH")
    if not bench:
        pytest.skip("RC_BENCH is not set")
    raw = json.loads((Path(bench) / "Ideal" / f"case{case}" / "goal.json").read_text())
    domain = RoboCerebraDomain(
        {o: [i["state_pair"] for i in v] for o, v in raw.items()}
    )
    assert check_domain(domain) == []
    done = labels(domain, plan(domain, domain.initial_atoms()))
    # Every goal once; a putdown only to free the hand after a final pour.
    goals = [label for label in done if label.split()[0] in ("pick", "place", "pour")]
    assert len(goals) == sum(len(specs) for specs in domain.chains.values()), done
    assert all(label.split()[0] != "regrasp" for label in done), done
