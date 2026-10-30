"""RoboCerebra as a mission domain (#186): the symbolic half.

The goals are copies of the benchmark's Ideal cases (``goal.json``), so the
tests need neither the benchmark nor HPP; the planning tests need a task
planner (Fast Downward or Unified Planning). With ``RC_BENCH`` set, every
Ideal case's own ``goal.json`` is planned too.
"""

from __future__ import annotations

import json
import types
import os
import sys
from pathlib import Path

import numpy as np
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


# -- the mission in an env -----------------------------------------------------
#
# A fake env counts each object's goals as the checker does, and the fake
# policy's "plans" act on it directly once their controller has run, so the
# mission loop (plan, run, check, replan, give up) runs without the
# benchmark or HPP.


class FakeEnv:
    def __init__(self, goal, steps=10):
        self.goal = {
            o: [[str(s[0]).lower(), *s[1:]] for s in v] for o, v in goal.items()
        }
        self.info = {
            "case": "Ideal/fake",
            "goal": goal,
            "steps": [f"step {k}" for k in range(steps)],
            "objects": sorted(goal),
        }
        self.reset()

    def load(self, case):
        return self.info

    def reset(self, step=0, seed=0):
        self.progress = {obj: 0 for obj in self.goal}
        self.held = None
        self.t = 0
        return {"obs": {"robot0_eef_pos": [0.0, 0.0, 0.0]}}

    def next_goal(self, obj):
        k = self.progress[obj]
        return self.goal[obj][k] if k < len(self.goal[obj]) else None

    def check(self):
        done = sum(self.progress.values())
        total = sum(len(v) for v in self.goal.values())
        return {
            "progress": dict(self.progress),
            "total_completed": done,
            "total": total,
            "all_done": done == total,
        }

    def step(self, action):
        if action[6] < 0:
            self.held = None  # the gripper opens
        self.t += 1
        return {**self.check(), "obs": {"robot0_eef_pos": [0.0, 0.0, 0.0]}}

    def export_scene(self, out_dir=None):
        from long_tamp.eval.robocerebra_plan import LEFT_FINGER, RIGHT_FINGER

        contacts = (
            [
                {"link": LEFT_FINGER[0], "object": self.held},
                {"link": RIGHT_FINGER[0], "object": self.held},
            ]
            if self.held
            else []
        )
        return {"manifest": {"robot_contacts": contacts}}


class FakeController:
    """Runs ``steps`` env steps, then applies its effect to the env; with
    ``hang``, never done after that (a retreat that does not converge)."""

    hang: set = set()

    def __init__(self, effect, tcp_start=None, info=None, steps=3):
        self.effect, self.info, self.left = effect, info or {}, steps
        self.hangs = info is not None and info.get("step") in self.hang

    @property
    def done(self):
        return self.left <= 0 and not self.hangs

    def __call__(self, obs):
        self.left -= 1
        if self.left == 0 and self.effect is not None:
            self.effect()
        return np.zeros(7)


class FakePolicy:
    """Plans every goal; ``fail`` says how many times a ``(kind, object)``
    plan runs without effect (-1: always), ``unsupported`` which raise."""

    def __init__(self, env, fail=None, unsupported=()):
        self.env, self.fail, self.unsupported = env, dict(fail or {}), set(unsupported)
        self.calls = []

    def plan(self, env, goal, name, record):
        from long_tamp.eval.robocerebra_policy import UnsupportedGoal

        self.calls.append((goal.kind, goal.obj))
        key = (goal.kind, goal.obj)
        if key in self.unsupported:
            raise UnsupportedGoal(f"no region {goal.region!r} in the scene")
        effect = None if self._fails(key) else (lambda: self._apply(goal))
        return types.SimpleNamespace(stages=effect, tcp_start=None, info={})

    def _fails(self, key):
        left = self.fail.get(key, 0)
        if left:
            self.fail[key] = left - 1
        return left != 0

    def _apply(self, goal):
        env, nxt = self.env, self.env.next_goal(goal.obj)
        if goal.kind == "grasped":
            env.held = goal.obj
            counts = nxt is not None and nxt[0] == "pick"
        elif goal.kind == "placed":
            counts = env.held == goal.obj and nxt == ["in", goal.obj, goal.region]
            env.held = None
        else:
            counts = env.held == goal.obj and nxt == ["pour", goal.obj, goal.receiver]
        if counts:
            env.progress[goal.obj] += 1


@pytest.fixture
def fake(monkeypatch):
    from long_tamp.eval import robocerebra_mission

    _task_planner_or_skip()
    monkeypatch.setattr(robocerebra_mission, "SegmentController", FakeController)

    def run(goal, seed=0, **options):
        env = FakeEnv(goal)
        policy = FakePolicy(
            env, options.pop("fail", None), options.pop("unsupported", ())
        )
        row = robocerebra_mission.run_mission_case(
            env, env.info, seed, policy, **options
        )
        return env, policy, row

    return run


def test_a_case_runs_as_one_mission(fake):
    env, policy, row = fake(CASE3)
    assert row["success"] and row["completed_share"] == 1.0
    mission = row["mission"]
    assert mission["given_up"] == [] and len(mission["rounds"]) == 1
    assert mission["rounds"][0]["task_replans"] == 0
    # Every goal planned once, in its object's order.
    goals = [c for c in policy.calls]
    assert len(goals) == sum(len(v) for v in CASE3.values()), goals
    assert mission["steps_used"] <= mission["budget"] == 150 * 10


def test_a_missed_goal_is_planned_again_from_the_observed_world(fake):
    env, policy, row = fake(CASE3, fail={("poured", "popcorn_1"): 1})
    assert row["success"], row["mission"]
    first = row["mission"]["rounds"][0]
    assert first["retries"] == 1
    missed = [e for e in row["mission"]["executions"] if e["status"] == "failure"]
    assert [e["facts"] for e in missed] == [["goal_missed(popcorn_1, s1)"]]


def test_a_dropped_object_is_regrasped(fake, monkeypatch):
    # The pick counts, but the cookies fall right after it: the counter
    # stays, so the replan grasps them again without a pick.
    original, dropped = FakePolicy._apply, []

    def drop_once(self, goal):
        original(self, goal)
        if goal.kind == "grasped" and goal.obj == "cookies_1" and not dropped:
            dropped.append(goal)
            self.env.held = None

    monkeypatch.setattr(FakePolicy, "_apply", drop_once)
    env, policy, row = fake(CASE9)
    # Every supported goal (the bowl's stop at the microwave's open).
    assert row["total_completed"] == 5, row["mission"]
    steps = [e["step"] for e in row["mission"]["executions"]]
    assert "regrasp cookies_1" in steps, steps
    assert steps.count("pick cookies_1") == 1, steps


def test_an_object_that_keeps_failing_is_given_up_on(fake):
    env, policy, row = fake(CASE3, fail={("poured", "popcorn_1"): -1})
    mission = row["mission"]
    assert mission["given_up"] == ["popcorn_1"]
    assert not row["success"]
    # Everything else is done, the popcorn's place too is lost.
    assert row["total_completed"] == row["total"] - 2
    # It was put down to free the hand for the others.
    assert "putdown popcorn_1" in [e["step"] for e in mission["executions"]]
    pours = [c for c in policy.calls if c == ("poured", "popcorn_1")]
    assert len(pours) == 3  # the first try and two retries


def test_an_unsupported_region_is_given_up_on_without_retries(fake):
    env, policy, row = fake(CASE3, unsupported=[("placed", "plate_1")])
    mission = row["mission"]
    assert "plate_1" in mission["given_up"]
    assert policy.calls.count(("placed", "plate_1")) == 1


def test_the_budget_is_shared_by_the_whole_rollout(fake):
    env, policy, row = fake(CASE3, step_factor=1)
    mission = row["mission"]
    assert mission["budget"] == 10
    assert mission["steps_used"] == 10
    assert not row["success"]
    facts = [f for e in mission["executions"] for f in e.get("facts", [])]
    assert "budget_spent()" in facts


def test_mission_rows_summarize_and_refuse_to_merge_with_segments(fake, monkeypatch):
    from long_tamp.eval import robocerebra_mission
    from long_tamp.eval.robocerebra_runner import merge_results, summarize

    env = FakeEnv(CASE3)
    row = robocerebra_mission.evaluate_mission_case(
        env, "Ideal/case3", seeds=[0, 1], policy=FakePolicy(env)
    )
    assert row["protocol"] == "mission" and row["success_rate"] == 1.0
    summary = summarize({"Ideal/case3": row})
    assert summary["completed_share"] == 1.0
    assert summary["mission_rollouts"] == 2 and summary["objects_given_up"] == 0
    segmented = {**row, "protocol": None}
    with pytest.raises(ValueError, match="mission runs mixed"):
        merge_results(
            [{"cases": {"Ideal/case3": row}}, {"cases": {"Ideal/case3": segmented}}]
        )


def test_a_step_that_does_not_converge_is_cut_at_its_cap(fake, monkeypatch):
    # The place counts, then the retreat never ends: cut at 150 steps, it
    # succeeded; the rest of the rollout still has its budget.
    label = f"place popcorn_1 {TABLE}"
    monkeypatch.setattr(FakeController, "hang", {label})
    env, policy, row = fake(CASE3)
    assert row["success"], row["mission"]
    place = [e for e in row["mission"]["executions"] if e["step"] == label]
    assert [(e["steps_used"], e["status"], e["message"]) for e in place] == [
        (150, "success", "stalled")
    ]
