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
        mine = [label.split()[0] for label in done if label.split()[1] == obj]
        # A chain that ends with a pour leaves the object in the hand: it is
        # put down to free the hand, unless it comes last (planner's choice).
        if mine[-1] == "putdown":
            mine.pop()
        assert mine == [
            {"pick": "pick", "in": "place", "pour": "pour"}[s[0]] for s in specs
        ], (obj, done)
    # Nothing put down on the way nor picked up again.
    assert not any(label.startswith("regrasp") for label in done), done


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
    plan runs without effect (-1: always), ``unsupported`` which raise
    ``UnsupportedGoal``, ``broken`` which raise ``PlanningError`` and
    ``ahead`` which, as the lookahead of a pick, fail it."""

    def __init__(self, env, fail=None, unsupported=(), broken=(), ahead=()):
        self.env, self.fail, self.unsupported = env, dict(fail or {}), set(unsupported)
        self.broken, self.ahead = set(broken), set(ahead)
        self.calls = []
        #: Per call, the lookahead it was given: ``(kind, object)`` or None.
        self.thens = []

    def plan(self, env, goal, name, record, then=None):
        from long_tamp.eval.robocerebra_policy import LookaheadFailed, UnsupportedGoal

        self.calls.append((goal.kind, goal.obj))
        self.thens.append(None if then is None else (then.kind, then.obj))
        key = (goal.kind, goal.obj)
        if then is not None and (then.kind, then.obj) in self.ahead:
            raise LookaheadFailed(
                f"{then.kind} of {goal.obj} after the pick does not plan",
                {"lookahead": {"message": "no release pose"}},
            )
        if key in self.unsupported:
            raise UnsupportedGoal(f"no region {goal.region!r} in the scene")
        if key in self.broken:
            from long_tamp.eval.robocerebra_plan import PlanningError

            tried = [{"handle": "grasp_0", "success": False, "message": "collides"}]
            raise PlanningError(f"no handle of {goal.obj} plans", {"tried": tried})
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
            env,
            options.pop("fail", None),
            options.pop("unsupported", ()),
            options.pop("broken", ()),
            options.pop("ahead", ()),
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
    # It was put down to free the hand, if anything was left to do after it.
    steps = [e["step"] for e in mission["executions"]]
    last = max(i for i, step in enumerate(steps) if step == "pour popcorn_1 plate_1")
    if steps[last + 1 :]:
        assert steps[last + 1] == "putdown popcorn_1", steps
    pours = [c for c in policy.calls if c == ("poured", "popcorn_1")]
    assert len(pours) == 3  # the first try and two retries


def test_an_unsupported_region_is_given_up_on_without_retries(fake):
    env, policy, row = fake(CASE3, unsupported=[("placed", "plate_1")])
    mission = row["mission"]
    assert "plate_1" in mission["given_up"]
    assert policy.calls.count(("placed", "plate_1")) == 1
    # The step that did not plan is kept, with why.
    (unplanned,) = mission["unplanned"]
    assert unplanned["facts"] == ["unsupported(plate_1)"]


def test_a_step_that_does_not_plan_keeps_why(fake):
    env, policy, row = fake(CASE3, broken=[("grasped", "plate_1")])
    mission = row["mission"]
    assert "plate_1" in mission["given_up"]
    assert policy.calls.count(("grasped", "plate_1")) == 1
    unplanned = mission["unplanned"]
    assert len(unplanned) == 1
    assert unplanned[0]["message"] == "PlanningError: no handle of plate_1 plans"
    assert unplanned[0]["tried"][0]["message"] == "collides"
    assert unplanned[0]["facts"][0].startswith("refinement_failed(")


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
    assert "supported_segment_success_rate" not in summary
    segmented = {**row, "protocol": None}
    with pytest.raises(ValueError, match="mission runs mixed"):
        merge_results(
            [{"cases": {"Ideal/case3": row}}, {"cases": {"Ideal/case3": segmented}}]
        )


def test_a_step_that_does_not_converge_is_cut_at_its_cap(fake, monkeypatch):
    # The pick counts, then the lift never ends: cut at 150 steps, it
    # succeeded; the rest of the rollout still has its budget. (A pick is
    # never the plan's last step, which would end the case before the cap.)
    label = "pick popcorn_1"
    monkeypatch.setattr(FakeController, "hang", {label})
    env, policy, row = fake(CASE3)
    assert row["success"], row["mission"]
    place = [e for e in row["mission"]["executions"] if e["step"] == label]
    assert [(e["steps_used"], e["status"], e["message"]) for e in place] == [
        (150, "success", "stalled")
    ]


def test_a_pick_is_checked_against_the_step_after_it():
    domain = RoboCerebraDomain(CASE3)
    place = domain.then("plate_1", "s0")
    assert (place.kind, place.obj, place.region) == ("placed", "plate_1", TABLE)
    pour = domain.then("popcorn_1", "s0")
    assert (pour.kind, pour.receiver) == ("poured", "plate_1")
    assert domain.then("plate_1", "s2") is None  # nothing after
    assert RoboCerebraDomain(CASE3, lookahead=False).then("plate_1", "s0") is None
    domain.defer("plate_1")
    assert "progress(plate_1, s2)" not in domain.goal()
    assert domain.resume() and not domain.resume()
    assert "progress(plate_1, s2)" in domain.goal()
    assert domain.then("plate_1", "s0") is None  # no longer checked


def test_an_object_whose_next_step_would_not_plan_goes_last(fake):
    key = ("poured", "popcorn_1")
    env, policy, row = fake(CASE3, ahead=[key], broken=[key])
    mission = row["mission"]
    assert mission["deferred"] == ["popcorn_1"]
    assert mission["unplanned"][0]["facts"] == ["lookahead_failed(popcorn_1)"]
    steps = [e["step"] for e in mission["executions"] if e["status"] == "success"]
    # Everything else first, then the popcorn's pick, unchecked, which counts.
    assert steps.index("pick popcorn_1") > max(
        i for i, step in enumerate(steps) if "popcorn" not in step
    ), steps
    picks = [
        t for c, t in zip(policy.calls, policy.thens) if c == ("grasped", "popcorn_1")
    ]
    assert picks == [key, None]
    # The pour does not plan: given up on, put down, the pick still counted.
    assert mission["given_up"] == ["popcorn_1"]
    assert row["total_completed"] == row["total"] - 2
    assert policy.calls.count(key) == 1  # not retried


def test_without_lookahead_no_pick_is_checked(fake):
    env, policy, row = fake(CASE3, lookahead=False)
    assert row["success"]
    assert policy.thens == [None] * len(policy.thens)


def test_a_held_export_moves_the_robot_and_the_object(tmp_path):
    from long_tamp.eval.robocerebra_scene import MANIFEST, held_export, load_export

    root = tmp_path / "scene"
    (root / "objects").mkdir(parents=True)
    (root / "objects" / "plate_1.srdf").write_text("<robot/>")
    manifest = {
        "robot": {"joints": {"robot0_joint1": 0.0, "gripper0_finger_joint1": 0.04}},
        "objects": {"plate_1": {"pose": [0, 0, 0, 0, 0, 0, 1]}},
    }
    (root / MANIFEST).write_text(json.dumps(manifest))
    scene = load_export(root)
    pose = [0.1, 0.2, 0.3, 0, 0, 0, 1]
    ahead = held_export(scene, "plate_1", {"robot0_joint1": 0.5}, pose, f"{root}-then")
    again = load_export(ahead.root)
    assert again.robot_joints() == {
        "robot0_joint1": 0.5,
        "gripper0_finger_joint1": 0.04,
    }
    assert again.object_config("plate_1") == pose
    assert (ahead.root / "objects" / "plate_1.srdf").exists()
    assert scene.object_config("plate_1") == [0, 0, 0, 0, 0, 0, 1]  # untouched
