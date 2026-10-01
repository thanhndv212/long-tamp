"""The execution supervisor (#88): goal-level decisions after the repair loop
gives up, within limits, with an escalation report. No API calls."""

import json
from types import SimpleNamespace

from long_tamp.ai import CallRecord, make_client
from long_tamp.tasks.task_planning.language import Vocabulary
from long_tamp.tasks.task_planning.supervisor import (
    Decision,
    Limits,
    Situation,
    check_decision,
    supervise,
)

GOAL = ["screwed(p1, h1)", "screwed(p2, h2)", "racked(d)"]
VOCAB = Vocabulary(
    predicates={"screwed": 2, "racked": 1},
    objects=("p1", "h1", "p2", "h2", "d"),
    state=("screwed(p1, h1)",),
)


def situation(goal=GOAL, history=()):
    return Situation(
        instruction="assemble both parts and rack the driver",
        original_goal=list(GOAL),
        goal=list(goal),
        failure={
            "step": "p2 A",
            "capability": "clamp_and_screw",
            "facts": ["cannot_reach(c2, p2)"],
        },
        message="no plan avoids [...]",
        blocked=[["clamp_and_screw", {"clamp": "c1"}]],
        vocabulary=VOCAB,
        history=list(history),
    )


# -- the check -------------------------------------------------------------------


def test_a_relaxed_goal_keeps_a_strict_subset_of_the_original():
    ok = Decision("relax_goal", ["screwed(p1, h1)", "racked(d)"])
    assert check_decision(ok, situation(), Limits()) == []
    invented = Decision("relax_goal", ["screwed(p1, h1)", "racked(p1)"])
    assert (
        "only keep literals of the original goal"
        in check_decision(invented, situation(), Limits())[0]
    )
    same = Decision("relax_goal", list(GOAL))
    assert "must drop something" in check_decision(same, situation(), Limits())[0]
    assert (
        "non-empty"
        in check_decision(Decision("relax_goal", []), situation(), Limits())[0]
    )


def test_a_relaxed_goal_must_be_reachable():
    d = Decision("relax_goal", ["racked(d)"])
    errors = check_decision(d, situation(), Limits(), reachable=lambda g: "blocked")
    assert "no plan reaches the relaxed goal" in errors[0]


def test_actions_are_limited_and_a_failure_is_retried_once():
    assert "unknown action" in check_decision(Decision("fly"), situation(), Limits())[0]
    no_retry = Limits(allowed=("relax_goal", "abort"))
    assert "not allowed" in check_decision(Decision("retry"), situation(), no_retry)[0]
    assert check_decision(Decision("escalate"), situation(), no_retry) == []  # always
    retried = situation(
        history=[{"action": "retry", "message": "no plan avoids [...]"}]
    )
    assert "already retried" in check_decision(Decision("retry"), retried, Limits())[0]


# -- the loop, with a fake model ---------------------------------------------------


def fake_client(*answers, tokens=0):
    queue = list(answers)

    def create(**kwargs):
        msg = SimpleNamespace(content=json.dumps(queue.pop(0)), refusal=None)
        return SimpleNamespace(
            choices=[SimpleNamespace(message=msg, finish_reason="stop")],
            usage=SimpleNamespace(prompt_tokens=tokens, completion_tokens=0),
        )

    sdk = SimpleNamespace(
        chat=SimpleNamespace(completions=SimpleNamespace(create=create))
    )
    return make_client("openai:m", sdk=sdk, base_url="http://gw")


class Runs:
    """run_repair: fails while part 2 is in the goal, succeeds once it isn't."""

    def __init__(self):
        self.goals = []

    def __call__(self, goal):
        self.goals.append(list(goal))
        return {"success": not any("p2" in g for g in goal)}


def situation_of(result, goal):
    return situation(goal)


def test_the_supervisor_relaxes_the_goal_and_the_mission_continues(tmp_path):
    runs = Runs()
    client = fake_client(
        {
            "action": "relax_goal",
            "goal": ["screwed(p1, h1)", "racked(p1)"],
            "reason": "x",
        },
        {
            "action": "relax_goal",
            "goal": ["screwed(p1, h1)", "racked(d)"],
            "reason": "part 2 has no clamp left",
        },
    )
    out = supervise(
        runs, situation_of, "assemble both", GOAL, client, report_dir=tmp_path
    )
    assert out.success and out.goal == ["screwed(p1, h1)", "racked(d)"]
    assert runs.goals[1] == out.goal
    (decision,) = out.decisions
    assert decision["action"] == "relax_goal" and len(decision["rejected"]) == 1
    assert not (tmp_path / "escalation.md").exists()  # no stop, no report


def test_no_valid_decision_escalates_with_a_report(tmp_path):
    bad = {"action": "relax_goal", "goal": ["racked(p9)"], "reason": "?"}
    out = supervise(
        Runs(),
        situation_of,
        "assemble both",
        GOAL,
        fake_client(bad, bad),
        report_dir=tmp_path,
    )
    assert not out.success and out.stopped.startswith("escalate")
    assert out.decisions[0]["fallback"]
    report = json.loads((tmp_path / "escalation.json").read_text())
    assert report["stopped"] == "escalate" and report["failure"]["step"] == "p2 A"
    assert "Mission stopped: escalate" in (tmp_path / "escalation.md").read_text()


def test_an_abort_stops_with_a_report(tmp_path):
    client = fake_client(
        {"action": "abort", "goal": [], "reason": "nothing left to do"}
    )
    out = supervise(
        Runs(), situation_of, "assemble both", GOAL, client, report_dir=tmp_path
    )
    assert not out.success and out.stopped == "abort: nothing left to do"
    assert (tmp_path / "escalation.md").exists()


def test_limits_stop_the_supervisor(tmp_path):
    always_fail = lambda goal: {"success": False}  # noqa: E731
    # decisions
    client = fake_client(*[{"action": "retry", "goal": [], "reason": "transient?"}] * 3)
    out = supervise(
        always_fail,
        lambda r, g: situation(g, history=[]),
        "x",
        GOAL,
        client,
        Limits(max_decisions=1),
    )
    assert out.decisions[-1]["action"] == "escalate" and "decision limit" in out.stopped
    # tokens
    client = fake_client({"action": "retry", "goal": [], "reason": "?"}, tokens=500)
    client.records.append(CallRecord("goal", "openai:m", 1.0, 400, 200))
    out = supervise(
        always_fail, situation_of, "x", GOAL, client, Limits(max_tokens=100)
    )
    assert "token limit" in out.stopped
    # time
    ticks = iter([0.0, 100.0, 200.0])
    out = supervise(
        always_fail,
        situation_of,
        "x",
        GOAL,
        fake_client(),
        Limits(max_seconds=10),
        clock=lambda: next(ticks),
    )
    assert "time limit" in out.stopped
    # no model
    out = supervise(always_fail, situation_of, "x", GOAL, None, report_dir=tmp_path)
    assert "no model" in out.stopped and (tmp_path / "escalation.json").exists()


def test_success_needs_no_decision():
    out = supervise(lambda g: {"success": True}, situation_of, "x", GOAL, None)
    assert out.success and out.decisions == []
