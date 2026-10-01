"""The operator chat (#89): a model acts only through gated tools. No API
calls: the model is scripted through a fake gateway client."""

import json
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

from long_tamp.ai import AIAuthError, make_client
from long_tamp.ai.chat import ChatSession, Tool, ToolRejected

SCREW_DIR = Path(__file__).resolve().parents[1] / "script" / "screw_assembly"
if str(SCREW_DIR) not in sys.path:
    sys.path.insert(0, str(SCREW_DIR))


def scripted_client(*answers):
    """A gateway client whose model replays ``answers`` (dicts, or
    exceptions to raise); records each prompt."""
    queue, prompts = list(answers), []

    def create(**kwargs):
        prompts.append(kwargs["messages"][1]["content"])
        answer = queue.pop(0)
        if isinstance(answer, Exception):
            raise answer
        msg = SimpleNamespace(content=json.dumps(answer), refusal=None)
        return SimpleNamespace(
            choices=[SimpleNamespace(message=msg, finish_reason="stop")], usage=None
        )

    sdk = SimpleNamespace(
        chat=SimpleNamespace(completions=SimpleNamespace(create=create))
    )
    return make_client("openai:m", sdk=sdk, base_url="http://gw"), prompts


def act(say, *actions, done=True):
    return {
        "say": say,
        "actions": [{"tool": t, "arguments": a} for t, a in actions],
        "done": done,
    }


# -- the engine --------------------------------------------------------------------


def counter_tools(log):
    def add(n):
        if not isinstance(n, int) or n < 0:
            raise ToolRejected("n must be a non-negative integer")
        log.append(n)
        return {"total": sum(log)}

    return [Tool("add", "add n to the total", add, {"n": "integer"})]


def test_tools_run_and_results_go_back_to_the_model():
    log, calls = [], []
    client, prompts = scripted_client(
        act("adding", ("add", {"n": 2}), done=False), act("the total is 2")
    )
    session = ChatSession(client, counter_tools(log), on_tool=calls.append)
    turn = session.turn("add two")
    assert turn.say == "the total is 2" and log == [2]
    assert [c.ok for c in turn.calls] == [True] and calls[0].tool == "add"
    assert 'add({"n": 2}) -> {"total": 2}' in prompts[1]


def test_rejected_unknown_and_malformed_calls_are_reported_not_run():
    log = []
    client, prompts = scripted_client(
        act(
            "trying",
            ("add", {"n": -1}),
            ("fly", {}),
            ("add", {"m": 1}),
            done=False,
        ),
        act("I can't add a negative number"),
    )
    turn = ChatSession(client, counter_tools(log)).turn("add minus one")
    assert log == [] and [c.ok for c in turn.calls] == [False, False, False]
    assert "REJECTED: n must be a non-negative integer" in prompts[1]
    assert "no tool 'fly'" in prompts[1] and "bad arguments" in prompts[1]


def test_history_is_kept_and_api_errors_end_the_turn():
    client, prompts = scripted_client(act("hi"), AIAuthError("bad key"))
    session = ChatSession(client, counter_tools([]))
    session.turn("hello")
    turn = session.turn("again")
    assert "Operator: hello" in prompts[1] and "You: hi" in prompts[1]
    assert turn.error.startswith("AIAuthError")


def test_the_model_steps_are_bounded():
    client, _ = scripted_client(*[act("more", ("add", {"n": 1}), done=False)] * 3)
    turn = ChatSession(client, counter_tools([]), max_steps=3).turn("keep adding")
    assert len(turn.calls) == 3


# -- the mission tools: the acceptance conversation ---------------------------------


def stub_mission_module(world):
    """What MissionChat needs from task_screw_assembly, without HPP: the
    real domain and task planner, a fixed world and a run that fails."""
    import screw_domain as D

    T = ModuleType("T")
    T.clamp_seats = lambda pairs: [
        (f"fixtures/clamp{i}", f"part{i}/h_seat") for i in (1, 2)
    ]
    T.descriptors = D.descriptors
    T.world_atoms = lambda planner, recorded: list(world)
    T.goal_vocabulary = D.goal_vocabulary
    T.pddl_problem = D.pddl_problem
    T.planned_document = D.planned_document
    T.runs = []

    def run_mission(task, planner, n_parts, q_start=None, document=None, **kw):
        T.runs.append(document)
        return {
            "success": False,
            "seconds": 1.0,
            "final_config": q_start,
            "failure": {
                "step": "part2 A: clamp + screw",
                "capability": "clamp_and_screw",
                "parameters": {"part": "part2", "clamp": "fixtures/clamp2"},
                "facts": ["cannot_reach(fixtures/clamp2, part2/h_seat)"],
            },
        }

    T.run_mission = run_mission
    return T


GOAL = [
    "screwed(part1, part1/h_hole1)",
    "screwed(part1, part1/h_hole2)",
    "screwed(part2, part2/h_hole1)",
    "screwed(part2, part2/h_hole2)",
]


def test_a_conversation_changes_the_plan_only_through_the_tools():
    pytest.importorskip("unified_planning")
    from chat_tools import INTRO, MissionChat

    T = stub_mission_module(world=[])
    task = SimpleNamespace(task_config=SimpleNamespace(VALID_PAIRS={}))
    work = MissionChat(T, task, planner=None, recorded=None, n_parts=2, mission={})
    first = ["screwed(part2, part2/h_hole1)", "screwed(part2, part2/h_hole2)"]
    client, prompts = scripted_client(
        # "assemble parts 1 and 2": set the goal, plan
        act(
            "setting the goal",
            ("set_goal", {"literals": GOAL}),
            ("plan", {}),
            done=False,
        ),
        act("planned both parts"),
        # "do part 2 first": a goal literal that isn't in the goal is refused...
        act(
            "part 2 first",
            ("plan", {"first": ["screwed(part3, part3/h_hole1)"]}),
            done=False,
        ),
        # ...then the planner orders it
        act("again", ("plan", {"first": first}), done=False),
        act("part 2 comes first now"),
        # "run it", then "why did that step fail?"
        act("running", ("run", {}), done=False),
        act("it failed"),
        act("explaining", ("explain_failure", {}), done=False),
        act("clamp 2 can't reach part 2's seat"),
    )
    session = ChatSession(client, work.tools(), INTRO)

    session.turn("assemble parts 1 and 2")
    assert work.goal == GOAL
    default_order = [s[1].get("part") for s in work.steps if s[0] == "clamp_and_screw"]

    turn = session.turn("do part 2 first")
    assert not turn.calls[0].ok and "not in it" in turn.calls[0].error
    assert turn.calls[1].ok
    order = [s[1].get("part") for s in work.steps if s[0] == "clamp_and_screw"]
    assert order == ["part2", "part1"]
    assert set(order) == set(default_order)  # same work, reordered by the planner

    session.turn("run it")
    assert len(T.runs) == 1 and work.document is None  # a run consumes the plan

    turn = session.turn("why did that step fail?")
    explained = turn.calls[0].result
    assert explained["facts"] == ["cannot_reach(fixtures/clamp2, part2/h_seat)"]
    assert turn.say.startswith("clamp 2")


def test_mission_tools_refuse_what_their_checks_refuse():
    from chat_tools import MissionChat

    T = stub_mission_module(world=[])
    task = SimpleNamespace(task_config=SimpleNamespace(VALID_PAIRS={}))
    work = MissionChat(T, task, planner=None, recorded=None, n_parts=2, mission={})
    with pytest.raises(ToolRejected, match="unknown object"):
        work.set_goal(["screwed(part9, part9/h_hole1)"])
    with pytest.raises(ToolRejected, match="set a goal first"):
        work.plan()
    with pytest.raises(ToolRejected, match="no plan to run"):
        work.run()
    with pytest.raises(ToolRejected, match="unknown capability"):
        work.add_constraint("weld", {"x": "part1"})
    with pytest.raises(ToolRejected, match="nothing has run"):
        work.explain_failure()
