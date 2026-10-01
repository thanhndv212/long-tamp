"""Goals from natural language (#22): the model writes the goal, checked
before use. No API calls: the writers here are fakes."""

import json
from types import SimpleNamespace

import pytest

from long_tamp.tasks.task_planning import CapabilityDescriptor
from long_tamp.tasks.task_planning.language import (
    ClaudeGoalWriter,
    GoalError,
    Vocabulary,
    check_goal,
    goal_from_instruction,
)

GRASP = CapabilityDescriptor(
    "grasp",
    "1.0",
    {"gripper": str, "handle": str},
    preconditions=("not holds(?gripper, _)",),
    effects=("holds(?gripper, ?handle)",),
)
SCREW = CapabilityDescriptor(
    "screw",
    "1.0",
    {"part": str, "hole": str},
    effects=("screwed(?part, ?hole)",),
)


def _vocabulary():
    return Vocabulary.from_domain(
        [GRASP, SCREW],
        state=["holds(left, part1/h)"],
        objects=["part1", "part1/hole", "part2", "part2/hole", "right"],
        notes="screwed(part, hole): the hole's screw is driven.",
    )


def test_the_vocabulary_comes_from_the_domain_and_state():
    v = _vocabulary()
    assert v.predicates == {"holds": 2, "screwed": 2}
    assert "left" in v.objects and "part1/h" in v.objects  # from the state
    assert v.state == ("holds(left, part1/h)",)


@pytest.mark.parametrize(
    "goal, problem",
    [
        ([], "empty"),
        (["screwed(part3, part3/hole)"], "unknown object 'part3'"),
        (["glued(part1, part2)"], "unknown predicate"),
        (["screwed(part1)"], "takes 2 arguments"),
        (["holds(left, _)"], "only allowed in negated"),
        (["screwed(?p, part1/hole)"], "variables"),
        (["screwed(part1, part1/hole"], "not a literal"),
    ],
)
def test_bad_goals_are_explained(goal, problem):
    errors = check_goal(goal, _vocabulary())
    assert errors and problem in " ".join(errors), errors


def test_a_good_goal_passes():
    goal = ["screwed(part2, part2/hole)", "not holds(left, _)"]
    assert check_goal(goal, _vocabulary()) == []


class Scripted:
    """Returns the scripted goals in turn, recording the feedback it got."""

    def __init__(self, *goals):
        self.goals, self.feedback = list(goals), []

    def write_goal(self, instruction, vocabulary, feedback):
        self.feedback.append(list(feedback))
        return self.goals.pop(0)


def test_rejected_goals_go_back_to_the_writer_with_the_reasons():
    writer = Scripted(["screwed(part3, part3/hole)"], ["screwed(part2, part2/hole)"])
    goal = goal_from_instruction("screw part 2", writer, _vocabulary())
    assert goal == ["screwed(part2, part2/hole)"]
    assert writer.feedback[0] == []
    assert any("unknown object 'part3'" in line for line in writer.feedback[1])


def test_an_unreachable_goal_is_rejected_by_the_planner_check():
    writer = Scripted(["screwed(part1, part1/hole)"], ["screwed(part2, part2/hole)"])
    reachable = lambda goal: "part1 is jammed" if "part1" in goal[0] else None
    goal = goal_from_instruction("screw a part", writer, _vocabulary(), reachable)
    assert goal == ["screwed(part2, part2/hole)"]
    assert any("part1 is jammed" in line for line in writer.feedback[1])


def test_attempts_run_out():
    writer = Scripted(*[["glued(a, b)"]] * 3)
    with pytest.raises(GoalError) as caught:
        goal_from_instruction("glue them", writer, _vocabulary(), max_rounds=3)
    assert len(caught.value.attempts) == 3


# -- ClaudeGoalWriter, against a fake client -------------------------------


class FakeClient:
    """Stands in for anthropic.Anthropic(): records requests, replays answers."""

    def __init__(self, *responses):
        self.responses, self.requests = list(responses), []
        self.beta = SimpleNamespace(messages=SimpleNamespace(create=self._create))

    def _create(self, **kwargs):
        self.requests.append(kwargs)
        return self.responses.pop(0)


def _answer(goal, unsupported="", stop_reason="end_turn"):
    text = json.dumps({"goal": goal, "unsupported": unsupported})
    return SimpleNamespace(
        stop_reason=stop_reason,
        stop_details=None,
        content=[SimpleNamespace(type="text", text=text)],
    )


def test_claude_writes_a_structured_goal():
    client = FakeClient(_answer(["screwed(part2, part2/hole)"]))
    writer = ClaudeGoalWriter(client=client)
    goal = goal_from_instruction("screw part 2", writer, _vocabulary())
    assert goal == ["screwed(part2, part2/hole)"]
    (request,) = client.requests
    assert request["model"] == "claude-opus-5-5"
    assert request["output_config"]["format"]["type"] == "json_schema"
    assert request["fallbacks"] == "default"
    prompt = request["messages"][0]["content"]
    assert "screwed/2" in prompt and "part2/hole" in prompt
    assert "holds(left, part1/h)" in prompt and "Instruction: screw part 2" in prompt
    assert "never write the plan" in request["system"]


def test_feedback_reaches_the_next_prompt():
    client = FakeClient(
        _answer(["glued(a, b)"]), _answer(["screwed(part2, part2/hole)"])
    )
    goal_from_instruction(
        "screw part 2", ClaudeGoalWriter(client=client), _vocabulary()
    )
    assert "unknown predicate 'glued'" in client.requests[1]["messages"][0]["content"]


def test_an_instruction_the_domain_cant_express_is_reported():
    client = FakeClient(_answer([], unsupported="no painting predicate"))
    with pytest.raises(GoalError, match="can't be expressed: no painting"):
        goal_from_instruction(
            "paint part 1", ClaudeGoalWriter(client=client), _vocabulary()
        )


def test_a_refusal_is_an_error_not_a_goal():
    client = FakeClient(_answer([], stop_reason="refusal"))
    with pytest.raises(GoalError, match="declined"):
        goal_from_instruction("...", ClaudeGoalWriter(client=client), _vocabulary())


def test_the_screw_domain_vocabulary_and_a_written_goal_plan():
    """The screw assembly's vocabulary, and a goal for 'assemble part 2'
    that the PDDL export accepts in place of the mission goal."""
    import sys
    from pathlib import Path

    sys.path.insert(
        0, str(Path(__file__).resolve().parents[1] / "script/screw_assembly")
    )
    import screw_domain

    v = screw_domain.goal_vocabulary(2)
    assert v.predicates == {"holds": 2, "screwed": 2}
    assert {"part2/h_hole1", "fixtures/rack_hold", "driver/h_rack"} <= set(v.objects)
    assert "carries the parts" in v.notes
    goal = [
        "screwed(part2, part2/h_hole1)",
        "screwed(part2, part2/h_hole2)",
        "not holds(ur10_left/gripper, _)",
        "holds(fixtures/rack_hold, driver/h_rack)",
    ]
    assert check_goal(goal, v) == []
    export = screw_domain.pddl_problem(2, goal=goal)
    assert "part1" not in export.problem.split(":goal")[1]  # only part 2 asked for


def test_an_api_error_fails_with_a_clear_message():
    class APIStatusError(Exception):  # stands in for the SDK's error class
        __module__ = "anthropic._exceptions"
        message = "Your credit balance is too low"

    client = FakeClient()
    client.beta.messages.create = lambda **kwargs: (_ for _ in ()).throw(
        APIStatusError()
    )
    with pytest.raises(GoalError, match="Anthropic API error: Your credit balance"):
        goal_from_instruction("...", ClaudeGoalWriter(client=client), _vocabulary())


# -- OpenAIGoalWriter, against a fake client --------------------------------


class FakeOpenAI:
    """Stands in for openai.OpenAI(): records requests, replays answers;
    an exception in the answers is raised instead."""

    def __init__(self, *responses):
        self.responses, self.requests = list(responses), []
        completions = SimpleNamespace(create=self._create)
        self.chat = SimpleNamespace(completions=completions)

    def _create(self, **kwargs):
        self.requests.append(kwargs)
        answer = self.responses.pop(0)
        if isinstance(answer, Exception):
            raise answer
        return answer


def _completion(goal, unsupported="", finish_reason="stop", refusal=None):
    text = json.dumps({"goal": goal, "unsupported": unsupported})
    message = SimpleNamespace(content=text, refusal=refusal)
    return SimpleNamespace(
        choices=[SimpleNamespace(message=message, finish_reason=finish_reason)]
    )


def test_an_openai_compatible_model_writes_a_goal():
    from long_tamp.tasks.task_planning.language import OpenAIGoalWriter

    client = FakeOpenAI(_completion(["screwed(part2, part2/hole)"]))
    writer = OpenAIGoalWriter(model="some-model", client=client)
    goal = goal_from_instruction("screw part 2", writer, _vocabulary())
    assert goal == ["screwed(part2, part2/hole)"]
    (request,) = client.requests
    assert request["model"] == "some-model"
    assert request["response_format"]["type"] == "json_schema"
    assert request["messages"][0]["role"] == "system"
    assert "Instruction: screw part 2" in request["messages"][1]["content"]


def test_an_endpoint_without_json_schema_falls_back_to_json_mode():
    from long_tamp.tasks.task_planning.language import OpenAIGoalWriter

    class BadRequestError(Exception):  # stands in for the SDK's error class
        __module__ = "openai._exceptions"
        status_code = 400

    client = FakeOpenAI(BadRequestError(), _completion(["screwed(part2, part2/hole)"]))
    writer = OpenAIGoalWriter(model="m", client=client)
    assert goal_from_instruction("screw part 2", writer, _vocabulary())
    assert client.requests[1]["response_format"] == {"type": "json_object"}
    assert '"goal"' in client.requests[1]["messages"][0]["content"]


def test_an_openai_refusal_or_bad_json_is_an_error():
    from long_tamp.tasks.task_planning.language import OpenAIGoalWriter

    client = FakeOpenAI(_completion([], refusal="no"))
    with pytest.raises(GoalError, match="declined"):
        goal_from_instruction(
            "...", OpenAIGoalWriter("m", client=client), _vocabulary()
        )
    broken = _completion([])
    broken.choices[0].message.content = "not json"
    with pytest.raises(GoalError, match="not JSON"):
        goal_from_instruction(
            "...", OpenAIGoalWriter("m", client=FakeOpenAI(broken)), _vocabulary()
        )
