"""Typed model roles (#87): the propose/check/refine loop, the grounder and
the plan reviewer. No API calls: proposers are scripted, clients faked."""

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from long_tamp.ai import AIAuthError, AIOutputError, make_client
from long_tamp.ai.roles import Rejected, RoleError, Unsupported, refine
from long_tamp.tasks.task_planning.language import (
    Grounding,
    ground_instruction,
    lexical_grounding,
    with_grounding,
)
from long_tamp.tasks.task_planning.review import (
    ReviewRequest,
    check_constraints,
    review_constraints,
)

SCREW_DIR = Path(__file__).resolve().parents[1] / "script" / "screw_assembly"
if str(SCREW_DIR) not in sys.path:
    sys.path.insert(0, str(SCREW_DIR))
import screw_domain  # noqa: E402

# -- the loop ----------------------------------------------------------------


def scripted(*answers):
    """A proposer replaying ``answers`` (values or exceptions), recording
    the feedback it was given."""
    seen = []
    queue = list(answers)

    def propose(feedback):
        seen.append(list(feedback))
        answer = queue.pop(0)
        if isinstance(answer, Exception):
            raise answer
        return answer

    return propose, seen


def even(x):
    return [] if x % 2 == 0 else [f"{x} is odd"]


def test_rejections_go_back_as_feedback_until_accepted():
    propose, seen = scripted(3, 4)
    outcome = refine(propose, even)
    assert outcome.value == 4 and outcome.fallback is None
    assert seen[0] == [] and "3 is odd" in seen[1]
    assert len(outcome.attempts) == 2


def test_unparseable_answers_are_fed_back_too():
    propose, seen = scripted(Rejected("not a number"), 2)
    assert refine(propose, even).value == 2
    assert "not a number" in seen[1]


def test_the_fallback_covers_unsupported_api_errors_and_exhausted_rounds():
    for answers in ([Unsupported("can't")], [AIAuthError("bad key")], [1, 3, 5]):
        propose, _ = scripted(*answers)
        outcome = refine(propose, even, max_rounds=3, fallback=lambda why: 0)
        assert outcome.value == 0 and outcome.fallback


def test_without_a_fallback_the_role_fails():
    propose, _ = scripted(1, 3)
    with pytest.raises(RoleError, match="no acceptable answer in 2 attempts") as caught:
        refine(propose, even, max_rounds=2, name="parity")
    assert len(caught.value.attempts) == 2


# -- fakes over the gateway ----------------------------------------------------


def fake_client(*answers):
    """A gateway client whose OpenAI-compatible SDK replays JSON answers."""
    queue = list(answers)
    requests = []

    def create(**kwargs):
        requests.append(kwargs)
        msg = SimpleNamespace(content=json.dumps(queue.pop(0)), refusal=None)
        return SimpleNamespace(
            choices=[SimpleNamespace(message=msg, finish_reason="stop")], usage=None
        )

    sdk = SimpleNamespace(
        chat=SimpleNamespace(completions=SimpleNamespace(create=create))
    )
    return make_client("openai:m", sdk=sdk, base_url="http://gw"), requests


def _vocabulary():
    return screw_domain.goal_vocabulary(2, clamps=screw_domain_clamps(3, 2))


def screw_domain_clamps(n_clamps, n_parts):
    """Every clamp takes every part: spare clamps give the planner choices."""
    return [
        (f"fixtures/clamp{c}", f"part{p}/h_seat")
        for c in range(1, n_clamps + 1)
        for p in range(1, n_parts + 1)
    ]


# -- grounder ------------------------------------------------------------------


def test_the_lexical_grounder_matches_names_spaces_and_case_aside():
    v = _vocabulary()
    g = lexical_grounding("Assemble Part 2, not in clamp 1; use the screwdriver", v)
    assert set(g.objects) == {
        "part2",
        "fixtures/clamp1",
        "driver/h_grip",  # the driver is listed only through its handles
        "driver/h_rack",
    }


def test_the_model_grounder_is_checked_and_falls_back():
    v = _vocabulary()
    client, _ = fake_client(
        {"objects": ["part2", "part9"], "unmatched": []},
        {"objects": ["part2"], "unmatched": ["the blue one"]},
    )
    outcome = ground_instruction("assemble part 2 and the blue one", v, client)
    assert outcome.value == Grounding(("part2",), ("the blue one",))
    assert outcome.fallback is None and len(outcome.attempts) == 2
    # no model: the lexical fallback
    outcome = ground_instruction("assemble part 2", v, None)
    assert outcome.value.objects == ("part2",) and outcome.fallback == "no model"


def test_grounding_becomes_a_note_for_the_goal_writer():
    v = with_grounding(_vocabulary(), Grounding(("part2",), ("the blue one",)))
    assert "refers to: part2" in v.notes and "the blue one" in v.notes


# -- plan reviewer -------------------------------------------------------------


def _request(instruction="assemble both parts but don't use clamp 1"):
    v = _vocabulary()
    return ReviewRequest(instruction, v, screw_domain.descriptors(2))


def test_constraints_are_checked_against_the_capabilities_and_objects():
    req = _request()
    assert (
        check_constraints([("clamp_and_screw", {"clamp": "fixtures/clamp1"})], req)
        == []
    )
    errors = check_constraints(
        [
            ("weld", {"x": "part1"}),
            ("clamp_and_screw", {"vise": "fixtures/clamp1"}),
            ("clamp_and_screw", {"clamp": "fixtures/clamp9"}),
            ("clamp_and_screw", {}),
            ("clamp_and_screw", {"block": "b03"}),
        ],
        req,
    )
    text = " ".join(errors)
    for problem in (
        "unknown capability",
        "no parameter 'vise'",
        "clamp9",
        "at least one",
        "no parameter 'block'",
    ):
        assert problem in text


def test_a_constraint_that_makes_the_mission_impossible_is_rejected():
    req = _request()
    errors = check_constraints(
        [("clamp_and_screw", {"clamp": "fixtures/clamp1"})],
        req,
        replannable=lambda blocked: "no plan",
    )
    assert "no plan reaches the goal" in errors[0]


def test_the_reviewer_turns_an_instruction_into_blocked_bindings():
    client, requests = fake_client(
        {
            "avoid": [
                {
                    "capability": "clamp_and_screw",
                    "parameters": [{"name": "clamp", "value": "fixtures/clamp1"}],
                }
            ],
            "reason": "the operator rules out clamp 1",
        }
    )
    outcome = review_constraints(_request(), client, replannable=lambda b: None)
    assert outcome.value == [("clamp_and_screw", {"clamp": "fixtures/clamp1"})]
    prompt = requests[0]["messages"][1]["content"]
    assert (
        "clamp_and_screw(part, clamp, seat" in prompt
        and "never write steps" in requests[0]["messages"][0]["content"]
    )


def test_without_a_model_or_an_acceptable_answer_there_is_no_constraint():
    assert review_constraints(_request(), None).value == []
    bad = {
        "avoid": [{"capability": "weld", "parameters": [{"name": "x", "value": "y"}]}],
        "reason": "",
    }
    client, _ = fake_client(bad, bad)
    outcome = review_constraints(_request(), client)
    assert outcome.value == [] and "no acceptable answer" in outcome.fallback


def test_a_reviewed_constraint_changes_the_plan_only_through_the_planner():
    """A constraint like 'don't use clamp N' is a blocked binding: the task
    planner plans around it (the acceptance case of #87). Whichever clamp
    the free plan gives part 1 is the one ruled out."""
    pytest.importorskip("unified_planning")
    from long_tamp.tasks.task_planning.skeleton import UnifiedPlanningPlanner

    planner = UnifiedPlanningPlanner()
    clamps = screw_domain_clamps(3, 2)

    def clamp_of(steps, part):
        (clamp,) = [
            p["clamp"] for c, p in steps if c == "clamp_and_screw" and p["part"] == part
        ]
        return clamp

    free = planner.solve(screw_domain.pddl_problem(2, clamps=clamps))
    ruled_out = clamp_of(free, "part1")
    blocked = [("clamp_and_screw", {"clamp": ruled_out})]
    constrained = planner.solve(
        screw_domain.pddl_problem(2, clamps=clamps, blocked=blocked)
    )
    assert ruled_out not in {
        p["clamp"] for c, p in constrained if c == "clamp_and_screw"
    }
    assert {clamp_of(constrained, "part1"), clamp_of(constrained, "part2")} <= {
        c for c, _ in clamps
    }


@pytest.mark.parametrize(
    "item",
    [
        {
            "capability": "clamp_and_screw",
            "parameters": [{"name": "clamp", "value": "fixtures/clamp1"}],
        },
        {"capability": "clamp_and_screw", "parameters": {"clamp": "fixtures/clamp1"}},
        {"capability": "clamp_and_screw", "binding": {"clamp": "fixtures/clamp1"}},
        {"capability": "clamp_and_screw", "clamp": "fixtures/clamp1"},  # flattened
    ],
)
def test_constraints_are_read_in_the_shapes_models_write(item):
    from long_tamp.tasks.task_planning.review import _parse

    assert _parse({"avoid": [item]}) == [
        ("clamp_and_screw", {"clamp": "fixtures/clamp1"})
    ]


def test_a_shapeless_answer_is_sent_back_with_an_example():
    from long_tamp.tasks.task_planning.review import _parse

    with pytest.raises(Rejected, match='"avoid"'):
        _parse({"constraints": "none"})


def test_an_unusable_answer_costs_a_round_not_the_role():
    propose, seen = scripted(AIOutputError("the answer is not JSON: 'Objects: ...'"), 2)
    outcome = refine(propose, even, fallback=lambda why: 0)
    assert outcome.value == 2 and outcome.fallback is None
    assert any("JSON object only" in line for line in seen[1])
