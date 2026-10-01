"""The step watchdog (#109): when a step plans too long, a rule or a model
decides to wait, skip its search or abort it, within a hard limit."""

import json
from types import SimpleNamespace

import pytest

from long_tamp.execution import ExecutionControl
from long_tamp.execution.watchdog import (
    StepWatchdog,
    WatchDecision,
    check_decision,
    model_decider,
)
from long_tamp.tasks.task_planning.events import make_event


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def _dog(**kw):
    clock, control, events = Clock(), ExecutionControl(), []
    dog = StepWatchdog(
        control, soft=60, hard=180, clock=clock, sink=events.append, **kw
    )
    return dog, clock, control, events


def _planning(dog, step="b06", search=None):
    dog.observe(make_event(step, "attempts", f"{step} retry", "RUNNING"))
    metrics = {"search": search} if search else {}
    dog.observe(
        make_event(
            step, "progress", step, "RUNNING", message="working", metrics=metrics
        )
    )


def test_nothing_happens_before_the_soft_limit_or_without_a_planning_step():
    dog, clock, control, events = _dog()
    clock.t = 500
    assert dog.tick() is None  # nothing planning
    _planning(dog)
    clock.t = 559
    assert dog.tick() is None and not events


def test_the_rule_skips_a_search_then_the_hard_limit_aborts():
    dog, clock, control, events = _dog()
    _planning(dog, search="a clamp pose")
    clock.t = 61
    decision = dog.tick()
    assert (decision.action, decision.by) == ("skip", "rule")
    assert control.pending == ("skip", "watchdog (rule)")
    assert events[-1]["role"] == "watchdog" and events[-1]["ir_id"] == "b06"
    assert events[-1]["metrics"]["action"] == "skip"
    control.take_request("search")  # the step took it, and kept planning
    clock.t = 120
    assert dog.tick() is None  # next look: at the hard limit
    clock.t = 181
    decision = dog.tick()
    assert (decision.action, decision.by) == ("abort_step", "rule")
    assert control.pending[0] == "abort_step"


def test_the_rule_waits_outside_a_search_never_past_the_hard_limit():
    dog, clock, control, events = _dog(wait_s=100)
    _planning(dog)
    clock.t = 100
    decision = dog.tick()
    assert decision.action == "wait" and control.pending is None
    assert dog.decisions[-1]["wait_s"] == 80  # capped: 180 - 100
    clock.t = 179
    assert dog.tick() is None
    clock.t = 180
    assert dog.tick().action == "abort_step"


def test_a_step_that_finishes_planning_is_no_longer_watched():
    dog, clock, control, events = _dog()
    _planning(dog)
    dog.observe(make_event("b06", "execute", "b06", "SUCCESS"))
    clock.t = 1000
    assert dog.tick() is None


def test_a_retried_attempt_is_watched_afresh():
    dog, clock, control, events = _dog()
    _planning(dog)
    clock.t = 50
    dog.observe(make_event("b06", "execute", "b06", "FAILURE"))  # attempt 2 starts
    clock.t = 100
    assert dog.tick() is None  # 50 s into attempt 2
    clock.t = 111
    assert dog.tick() is not None


def test_an_operators_request_or_a_pause_comes_first():
    dog, clock, control, events = _dog()
    _planning(dog, search="x")
    control.request("abort_step")
    clock.t = 100
    assert dog.tick() is None and control.pending == ("abort_step", "operator")
    control.clear_request()
    control.pause()
    assert dog.tick() is None
    control.resume()
    assert dog.tick().action == "skip"


def test_a_model_decision_is_checked_and_the_rule_replaces_a_bad_one():
    asked = []

    def decide(watch, elapsed, remaining):
        asked.append((watch.search, round(elapsed), round(remaining)))
        return WatchDecision("skip", "searching too long", by="model")

    dog, clock, control, events = _dog(decide=decide)
    _planning(dog)  # not in a search: a skip can't apply
    clock.t = 70
    decision = dog.tick()
    assert asked == [(None, 70, 110)]
    assert (decision.action, decision.by) == ("wait", "rule")

    def broken(watch, elapsed, remaining):
        raise RuntimeError("gateway down")

    dog, clock, control, events = _dog(decide=broken)
    _planning(dog, search="x")
    clock.t = 70
    assert dog.tick().by == "rule"


def test_check_decision():
    watch = SimpleNamespace(search=None)
    assert check_decision(WatchDecision("wait", "ok", 60), watch, 100) == []
    assert "at most" in check_decision(WatchDecision("wait", "ok", 600), watch, 100)[0]
    assert check_decision(WatchDecision("dance", "x"), watch, 100)
    assert check_decision(WatchDecision("abort_step", ""), watch, 100) == [
        "give a reason"
    ]


def test_the_model_role_through_the_gateway():
    from tests.test_ai_chat import scripted_client

    client, prompts = scripted_client(
        {"action": "wait", "wait_minutes": 99, "reason": "phases advance"},  # too long
        {"action": "wait", "wait_minutes": 1.5, "reason": "phases advance"},
    )
    dog, clock, control, events = _dog(decide=model_decider(client))
    _planning(dog)
    dog.observe(make_event("b06", "progress", "b06", "RUNNING", message="phase 3/5"))
    clock.t = 90
    decision = dog.tick()
    assert (decision.action, decision.by, decision.wait_s) == ("wait", "model", 90.0)
    assert "phase 3/5" in prompts[0] and "at most 1.5 more minutes" in prompts[1]
    assert json.loads(json.dumps(dog.decisions))[-1]["wait_s"] == 90.0


def test_soft_and_hard_must_make_sense():
    with pytest.raises(ValueError):
        StepWatchdog(ExecutionControl(), soft=100, hard=50)
