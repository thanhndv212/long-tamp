"""World-state providers: observed predicates and recorded facts (issue #3, ADR-0002)."""

import json
from types import SimpleNamespace

from long_tamp.tasks.task_planning import (
    CapabilityDescriptor,
    CapabilityRegistry,
    TaskPlan,
    TaskPlanningSession,
)
from long_tamp.tasks.task_planning.predicates import Literal, parse_atom
from long_tamp.tasks.task_planning.world_state import (
    CompositeWorldState,
    GraspTrackerState,
    RecordedFacts,
)


def _atoms(*texts):
    return frozenset(parse_atom(t) for t in texts)


# ------------------------------------------------------------------- observed


def test_grasp_tracker_state_reports_what_each_gripper_holds():
    tracker = SimpleNamespace(
        current_grasps={"left/gripper": "part1/h_grasp", "right/gripper": None}
    )
    state = GraspTrackerState(tracker)
    assert state() == _atoms("holds(left/gripper, part1/h_grasp)")
    tracker.current_grasps["right/gripper"] = "driver/h_grip"  # read live
    assert parse_atom("holds(right/gripper, driver/h_grip)") in state()


def test_grasp_tracker_predicate_name_is_configurable():
    tracker = SimpleNamespace(current_grasps={"g": "h"})
    assert GraspTrackerState(tracker, predicate="grasped")() == _atoms("grasped(g, h)")


# ------------------------------------------------------------------- recorded


def test_recorded_facts_apply_only_their_own_predicates(tmp_path):
    facts = RecordedFacts(tmp_path / "facts.json", predicates={"screwed"})
    applied = facts.apply(
        [
            Literal.parse("screwed(part1, hole1)"),
            Literal.parse("holds(left, part1)"),  # observed, not recorded
        ]
    )
    assert [str(lit) for lit in applied] == ["screwed(part1, hole1)"]
    assert facts() == _atoms("screwed(part1, hole1)")


def test_recorded_facts_delete_with_wildcards(tmp_path):
    facts = RecordedFacts(tmp_path / "facts.json", predicates={"racked"})
    facts.apply([Literal.parse("racked(driver, rack1)")])
    facts.apply([Literal.parse("not racked(driver, _)")])
    assert facts() == frozenset()


def test_recorded_facts_survive_a_restart_and_are_written_atomically(tmp_path):
    path = tmp_path / "facts.json"
    RecordedFacts(path, predicates={"screwed"}).apply(
        [Literal.parse("screwed(part1, hole1)")]
    )
    assert not list(tmp_path.glob("*.tmp"))
    assert json.loads(path.read_text())["facts"] == ["screwed(part1, hole1)"]
    reloaded = RecordedFacts(path, predicates={"screwed"})
    assert reloaded() == _atoms("screwed(part1, hole1)")


def test_recorded_facts_can_live_in_memory_only():
    facts = RecordedFacts(None, predicates={"screwed"})
    facts.apply([Literal.parse("screwed(p, h)")])
    assert facts() == _atoms("screwed(p, h)")


def test_composite_state_merges_sources():
    tracker = SimpleNamespace(current_grasps={"g": "h"})
    recorded = RecordedFacts(None, predicates={"screwed"})
    recorded.apply([Literal.parse("screwed(p, h1)")])
    world = CompositeWorldState(
        GraspTrackerState(tracker), recorded, lambda: ["clamped(p, c1)"]
    )
    assert world() == _atoms("holds(g, h)", "screwed(p, h1)", "clamped(p, c1)")


# ---------------------------------------------------------------- the session


def _session(recorded, implementation=lambda parameters: {}, world_state=None):
    registry = CapabilityRegistry()
    registry.register(
        CapabilityDescriptor(
            "screw",
            "1.0",
            {"part": str, "hole": str},
            preconditions=("not screwed(?part, ?hole)",),
            effects=("screwed(?part, ?hole)",),
            restartable=True,
        ),
        implementation,
    )
    document = {
        "schema_version": "1.0",
        "mission_id": "screw-demo",
        "scene": {"id": "fake"},
        "provenance": {"kind": "human", "generator": "test"},
        "initial_state": [],
        "root": {
            "type": "transaction",
            "id": "screw-1",
            "restart_state": ["q_current"],
            "children": [
                {
                    "type": "operation",
                    "id": "screw-1.execute",
                    "capability": "screw",
                    "parameters": {"part": "part1", "hole": "hole1"},
                }
            ],
        },
    }
    plan = TaskPlan.from_dict(document, registry)  # simulation runs here
    return TaskPlanningSession(
        plan, registry, world_state=world_state, recorded=recorded
    )


def test_planning_never_writes_a_recorded_fact(tmp_path):
    recorded = RecordedFacts(tmp_path / "facts.json", predicates={"screwed"})
    _session(recorded)
    assert recorded() == frozenset()
    assert not (tmp_path / "facts.json").exists()


def test_a_successful_step_records_its_effects():
    recorded = RecordedFacts(None, predicates={"screwed"})
    session = _session(recorded)
    assert json.loads(session.execute_step("screw-1"))["status"] == "success"
    assert recorded() == _atoms("screwed(part1, hole1)")


def test_a_failed_step_records_nothing():
    recorded = RecordedFacts(None, predicates={"screwed"})

    def fails(parameters):
        raise RuntimeError("cross-threaded")

    session = _session(recorded, implementation=fails)
    assert json.loads(session.execute_step("screw-1"))["status"] == "retry"
    assert recorded() == frozenset()


def test_recorded_facts_feed_the_precondition_check():
    recorded = RecordedFacts(None, predicates={"screwed"})
    session = _session(recorded, world_state=CompositeWorldState(recorded))
    assert json.loads(session.check_precondition("screw-1"))["ready"] is True
    session.execute_step("screw-1")
    result = json.loads(session.check_precondition("screw-1"))
    assert result["ready"] is False
    assert result["unsatisfied"] == ["not screwed(part1, hole1)"]
