"""The screw-assembly plan, run logically (issue #5): no HPP, no planning.

The real plan and runner (``run_node``) drive fake capabilities that update a
fake grasp tracker and the recorded facts the way the real blocks do, so the
guards and effect-based skips are checked in milliseconds.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

from long_tamp.tasks.task_planning import (
    CapabilityRegistry,
    CompositeWorldState,
    GraspTrackerState,
    RecordedFacts,
    TaskPlan,
    TaskPlanningSession,
)
from long_tamp.tasks.task_planning.predicates import Literal, holds
from long_tamp.tasks.task_planning.runner import run_plan

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script/screw_assembly"))
import screw_domain  # noqa: E402


class _Mission:
    """A fake cell: each block updates the grasps as the real one would."""

    def __init__(self, n_parts, held=None, facts=()):
        self.grasp_tracker = type("T", (), {"current_grasps": dict(held or {})})()
        self.recorded = RecordedFacts(None, screw_domain.RECORDED_PREDICATES)
        self.recorded.apply([Literal.parse(fact) for fact in facts])
        self.ran = []
        self.blocks = screw_domain.blocks_by_label(n_parts)
        world = CompositeWorldState(GraspTrackerState(self), self.recorded)
        registry = CapabilityRegistry()
        for name, descriptor in screw_domain.descriptors(n_parts).items():
            if name in ("part_done", "all_parts_done"):

                def check(parameters, descriptor=descriptor):
                    return all(
                        holds(lit.ground(parameters), world())
                        for lit in descriptor.precondition_literals
                    )

                registry.register(descriptor, check)
            else:
                registry.register(descriptor, self.run)
        plan = TaskPlan.from_dict(screw_domain.build_plan_document(n_parts), registry)
        self.session = TaskPlanningSession(
            plan, registry, world_state=world, recorded=self.recorded
        )

    def run(self, parameters):
        label = parameters["block"]
        self.ran.append(label)
        grasps = self.grasp_tracker.current_grasps
        for gripper, handle in self.blocks[label].get("seq", []):
            grasps[gripper] = handle
        return {}

    def go(self):
        run = run_plan(self.session)
        return run.success, run.skipped


LABELS = [b["label"] for b in screw_domain.build_mission(2)]


def test_nominal_run_executes_every_block_in_order():
    mission = _Mission(2)
    ok, skipped = mission.go()
    assert ok and skipped == []
    assert mission.ran == LABELS
    # Each completed clamp_and_screw block recorded its two holes.
    assert {str(a) for a in mission.recorded()} == {
        f"screwed(part{i}, part{i}/h_hole{h})" for i in (1, 2) for h in (1, 2)
    }


def test_resume_after_part1_released_skips_the_whole_part():
    held = {
        "ur10_right/gripper": "driver/h_grip",
        "fixtures/clamp1": "part1/h_seat",
    }
    facts = ["screwed(part1, part1/h_hole1)", "screwed(part1, part1/h_hole2)"]
    mission = _Mission(2, held, facts)
    ok, skipped = mission.go()
    assert ok
    assert "part1 assembled" in skipped
    assert not any(label.startswith("part1") for label in mission.ran)
    assert "bootstrap: pick driver" in skipped  # driver already in hand
    assert mission.ran[0] == "ur10_right home (bootstrap)"


def test_resume_after_the_screws_but_before_the_release_still_releases():
    """Killed after part 1's clamp + screw block, before its release
    (``kill_resume.py --kill-after b03-clamp_and_screw``, #12): part 1 is
    clamped and screwed, but ur10_left still holds it, so the part is not
    done -- its release (block B) must run, and no block with an effect
    before it."""
    held = {
        "ur10_right/gripper": "driver/h_grip",
        "ur10_left/gripper": "part1/h_grasp",
        "fixtures/clamp1": "part1/h_seat",
    }
    facts = ["screwed(part1, part1/h_hole1)", "screwed(part1, part1/h_hole2)"]
    mission = _Mission(1, held, facts)
    ok, skipped = mission.go()
    assert ok
    assert mission.ran == [
        "ur10_right home (bootstrap)",  # home moves have no effects: they rerun
        "ur10_right home (part1)",
        "part1 B: release",
        "return: rack driver",
    ]
    assert "part1 assembled" not in skipped
    assert mission.grasp_tracker.current_grasps["ur10_left/gripper"] is None


def test_a_finished_mission_runs_nothing():
    held = {
        "fixtures/clamp1": "part1/h_seat",
        "fixtures/clamp2": "part2/h_seat",
        "fixtures/rack_hold": "driver/h_rack",
    }
    facts = [f"screwed(part{i}, part{i}/h_hole{h})" for i in (1, 2) for h in (1, 2)]
    mission = _Mission(2, held, facts)
    ok, skipped = mission.go()
    assert ok and mission.ran == []


def test_a_failing_block_stops_the_mission():
    mission = _Mission(2)

    def fail(parameters):
        mission.ran.append(parameters["block"])
        raise RuntimeError("planning failed")

    mission.session.registry._entries["clamp_and_screw"] = (
        mission.session.registry._entries["clamp_and_screw"][0],
        fail,
    )
    ok, _ = mission.go()
    assert not ok
    assert mission.ran[-1] == "part1 A: clamp + screw"
    assert json.loads(mission.session.get_report())["completed"]  # earlier steps


def test_an_aborted_step_is_the_runs_failure():
    """#108: a step aborted mid-planning reported no failure of its own."""
    import pytest
    from types import SimpleNamespace

    T = pytest.importorskip("task_screw_assembly")
    plan = SimpleNamespace(
        document={
            "root": {
                "id": "mission",
                "children": [{"id": "b07", "label": "part1 A: clamp + screw"}],
            }
        }
    )
    run = SimpleNamespace(failed_step="b07", message="aborted (abort_step by operator)")
    assert T.stopped_failure(run, SimpleNamespace(plan=plan)) == {
        "step": "part1 A: clamp + screw",
        "facts": [],
        "message": "aborted (abort_step by operator)",
    }
    assert T.stopped_failure(SimpleNamespace(failed_step=None), None) is None
