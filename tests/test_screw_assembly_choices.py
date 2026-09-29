"""Real choices in the screw-assembly domain (issue #16), and the M3 exit test
run logically: an injected ``cannot_reach`` on a clamp makes the mission
replan into a spare clamp and complete (no HPP; the planner runs when the
``planning`` extra is installed).
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

from long_tamp.tasks.task_planning import (
    CapabilityRegistry,
    TaskPlan,
    TaskPlanningSession,
)
from long_tamp.tasks.task_planning.repair import plan_execute_repair
from long_tamp.tasks.task_planning.runner import run_plan

SCREW = Path(__file__).resolve().parents[1] / "script/screw_assembly"
sys.path.insert(0, str(SCREW))
import build_scene  # noqa: E402
import screw_domain  # noqa: E402

LEFT, RIGHT = "ur10_left/gripper", "ur10_right/gripper"


def _valid_pairs(config_text):
    import yaml

    return yaml.safe_load(config_text)["valid_pairs"]


def test_default_scene_keeps_clamp_i_for_part_i():
    pairs = screw_domain.clamp_seats(_valid_pairs(build_scene.config_yaml(2)))
    assert pairs == [
        ("fixtures/clamp1", "part1/h_seat"),
        ("fixtures/clamp2", "part2/h_seat"),
    ]


def test_spare_clamps_take_any_part():
    pairs = screw_domain.clamp_seats(_valid_pairs(build_scene.config_yaml(1, 2)))
    assert pairs == [
        ("fixtures/clamp1", "part1/h_seat"),
        ("fixtures/clamp2", "part1/h_seat"),
    ]
    assert build_scene.fixtures_srdf(2).count('<gripper name="clamp') == 2
    facts = screw_domain.static_facts(1, pairs)
    assert "clamp_takes(fixtures/clamp2, part1/h_seat)" in facts


def _planner():
    from long_tamp.tasks.task_planning.skeleton import (
        FastDownwardPlanner,
        default_planner,
    )

    if FastDownwardPlanner.find() is None:
        pytest.importorskip("unified_planning")
    return default_planner()


SPARE = [("fixtures/clamp1", "part1/h_seat"), ("fixtures/clamp2", "part1/h_seat")]


def test_a_blocked_clamp_is_swapped_for_a_spare_one():
    blocked = [
        ("clamp_and_screw", {"clamp": "fixtures/clamp1", "seat": "part1/h_seat"})
    ]
    steps = _planner().solve(
        screw_domain.pddl_problem(1, blocked=blocked, clamps=SPARE)
    )
    clamps = [p["clamp"] for c, p in steps if c == "clamp_and_screw"]
    assert clamps == ["fixtures/clamp2"]


class _Cell:
    """A fake cell: each step updates the grasps as its block would, and
    recorded facts hold the screws. Steps are counted, to check none is
    redone after the replan."""

    def __init__(self, n_parts, inject):
        self.grasps: dict[str, str] = {}
        self.screwed: set[str] = set()
        self.ran: list[str] = []
        self.inject = dict(inject)
        self.n_parts = n_parts

    def world(self):
        return {f"holds({g}, {h})" for g, h in self.grasps.items()} | self.screwed

    def execute(self, document):
        failure = {}
        registry = CapabilityRegistry()
        for name, descriptor in screw_domain.descriptors(self.n_parts).items():
            registry.register(descriptor, self._runner(name, failure))
        plan = TaskPlan.from_dict(document, registry)
        session = TaskPlanningSession(plan, registry, world_state=self.world)
        run = run_plan(session)
        return None if run.success else failure

    def _runner(self, capability, failure):
        def run(parameters):
            block = screw_domain.block_for(capability, parameters)
            binding = {k: v for k, v in parameters.items() if k != "block"}
            if capability == self.inject.get("capability") and all(
                binding.get(k) == v for k, v in self.inject["match"].items()
            ):
                self.inject = {}  # fires once
                gripper, handle = block["seq"][0]
                failure.update(
                    step=block["label"],
                    capability=capability,
                    parameters=binding,
                    facts=[f"cannot_reach({gripper}, {handle})"],
                )
                raise RuntimeError("injected")
            self.ran.append(block["label"])
            for gripper, handle in block.get("seq", []):
                if handle is None:
                    self.grasps.pop(gripper, None)
                else:
                    self.grasps[gripper] = handle
            if capability == "clamp_and_screw":
                p = parameters
                self.screwed |= {f"screwed({p['part']}, {p['hole1']})",
                                 f"screwed({p['part']}, {p['hole2']})"}  # fmt: skip
            return {}

        return run


def test_the_m3_exit_test_runs_logically():
    """1 part, 2 clamps. The first plan clamps part 1 into whichever clamp the
    planner prefers; that clamp 'cannot reach' the seat (injected), the loop
    blocks it and replans from the world state, the part goes into the other
    clamp, and the mission completes without redoing the pickup."""
    planner = _planner()
    first = planner.solve(screw_domain.pddl_problem(1, clamps=SPARE))
    first_clamp = next(p["clamp"] for c, p in first if c == "clamp_and_screw")
    other = {
        "fixtures/clamp1": "fixtures/clamp2",
        "fixtures/clamp2": "fixtures/clamp1",
    }[first_clamp]
    cell = _Cell(1, {"capability": "clamp_and_screw", "match": {"clamp": first_clamp}})

    def plan(blocked):
        state = sorted(cell.world())
        steps = planner.solve(screw_domain.pddl_problem(1, state, blocked, SPARE))
        return screw_domain.planned_document(steps, 1, state)

    outcome = plan_execute_repair(plan, cell.execute, screw_domain.repair_policy)
    assert outcome.success, outcome.message
    assert len(outcome.rounds) == 2
    assert outcome.blocked == [
        ("clamp_and_screw", {"clamp": first_clamp, "seat": "part1/h_seat"})
    ]
    assert cell.grasps == {other: "part1/h_seat", "fixtures/rack_hold": "driver/h_rack"}
    assert cell.screwed == {
        "screwed(part1, part1/h_hole1)",
        "screwed(part1, part1/h_hole2)",
    }
    # Nothing done before the failure was planned again.
    assert cell.ran.count("bootstrap: pick driver") == 1
    assert cell.ran.count("part1 A0: grasp") == 1
