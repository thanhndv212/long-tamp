"""The generic mission runner (#132, ADR-0012): pure Python, no HPP.

The runner drives two domains, the toy pick-and-place one and the screw
assembly, on a fake planner (a grasp tracker), a fake refiner (each phase
one straight path) and the ``MockBackend``: steps, run folder, trajectory,
failures, drift, missed grasps and repair.
"""

from __future__ import annotations

import json
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import pytest

from long_tamp.execution import ExecutionStatus, MockBackend
from long_tamp.mission import (
    MissionRunner,
    MissionScene,
    Move,
    execution_summary,
    parse_injection,
)
from long_tamp.tasks.mission_checkpoint import MissionCheckpoint
from long_tamp.tasks.refiner import Refinement, RefinementStep
from long_tamp.tasks.task_planning import RecordedFacts
from long_tamp.tasks.task_planning.predicates import Literal
from long_tamp.tasks.task_planning.skills import SkillCommand

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "script/screw_assembly"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import screw_domain  # noqa: E402
from test_mission_domain import GRIPPER, START, PickPlaceDomain  # noqa: E402
from screw_mission import ScrewAssemblyDomain  # noqa: E402


class FakeTracker:
    def __init__(self, held=None):
        self.current_grasps: dict[str, str | None] = dict(held or {})

    def update_grasp(self, gripper, handle):
        self.current_grasps[gripper] = handle


class FakePlanner:
    """What the runner reads from a ``GraspSequencePlanner``."""

    def __init__(self, held=None):
        self.grasp_tracker = FakeTracker(held)
        self.resets = 0
        self._call_start: dict[str, str | None] = {}

    def reset_grasp_tracker_to_call_start(self):
        self.resets += 1
        self.grasp_tracker.current_grasps = dict(self._call_start)


class FakePath:
    """A straight path from ``a`` to ``b`` in one second."""

    def __init__(self, a, b):
        self.a, self.b = list(a), list(b)

    def length(self):
        return 1.0

    def eval(self, t):
        return [x + (y - x) * t for x, y in zip(self.a, self.b)], True


class FakeRefiner:
    """Each phase: the grasp applied, the config's first joint + 1. A label
    in ``fail`` fails (once per entry) after committing its first phase."""

    def __init__(self, planner, fail=()):
        self.planner = planner
        self.fail = list(fail)
        self.refined: list[str] = []

    def refine(self, step: RefinementStep, q_init: Sequence[float]) -> Refinement:
        self.refined.append(step.label)
        grasps = self.planner.grasp_tracker.current_grasps
        self.planner._call_start = dict(grasps)
        q, phases = list(q_init), []
        for gripper, handle in step.sequence:
            grasps[gripper] = handle
            if step.label in self.fail:
                self.fail.remove(step.label)
                return Refinement(
                    False,
                    step.label,
                    list(q_init),
                    message="stuck",
                    facts=[f"cannot_reach({gripper}, {handle})"],
                )
            end = [q[0] + 1.0, *q[1:]]
            phases.append(
                {
                    "gripper": gripper,
                    "handle": handle,
                    "paths": [FakePath(q, end)],
                    "edges": ["approach_12" if handle else "release_21"],
                }
            )
            q = end
        return Refinement(True, step.label, q, phases=phases, message="ok")


class ScriptedBackend(MockBackend):
    """The mock backend, failing the first command of ``fail_step`` with
    ``facts``; with ``drift``, the robot is that far off at ``drift_step``'s
    first start; it observes the planned config with joint 1 set to 7."""

    def __init__(self, fail_step=None, facts=(), drift_step=None, drift=0.0):
        super().__init__(rtf=1e6)
        self.fail_step, self.fail_facts = fail_step, tuple(facts)
        self.drift_step, self.drift = drift_step, drift
        self.started: list[str] = []
        self.observed: list[Any] = []

    def start(self, command):
        self.started.append(command.step_id)
        return super().start(command)

    def poll(self):
        if self.command is not None and self.command.step_id == self.fail_step:
            self.fail_step = None
            from long_tamp.execution import Feedback

            return ExecutionStatus.FAILURE, Feedback(
                message="missed", facts=self.fail_facts
            )
        return super().poll()

    def start_error(self, command):
        if command.step_id == self.drift_step:
            self.drift_step = None
            return self.drift
        return 0.0

    def observed_config(self, q, with_objects=False):
        self.observed.append(with_objects)
        return [q[0], 7.0, *q[2:]]


def toy_runner(fail=(), recorded=START):
    planner = FakePlanner()
    domain = PickPlaceDomain()
    facts = RecordedFacts(None, predicates=domain.recorded_predicates)
    facts.apply([Literal.parse(f) for f in recorded])
    scene = MissionScene(task=None, planner=planner, q_init=[0.0, 0.0])
    runner = MissionRunner(domain, scene, facts, refiner=FakeRefiner(planner, fail))
    return runner, planner


def toy_plan(runner, region="tray", objects=("mug",)):
    steps = []
    for obj in objects:
        steps += [
            ("pick", {"gripper": GRIPPER, "object": obj, "region": "table"}),
            ("place", {"gripper": GRIPPER, "object": obj, "region": region}),
        ]
    return runner.domain.planned_document(steps, runner.world_atoms())


# -- one plan -------------------------------------------------------------


def test_a_toy_mission_runs_and_executes(tmp_path):
    runner, planner = toy_runner()
    backend = ScriptedBackend()
    checkpoint = MissionCheckpoint(tmp_path)
    trajectory: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    result = runner.run_mission(
        document=toy_plan(runner, objects=("mug", "apple")),
        backend=backend,
        checkpoint=checkpoint,
        trajectory=trajectory,
        on_event=events.append,
    )
    labels = ["pick mug", "place mug", "pick apple", "place apple"]
    assert result["success"] and result["failure"] is None
    assert [b["label"] for b in result["blocks"]] == labels
    assert backend.started == labels  # one path per phase
    assert result["executed"] == 4
    assert result["final_config"] == [4.0, 0.0]
    assert result["execution"]["commands"] == 4
    assert {"placed_at(mug, tray)", "placed_at(apple, tray)"} <= set(
        runner.world_atoms()
    )
    assert planner.grasp_tracker.current_grasps == {GRIPPER: None}
    # The run folder and the replay input, as the screw runner writes them.
    mission = json.loads((tmp_path / "mission.json").read_text())
    assert [b["label"] for b in mission["blocks"]] == labels
    assert json.loads((tmp_path / "checkpoint.json").read_text())["next_block"] == 4
    replay = json.loads((tmp_path / "trajectory.json").read_text())
    assert replay["id"] == "pick-place" and replay["dt"] == 0.05
    assert [s["block"] for s in replay["segments"]] == labels
    assert replay["segments"][0]["configs"][-1] == [1.0, 0.0]
    assert events[0]["role"] == "plan"


def test_a_step_whose_preconditions_fail_stops_the_mission():
    # The mug is already on the tray: a plan that picks it from the table
    # is stale (the toy domain has no guard to skip it).
    runner, _ = toy_runner(recorded=["placed_at(mug, tray)", "placed_at(apple, table)"])
    result = runner.run_mission(document=_stale(runner))
    assert not result["success"]
    assert result["failure"]["message"] == "not ready: placed_at(mug, table)"
    assert result["blocks"] == [] and runner.refiner.refined == []


def _stale(runner):
    steps = [
        ("pick", {"gripper": GRIPPER, "object": "mug", "region": "table"}),
        ("place", {"gripper": GRIPPER, "object": "mug", "region": "tray"}),
    ]
    return runner.domain.planned_document(steps, START)


def test_a_failed_refinement_stops_the_mission_and_undoes_its_grasps():
    runner, planner = toy_runner(fail=["place mug"])
    result = runner.run_mission(document=toy_plan(runner))
    assert not result["success"]
    failure = result["failure"]
    assert failure["step"] == "place mug"
    assert failure["capability"] == "place"
    assert failure["parameters"]["region"] == "tray"
    assert failure["facts"] == [f"cannot_reach({GRIPPER}, None)"]
    assert planner.resets == 1
    # Where the failed step started: the mug in the hand, after one motion.
    assert planner.grasp_tracker.current_grasps == {GRIPPER: "mug/handle"}
    assert result["final_config"] == [1.0, 0.0]


def test_an_injected_failure_fails_without_planning():
    runner, _ = toy_runner()
    inject = [parse_injection("pick:object=mug")]
    result = runner.run_mission(document=toy_plan(runner), inject=inject)
    assert inject[0]["fired"]
    assert runner.refiner.refined == []
    assert result["failure"]["facts"] == [
        "refinement_failed(pick_mug)",
        f"cannot_reach({GRIPPER}, mug/handle)",
    ]


def test_a_mission_needs_a_plan():
    runner, _ = toy_runner()
    with pytest.raises(ValueError, match="no hand-written plan"):
        runner.run_mission()


def test_drift_replans_the_step_from_the_observed_config():
    runner, _ = toy_runner()
    backend = ScriptedBackend(drift_step="place mug", drift=0.5)
    result = runner.run_mission(document=toy_plan(runner), backend=backend, max_drift=0.1)
    assert result["success"], result
    assert runner.refiner.refined == ["pick mug", "place mug", "place mug"]
    # Replanned from the observed config: the fake observes the robot at
    # the last planned config with joint 1 at 7, as the script's on_drift.
    assert result["final_config"] == [3.0, 7.0]
    assert result["timing"]["drift_replans"] == 1


def test_a_missed_grasp_opens_the_hand_and_observes_the_world():
    class Opened(PickPlaceDomain):
        def opened_config(self, scene, q, grippers):
            self.opened = list(grippers)
            return [*q[:-1], -1.0]

    runner, planner = toy_runner()
    runner.domain = Opened()
    facts = (f"grasp_missed({GRIPPER}, mug)",)
    backend = ScriptedBackend(fail_step="pick mug", facts=facts)
    result = runner.run_mission(document=toy_plan(runner), backend=backend)
    failure = result["failure"]
    assert failure["facts"] == list(facts)
    assert failure["capability"] == "pick"  # added from the step's binding
    assert runner.domain.retry(failure)
    # The missed grasp is undone, the world observed with its objects, and
    # the hand the mock can't open opened in the configuration.
    assert planner.grasp_tracker.current_grasps == {GRIPPER: None}
    assert backend.observed == [True]
    assert runner.domain.opened == [GRIPPER]
    assert result["final_config"] == [1.0, -1.0]


# -- repair ---------------------------------------------------------------


def _task_planner_or_skip():
    from long_tamp.tasks.task_planning.skeleton import FastDownwardPlanner

    if FastDownwardPlanner.find() is None:
        pytest.importorskip("unified_planning")


def test_repair_replans_around_a_failed_place():
    _task_planner_or_skip()
    runner, _ = toy_runner()
    runner.domain = PickPlaceDomain(regions=("table", "tray", "shelf"))
    goal = ["not placed_at(mug, table)", "not grasped(_, mug)"]
    first = runner.plan_from_goal(goal=goal)
    region = next(
        node["parameters"]["region"]
        for node in _operations(first["root"])
        if node["capability"] == "place"
    )
    runner.refiner.fail = ["place mug"]
    result = runner.run_with_repair(None, rounds=3, mission={}, goal=goal)
    assert result["success"], result
    assert result["task_replans"] == 1
    assert result["blocked"] == [
        ("place", {"gripper": GRIPPER, "object": "mug", "region": region})
    ]
    placed = [a for a in runner.world_atoms() if a.startswith("placed_at(mug")]
    assert len(placed) == 1 and region not in placed[0] and "table" not in placed[0]


def _operations(node):
    if node.get("type") == "operation":
        yield node
    for child in node.get("children", []) + (
        [node["child"]] if "child" in node else []
    ):
        yield from _operations(child)


# -- the screw assembly on the generic runner ------------------------------


class LogicalScrewDomain(ScrewAssemblyDomain):
    """The screw assembly with its home moves stubbed (they need HPP)."""

    def plan_move(self, scene, move: Move, q, verbose=True):
        return {
            "success": True,
            "final_config": list(q),
            "resumes": 0,
            "replans": 0,
            "message": "moved",
            "phase_results": [],
        }


def test_the_screw_mission_runs_on_the_generic_runner():
    domain = LogicalScrewDomain(2)
    planner = FakePlanner()
    scene = MissionScene(task=None, planner=planner, q_init=[0.0])
    runner = MissionRunner(domain, scene, refiner=FakeRefiner(planner))
    backend = ScriptedBackend()
    payloads = []
    start = backend.start

    def record(command):
        payloads.append(command.payload)
        return start(command)

    backend.start = record
    result = runner.run_mission(backend=backend)
    assert result["success"], result
    labels = [b["label"] for b in screw_domain.build_mission(2)]
    assert [b["label"] for b in result["blocks"]] == labels
    assert {a for a in runner.world_atoms() if a.startswith("screwed")} == {
        f"screwed(part{i}, part{i}/h_hole{h})" for i in (1, 2) for h in (1, 2)
    }
    # Driving a screw (the driver tip's _12 edge) is the screw skill.
    skills = [p for p in payloads if isinstance(p, SkillCommand)]
    assert len(skills) == 4
    assert {p.parameters["hole"] for p in skills} == {
        f"part{i}/h_hole{h}" for i in (1, 2) for h in (1, 2)
    }
    summary = result["execution"]
    assert summary["screws_driven"] == 0 and "max_screw_lateral_error" in summary


def test_a_resumed_screw_mission_skips_the_done_part():
    domain = LogicalScrewDomain(2)
    # Part 1 seated in its clamp, screwed, and let go: part_done holds.
    planner = FakePlanner({"fixtures/clamp1": "part1/h_seat"})
    recorded = RecordedFacts(None, predicates=domain.recorded_predicates)
    recorded.apply(
        [Literal.parse(f"screwed(part1, part1/h_hole{h})") for h in (1, 2)]
    )
    scene = MissionScene(task=None, planner=planner, q_init=[0.0])
    runner = MissionRunner(domain, scene, recorded, refiner=FakeRefiner(planner))
    result = runner.run_mission()
    assert result["success"], result
    assert not any("part1" in b["label"] for b in result["blocks"])
    assert any("part2" in b["label"] for b in result["blocks"])


# -- summaries ------------------------------------------------------------


def test_execution_summary_adds_the_domain_metrics():
    class Result:
        def __init__(self, status, facts, metrics):
            self.status, self.facts, self.metrics = status, facts, metrics

    class Execution:
        def __init__(self, *args):
            self.result = Result(*args)

    executions = [
        Execution(ExecutionStatus.SUCCESS, ["screwed(part1, part1/h_hole1)"], {}),
        Execution(
            ExecutionStatus.FAILURE,
            [f"grasp_missed({GRIPPER}, mug)"],
            {"lateral_error": 0.002, "tracking_error": 0.1, "sim_seconds": 1.234},
        ),
    ]
    summary = execution_summary(executions, ScrewAssemblyDomain(1))
    assert summary == {
        "commands": 2,
        "failed": 1,
        "sim_seconds": 1.23,
        "max_tracking_error": 0.1,
        "max_drift": None,
        "max_object_drift": None,
        "max_start_drift": None,
        "grasps_held": 0,
        "grasps_missed": 1,
        "screws_driven": 1,
        "max_screw_lateral_error": 0.002,
    }
    assert "screws_driven" not in execution_summary(executions, PickPlaceDomain())
    assert execution_summary([]) is None
