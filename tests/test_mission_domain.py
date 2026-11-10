"""The MissionDomain interface (#131, ADR-0011): pure Python, no HPP.

A toy pick-and-place domain shows a domain needs only its capabilities and
steps; the screw-assembly adapter must answer exactly what ``screw_domain``
answers (no behaviour change); both are type-checked against the protocol.
"""

from __future__ import annotations

import inspect
import sys
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import pytest

from long_tamp.mission import (
    BaseMissionDomain,
    MissionDomain,
    MissionStep,
    Move,
    ObjectLevelState,
    capability_registry,
    check_domain,
    check_object_goal,
    grasped,
    object_of,
    placed_at,
)
from long_tamp.tasks.refiner import RefinementStep
from long_tamp.tasks.task_planning import CapabilityDescriptor, TaskPlan
from long_tamp.tasks.task_planning.predicates import Atom, Literal, holds, parse_state

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "script/screw_assembly"))
import screw_domain  # noqa: E402
from screw_mission import ScrewAssemblyDomain  # noqa: E402

GRIPPER = "arm/gripper"


class PickPlaceDomain(BaseMissionDomain):
    """One arm moving objects between regions: the shape of a RoboCerebra
    task (#159), with goals over ``grasped`` / ``placed_at``."""

    name = "pick-place"
    recorded_predicates = frozenset({"placed_at"})

    def __init__(self, objects=("mug", "apple"), regions=("table", "tray")):
        self._objects = tuple(objects)
        self._regions = tuple(regions)

    def descriptors(self) -> dict[str, CapabilityDescriptor]:
        return {
            d.capability_id: d
            for d in (
                CapabilityDescriptor(
                    "pick",
                    "1.0",
                    {"gripper": str, "object": str, "region": str},
                    preconditions=(
                        "not grasped(?gripper, _)",
                        "placed_at(?object, ?region)",
                    ),
                    effects=(
                        "grasped(?gripper, ?object)",
                        "not placed_at(?object, ?region)",
                    ),
                    writes=("grasp_state",),
                    restartable=True,
                ),
                CapabilityDescriptor(
                    "place",
                    "1.0",
                    {"gripper": str, "object": str, "region": str},
                    preconditions=("grasped(?gripper, ?object)",),
                    effects=(
                        "not grasped(?gripper, ?object)",
                        "placed_at(?object, ?region)",
                    ),
                    writes=("grasp_state",),
                    restartable=True,
                ),
                CapabilityDescriptor(
                    "tidy", "1.0", {}, preconditions=("placed_at(mug, tray)",)
                ),
            )
        }

    def objects(self) -> tuple[str, ...]:
        return self._objects

    def regions(self) -> tuple[str, ...]:
        return self._regions

    def static_facts(self) -> tuple[str, ...]:
        return (f"gripper({GRIPPER})", *(f"region({r})" for r in self._regions))

    def static_preconditions(self) -> dict[str, tuple[str, ...]]:
        # Export-only: what the planner may bind the parameters to.
        return {
            "pick": ("gripper(?gripper)",),
            "place": ("gripper(?gripper)", "region(?region)"),
        }

    def default_goal(self) -> list[str]:
        return [placed_at("mug", "tray"), grasped("apple", negate=True)]

    def step_for(self, capability: str, parameters: Mapping[str, Any]) -> MissionStep:
        handle = f"{parameters['object']}/handle"
        if capability == "pick":
            sequence: tuple[tuple[str, str | None], ...] = ((GRIPPER, handle),)
        else:
            sequence = ((GRIPPER, None),)
        return RefinementStep(f"{capability} {parameters['object']}", sequence)


START = ["placed_at(mug, table)", "placed_at(apple, table)"]


# -- the protocol ---------------------------------------------------------


@pytest.mark.parametrize(
    "domain", [PickPlaceDomain(), ScrewAssemblyDomain(2)], ids=["toy", "screw"]
)
def test_domains_implement_the_protocol(domain):
    assert isinstance(domain, MissionDomain)
    assert check_domain(domain) == []
    for name, member in inspect.getmembers(MissionDomain, inspect.isfunction):
        if name.startswith("_"):
            continue
        wanted = list(inspect.signature(member).parameters)
        assert list(inspect.signature(getattr(domain, name)).parameters) == wanted[1:]


def test_guards_are_capabilities_that_change_nothing():
    assert PickPlaceDomain().guards() == {"tidy"}
    # A move home has no effects but writes the arm pose: not a guard.
    assert ScrewAssemblyDomain(1).guards() == {"part_done", "all_parts_done"}


def test_check_domain_reports_inconsistencies():
    class Broken(PickPlaceDomain):
        recorded_predicates = frozenset({"glued"})

        def guards(self):
            return frozenset({"pick", "nope"})

        def static_preconditions(self):
            return {"fly": ("wings(?x)",)}

        def default_goal(self):
            return ["placed_at(teapot, tray)", "placed_at(mug, roof)"]

    errors = "\n".join(check_domain(Broken()))
    for expected in (
        "guard 'nope'",
        "guard 'pick' changes the state",
        "unknown capability 'fly'",
        "'glued' is no capability's effect",
        "unknown object 'teapot'",
        "unknown region 'roof'",
    ):
        assert expected in errors


def test_capability_registry_binds_guards_apart():
    registry = capability_registry(PickPlaceDomain(), "run".format, "check".format)
    assert registry.implementation("pick")() == "run"
    assert registry.implementation("tidy")() == "check"


def test_base_defaults():
    domain = PickPlaceDomain()
    export = domain.pddl_problem(START)
    assert "(define (domain pick-place)" in export.domain
    assert "tray" in export.problem
    vocabulary = domain.goal_vocabulary(START)
    assert {"grasped", "placed_at"} <= set(vocabulary.predicates)
    assert {"mug", "tray"} <= set(vocabulary.objects)
    assert domain.plan_document() is None
    assert domain.repair_policy(
        {"capability": "pick", "parameters": {"object": "mug", "block": "x"}}
    ) == [("pick", {"object": "mug"})]
    assert domain.repair_policy({"capability": "tidy", "parameters": {}}) == []
    assert domain.retry({"facts": ["grasp_missed(arm/gripper, mug/handle)"]})
    assert domain.execution_payload({}, None, "path") == "path"
    assert domain.make_backend("none", None) is None  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="unknown backend"):
        domain.make_backend("warp-drive", None)  # type: ignore[arg-type]
    with pytest.raises(NotImplementedError):
        domain.setup()


def test_planned_document_tracks_what_is_held():
    domain = PickPlaceDomain()
    steps = [
        ("pick", {"gripper": GRIPPER, "object": "mug", "region": "table"}),
        ("place", {"gripper": GRIPPER, "object": "mug", "region": "tray"}),
    ]
    seen = []

    class Recording(PickPlaceDomain):
        def expand_step(self, index, capability, parameters, held):
            seen.append(dict(held))
            return super().expand_step(index, capability, parameters, held)

    document = Recording().planned_document(steps, START)
    assert seen == [{}, {GRIPPER: "mug"}]
    registry = capability_registry(domain, lambda **_: None)
    TaskPlan.from_dict(document, registry)
    assert document["mission_id"] == "pick-place"
    labels = [t["label"] for t in document["root"]["children"]]
    assert labels == [
        f"pick({GRIPPER}, mug, table)",
        f"place({GRIPPER}, mug, tray)",
    ]
    assert domain.step_for(*steps[0]).sequence == ((GRIPPER, "mug/handle"),)


def test_a_planner_reaches_an_object_level_goal():
    from long_tamp.tasks.task_planning.skeleton import (
        FastDownwardPlanner,
        default_planner,
    )

    if FastDownwardPlanner.find() is None:
        pytest.importorskip("unified_planning")
    domain = PickPlaceDomain()
    goal = [placed_at("mug", "tray"), placed_at("apple", "tray")]
    steps = default_planner().solve(domain.pddl_problem(START, goal=goal))
    assert [cap for cap, _ in steps] == ["pick", "place", "pick", "place"]
    assert {p["region"] for cap, p in steps if cap == "place"} == {"tray"}
    blocked = [("place", {"object": "apple", "region": "tray"})]
    with pytest.raises(Exception):  # noqa: B017 - NoPlanFound or engine error
        default_planner().solve(domain.pddl_problem(START, blocked, goal))


# -- goals ----------------------------------------------------------------


def test_goal_builders():
    assert grasped("mug") == "grasped(_, mug)"
    assert grasped("mug", GRIPPER, negate=True) == f"not grasped({GRIPPER}, mug)"
    assert placed_at("mug", "tray") == "placed_at(mug, tray)"
    assert object_of("mug/handle") == "mug"
    assert Literal.parse(grasped("mug")).positive


def test_object_level_state_derives_grasped_from_holds():
    source = ObjectLevelState(lambda: [f"holds({GRIPPER}, mug/handle)", "on(a, b)"])
    state = source()
    assert state == {Atom("grasped", (GRIPPER, "mug"))}
    assert holds(Literal.parse(grasped("mug")), state)
    assert holds(Literal.parse(grasped("apple", negate=True)), parse_state(state))


def test_check_object_goal():
    objects, regions = ["mug"], ["tray"]
    assert check_object_goal(["placed_at(mug, tray)", "on(x, y)"], objects, regions) == []
    assert check_object_goal([grasped("_")], objects, regions) == []
    errors = check_object_goal(
        ["grasped(mug)", "placed_at(cup, tray)", "placed_at(mug, roof)", "("],
        objects,
        regions,
    )
    assert len(errors) == 4


# -- the screw-assembly adapter: same answers as screw_domain --------------


@pytest.mark.parametrize("n_parts", [1, 2])
def test_screw_adapter_matches_screw_domain(n_parts):
    clamps = [("fixtures/clamp1", "part1/h_seat"), ("fixtures/clamp2", "part1/h_seat")]
    domain = ScrewAssemblyDomain(n_parts, clamps)
    assert domain.descriptors() == screw_domain.descriptors(n_parts)
    assert domain.plan_document() == screw_domain.build_plan_document(n_parts)
    assert list(domain.static_facts()) == screw_domain.static_facts(n_parts, clamps)
    assert domain.default_goal() == screw_domain.mission_goal(n_parts)
    state = ["holds(ur10_right/gripper, driver/h_grip)"]
    blocked = [("grasp", {"gripper": "ur10_left/gripper"})]
    ours = domain.pddl_problem(state, blocked)
    theirs = screw_domain.pddl_problem(n_parts, state, blocked, clamps)
    assert (ours.domain, ours.problem) == (theirs.domain, theirs.problem)
    assert domain.goal_vocabulary(state) == screw_domain.goal_vocabulary(
        n_parts, state, clamps
    )


def test_screw_adapter_steps_match_the_blocks():
    domain = ScrewAssemblyDomain(2)
    for index, block in enumerate(screw_domain.build_mission(2)):
        cap, params = screw_domain._step(index, block)
        step = domain.step_for(cap, params)
        if "move" in block:
            assert step == Move(block["label"], {"arm": "ur10_right", "carried": "driver"})
        else:
            assert step == screw_domain.refinement_step(block)


def test_screw_adapter_planned_document_and_repair():
    domain = ScrewAssemblyDomain(1)
    steps = [
        ("grasp", {"gripper": "ur10_right/gripper", "handle": "driver/h_grip"}),
        ("grasp", {"gripper": "ur10_left/gripper", "handle": "part1/h_grasp"}),
    ]
    assert domain.planned_document(steps) == screw_domain.planned_document(steps, 1)
    held: dict[str, str] = {}
    assert domain.expand_step(0, *steps[0], held) == screw_domain.expand_step(
        0, *steps[0], {}
    )
    assert held == {}  # the adapter doesn't write to the caller's dict
    for failure in (
        {
            "capability": "clamp_and_screw",
            "parameters": {"clamp": "fixtures/clamp1", "block": "x"},
            "facts": ["cannot_reach(fixtures/clamp1, part1/h_seat)"],
        },
        {"capability": "release", "parameters": {"gripper": "g", "block": "x"}},
        {"capability": "home", "parameters": {"block": "x"}},
    ):
        assert domain.repair_policy(failure) == screw_domain.repair_policy(failure)


def test_screw_adapter_scenarios_and_payload():
    domain = ScrewAssemblyDomain(2)
    scenarios = domain.scenarios()
    assert scenarios["nominal"].setup == ()
    labels = {t["label"] for t in screw_domain.transactions(domain.plan_document())}
    for scenario in scenarios.values():
        assert set(scenario.setup) <= labels
    assert domain.execution_payload({"gripper": "x"}, "e_12", "path") == "path"
    assert domain.execution_payload({"gripper": "driver/tip"}, "e_01", "p") == "p"


# -- type checking --------------------------------------------------------


def test_reference_implementations_type_check():
    api = pytest.importorskip("mypy.api")
    stdout, stderr, status = api.run(
        [
            "--follow-imports=silent",
            "--ignore-missing-imports",
            "--no-error-summary",
            str(ROOT / "src/long_tamp/mission"),
            str(ROOT / "script/screw_assembly/screw_mission.py"),
            str(ROOT / "tests/mission_domain_conformance.py"),
        ]
    )
    assert status == 0, stdout + stderr
