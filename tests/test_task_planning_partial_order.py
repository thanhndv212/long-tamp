"""Partial-order plans (#21): independent steps become parallel lanes."""

import xml.etree.ElementTree as ET

import pytest

from long_tamp.tasks.task_planning import (
    CapabilityDescriptor,
    CapabilityRegistry,
    TaskPlan,
    TaskPlanningSession,
    compile_behavior_tree,
)
from long_tamp.tasks.task_planning.model import PlanValidationError
from long_tamp.tasks.task_planning.partial_order import (
    depends,
    footprint,
    lanes,
    parallelize,
)
from long_tamp.tasks.task_planning.runner import run_plan
from long_tamp.tasks.task_planning.visualize import to_mermaid


def _registry(world=None, ran=None):
    world = world if world is not None else set()
    ran = ran if ran is not None else []

    def grasp(p):
        ran.append(("grasp", p["handle"]))
        world.add(f"holds({p['gripper']}, {p['handle']})")

    def release(p):
        ran.append(("release", p["gripper"]))
        world.difference_update(
            {a for a in world if a.startswith(f"holds({p['gripper']},")}
        )

    registry = CapabilityRegistry()
    registry.register(
        CapabilityDescriptor(
            "grasp",
            "1.0",
            {"gripper": str, "handle": str},
            resources=("gripper", "handle"),
            preconditions=("not holds(?gripper, _)", "not holds(_, ?handle)"),
            effects=("holds(?gripper, ?handle)",),
            restartable=True,
        ),
        grasp,
    )
    registry.register(
        CapabilityDescriptor(
            "release",
            "1.0",
            {"gripper": str},
            resources=("gripper",),
            preconditions=("holds(?gripper, _)",),
            effects=("not holds(?gripper, _)",),
            restartable=True,
        ),
        release,
    )
    registry.register(
        CapabilityDescriptor(
            "home", "1.0", {"arm": str}, resources=("arm",), restartable=True
        ),
        lambda p: ran.append(("home", p["arm"])),
    )
    registry.register(
        CapabilityDescriptor("wait", "1.0", {"what": str}, restartable=True),
        lambda p: ran.append(("wait", p["what"])),
    )
    return registry


def _step(node_id, capability, **parameters):
    return {
        "type": "transaction",
        "id": node_id,
        "label": node_id,
        "restart_state": ["q_current"],
        "children": [
            {
                "type": "operation",
                "id": f"{node_id}.execute",
                "capability": capability,
                "parameters": parameters,
            }
        ],
    }


def _document(root):
    return {
        "schema_version": "1.0",
        "mission_id": "po-demo",
        "scene": {"id": "fake"},
        "provenance": {"kind": "planner", "generator": "test"},
        "root": root,
    }


def _mission():
    """Right arm home, then the left arm releases part 1 and grasps part 2:
    the home move is independent of the left arm's two steps."""
    return {
        "type": "sequence",
        "id": "root",
        "children": [
            _step("grasp-1", "grasp", gripper="left/gripper", handle="part1/h"),
            _step("home-r", "home", arm="right"),
            _step("release-1", "release", gripper="left/gripper"),
            _step("grasp-2", "grasp", gripper="left/gripper", handle="part2/h"),
            _step("home-l", "home", arm="left"),
        ],
    }


def _fp(registry, step):
    return footprint(step, registry)


def test_resources_conflict_by_path_prefix():
    registry = _registry()
    home = _fp(registry, _step("h", "home", arm="left"))
    release = _fp(registry, _step("r", "release", gripper="left/gripper"))
    other = _fp(registry, _step("o", "home", arm="right"))
    assert depends(home, release) and depends(release, home)
    assert not depends(other, release) and not depends(release, other)


def test_interfering_literals_order_steps():
    registry = _registry()
    grasp_l = _fp(registry, _step("a", "grasp", gripper="l", handle="ball"))
    grasp_r = _fp(registry, _step("b", "grasp", gripper="r", handle="ball"))
    grasp_r2 = _fp(registry, _step("c", "grasp", gripper="r", handle="cube"))
    # the same object: holds(l, ball) interferes with "not holds(_, ball)"
    assert depends(grasp_l, grasp_r)
    assert not depends(grasp_l, grasp_r2)


def test_a_step_with_no_footprint_is_ordered_with_everything():
    registry = _registry()
    wait = _fp(registry, _step("w", "wait", what="x"))
    home = _fp(registry, _step("h", "home", arm="right"))
    assert wait.unknown and depends(wait, home) and depends(home, wait)


def test_lanes_group_independent_chains():
    registry = _registry()
    prints = [_fp(registry, s) for s in _mission()["children"]]
    # grasp-1 | home-r, then release-1 and grasp-2 follow grasp-1 in its
    # lane; home-l needs the left arm and closes nothing (it joins the lane).
    assert lanes(prints) == [[[0, 2, 3, 4], [1]]]


def test_parallelize_builds_a_valid_parallel_node():
    registry = _registry()
    document = parallelize(_document(_mission()), registry)
    plan = TaskPlan.from_dict(document, registry)
    (par,) = plan.document["root"]["children"]
    assert par["type"] == "parallel"
    left, right = par["children"]
    assert [s["id"] for s in left["children"]] == [
        "grasp-1",
        "release-1",
        "grasp-2",
        "home-l",
    ]
    assert right["id"] == "home-r"


def test_parallelize_keeps_other_nodes_as_barriers():
    registry = _registry()
    root = _mission()
    root["children"].insert(
        1,
        {
            "type": "fallback",
            "id": "fb",
            "children": [_step("w", "wait", what="x")],
        },
    )
    document = parallelize(_document(root), registry)
    kinds = [c["type"] for c in document["root"]["children"]]
    # grasp-1 alone, the fallback, then home-r | left lane
    assert kinds == ["transaction", "fallback", "parallel"]


def test_dependent_lanes_are_rejected():
    registry = _registry()
    root = {
        "type": "sequence",
        "id": "root",
        "children": [
            {
                "type": "parallel",
                "id": "par",
                "children": [
                    _step("home-l", "home", arm="left"),
                    _step("release", "release", gripper="left/gripper"),
                ],
            }
        ],
    }
    with pytest.raises(PlanValidationError, match="not independent"):
        TaskPlan.from_dict(_document(root), registry)


def test_a_parallel_node_needs_two_lanes_of_steps():
    registry = _registry()
    for children in (
        [_step("h", "home", arm="left")],
        [
            _step("h", "home", arm="left"),
            {"type": "fallback", "id": "f", "children": []},
        ],
    ):
        root = {"type": "parallel", "id": "par", "children": children}
        with pytest.raises(PlanValidationError):
            TaskPlan.from_dict(_document(root), registry)


def test_the_runner_plans_lanes_in_order_and_reports_the_group():
    world, ran, groups = set(), [], []
    registry = _registry(world, ran)
    plan = TaskPlan.from_dict(parallelize(_document(_mission()), registry), registry)
    session = TaskPlanningSession(plan, registry, world_state=lambda: set(world))
    run = run_plan(session, on_group=lambda node, ok: groups.append((node["id"], ok)))
    assert run.success, run.message
    assert ran == [
        ("grasp", "part1/h"),
        ("release", "left/gripper"),
        ("grasp", "part2/h"),
        ("home", "left"),
        ("home", "right"),
    ]
    assert groups == [("root-par0", None), ("root-par0", True)]


def test_a_group_hook_error_fails_the_parallel_node():
    world = set()
    registry = _registry(world)
    plan = TaskPlan.from_dict(parallelize(_document(_mission()), registry), registry)
    session = TaskPlanningSession(plan, registry, world_state=lambda: set(world))
    run = run_plan(session, on_group=lambda node, ok: None if ok is None else "boom")
    assert not run.success
    assert run.failed_step == "root-par0" and run.message == "boom"


def test_parallel_lowers_to_a_btcpp_parallel_node():
    registry = _registry()
    plan = TaskPlan.from_dict(parallelize(_document(_mission()), registry), registry)
    xml = ET.fromstring(compile_behavior_tree(plan).xml)
    par = xml.find(".//Parallel")
    assert par is not None
    assert par.attrib["success_count"] == "2" and par.attrib["failure_count"] == "1"
    assert par.attrib["_ir_role"] == "parallel"
    assert len(par.findall("./*")) == 2
    assert "⇉" in to_mermaid(plan, registry)


def test_the_screw_assembly_sends_the_driver_home_while_the_part_is_released():
    import sys
    from pathlib import Path

    sys.path.insert(
        0, str(Path(__file__).resolve().parents[1] / "script/screw_assembly")
    )
    import screw_domain

    registry = CapabilityRegistry()
    for descriptor in screw_domain.descriptors(2).values():
        registry.register(descriptor, lambda p: None)
    document = parallelize(screw_domain.build_plan_document(2), registry)
    plan = TaskPlan.from_dict(document, registry)  # lanes checked independent

    groups = []

    def walk(node):
        if node["type"] == "parallel":
            groups.append(
                sorted(c["children"][0]["capability"] for c in node["children"])
            )
        for child in node.get("children", []):
            walk(child)

    walk(plan.document["root"])
    # per part: the right arm's home move | the left arm's release
    assert groups == [["home", "release"], ["home", "release"]]
