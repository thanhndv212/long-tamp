"""The template mission domain (#135): the whole loop without HPP.

``long_tamp.mission.template`` is what the tutorial
(``docs/usage/write-your-own-mission.md``) builds: plan a goal, run it on
the mock backend, repair a failed place, and a chat mission through the
launcher, all on its kinematic scene.
"""

from __future__ import annotations

import io
import json
import sys
from pathlib import Path

import pytest

from long_tamp.execution import MockBackend
from long_tamp.mission import MissionRunner, check_domain, ui
from long_tamp.mission.template import GRIPPER, TemplateDomain, domain, repair_demo
from long_tamp.tasks.task_planning import RecordedFacts
from long_tamp.tasks.task_planning.predicates import Literal

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_ai_chat import act, scripted_client  # noqa: E402
from test_mission_runner import _operations, _task_planner_or_skip  # noqa: E402


def runner_for(dom: TemplateDomain) -> MissionRunner:
    """What ``serve`` builds from a domain (without the chat and the page)."""
    scene = dom.setup()
    facts = RecordedFacts(None, predicates=dom.recorded_predicates)
    facts.apply([Literal.parse(f) for f in scene.extras["facts"]])
    return MissionRunner(dom, scene, facts)


def test_the_template_is_a_mission_domain():
    dom = domain()
    check_domain(dom)
    assert set(dom.descriptors()) == {"pick", "place"}
    assert dom.regions() == ("table", "tray", "bin")
    runner = runner_for(dom)
    assert {"placed_at(cube, table)", "placed_at(ball, table)"} <= set(
        runner.world_atoms()
    )


def test_the_default_goal_plans_and_runs_on_the_mock_backend(tmp_path):
    _task_planner_or_skip()
    runner = runner_for(domain())
    document = runner.plan_from_goal()
    backend = MockBackend(rtf=1e6)
    result = runner.run_mission(document=document, backend=backend)
    assert result["success"], result["failure"]
    assert sorted(b["label"] for b in result["blocks"]) == [
        "pick ball table",
        "pick cube table",
        "place ball tray",
        "place cube tray",
    ]
    assert {"placed_at(cube, tray)", "placed_at(ball, tray)"} <= set(
        runner.world_atoms()
    )
    assert runner.scene.planner.grasp_tracker.current_grasps == {GRIPPER: None}
    # The objects moved with the tool point: both now sit in the tray.
    kin = runner.scene.extras["kinematics"]
    q = result["final_config"]
    for i, obj in enumerate(kin.objects):
        assert kin.position(q, obj) == pytest.approx(kin.slot("tray", i))


def test_repair_places_elsewhere_when_a_region_is_out_of_reach():
    """Whichever region the first plan places the cube in is out of reach:
    the place fails with ``cannot_reach``, the repair loop blocks that
    binding and the replan places the cube in the other region."""
    _task_planner_or_skip()
    goal = ["not placed_at(cube, table)", "not grasped(_, cube)"]
    first = runner_for(domain()).plan_from_goal(goal=goal)
    region = next(
        n["parameters"]["region"]
        for n in _operations(first["root"])
        if n["capability"] == "place"
    )
    runner = runner_for(TemplateDomain(unreachable=(region,)))
    result = runner.run_with_repair(None, rounds=3, mission={}, goal=goal)
    assert result["success"], result
    assert result["task_replans"] == 1
    assert result["blocked"] == [
        ("place", {"gripper": GRIPPER, "object": "cube", "region": region})
    ]
    placed = [a for a in runner.world_atoms() if a.startswith("placed_at(cube")]
    assert len(placed) == 1 and region not in placed[0] and "table" not in placed[0]


def test_the_repair_demo_cannot_reach_the_tray():
    runner = runner_for(repair_demo())
    step = runner.domain.step_for(
        "place", {"gripper": GRIPPER, "object": "cube", "region": "tray"}
    )
    kin = runner.scene.extras["kinematics"]
    kin.planner.grasp_tracker.current_grasps[GRIPPER] = "cube/handle"
    refinement = kin.refine(step, runner.scene.q_init)
    assert not refinement.success
    assert refinement.facts == [f"cannot_reach({GRIPPER}, tray)"]
    assert kin.planner.grasp_tracker.current_grasps == {GRIPPER: "cube/handle"}


def test_a_chat_mission_of_the_template_through_the_launcher(
    monkeypatch, tmp_path, capsys
):
    """The instruction, the plan and the run from the operator chat, on the
    mock backend: ``python -m long_tamp.mission serve --domain
    long_tamp.mission.template:domain`` with a scripted model."""
    pytest.importorskip("unified_planning")
    import long_tamp.ai as ai

    goal = ["placed_at(ball, bin)", "not grasped(_, ball)"]
    client, _ = scripted_client(
        act("planning", ("set_goal", {"literals": goal}), ("plan", {}), done=False),
        act("planned"),
        act("running", ("run", {}), done=False),
        act("the ball is in the bin"),
    )
    monkeypatch.setattr(ai, "configure", lambda env=None: None)
    monkeypatch.setattr(ai, "make_client", lambda model=None, on_call=None: client)
    monkeypatch.setattr(sys, "stdin", io.StringIO("put the ball in the bin\nrun it\n"))
    run_dir = tmp_path / "run"
    code = ui.serve(
        [
            "--domain", "long_tamp.mission.template:domain",
            "--no-web", "--no-scene", "--watchdog", "off",
            "--run-dir", str(run_dir),
        ]
    )  # fmt: skip
    out = capsys.readouterr().out
    assert code == 0, out
    assert "model> the ball is in the bin" in out
    mission = json.loads((run_dir / "mission.json").read_text())
    assert [b["label"] for b in mission["blocks"]] == [
        "pick ball table",
        "place ball bin",
    ]
    assert "placed_at(ball, bin)" in (run_dir / "facts.json").read_text()
    replay = json.loads((run_dir / "trajectory.json").read_text())
    assert replay["id"] == "template" and len(replay["segments"]) == 2
