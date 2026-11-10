"""The mission UI for any domain (#133): ``python -m long_tamp.mission``.

``serve`` runs one chat mission of a toy pick-and-place domain (no HPP: the
fake planner and refiner of ``test_mission_runner``) on the mock backend,
with a scripted model; the launcher's restart loop is in ``test_viewer``.
"""

from __future__ import annotations

import io
import json
import sys
from pathlib import Path

import pytest

from long_tamp.mission import MissionChat, MissionScene, ui

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from test_ai_chat import act, scripted_client  # noqa: E402
from test_mission_domain import START, PickPlaceDomain  # noqa: E402
from test_mission_runner import FakePlanner, FakeRefiner  # noqa: E402


class ToyDomain(PickPlaceDomain):
    """The pick-and-place domain with a scene: a fake planner and refiner."""

    intro = "One arm (arm) moves a mug and an apple between a table and a tray."

    def setup(self, seed=None, log_dir="auto"):
        planner = FakePlanner()
        return MissionScene(
            None,
            planner,
            [0.0, 0.0],
            extras={"refiner": FakeRefiner(planner), "facts": list(START)},
        )


def toy_domain() -> ToyDomain:
    return ToyDomain()


def test_load_domain_takes_a_module_or_a_file():
    # (another module object than this one: pytest imports this file as tests.*)
    assert type(ui.load_domain("test_mission_ui:toy_domain")).__name__ == "ToyDomain"
    path = HERE.parent / "script/screw_assembly/screw_mission.py"
    domain = ui.load_domain(f"{path}:domain")
    assert domain.name == "screw-assembly" and domain.n_parts is None
    with pytest.raises(ValueError, match="MODULE:FACTORY"):
        ui.load_domain("no_factory")
    with pytest.raises(ValueError, match="no file"):
        ui.load_domain("missing.py:domain")


def test_options_and_the_serve_command():
    assert ui.parse_option("speed=1.5") == ("speed", 1.5)
    assert ui.parse_option("grasp=contact") == ("grasp", "contact")
    assert ui.parse_option('hole_error=[0, 0.001, 0]') == ("hole_error", [0, 0.001, 0])
    with pytest.raises(ValueError):
        ui.parse_option("speed")
    assert ui.parse_watchdog("10") == (10.0, 30.0)
    args = ui.argparse.Namespace(
        domain="m:f", seed=3, backend="mock", port=1, scene_port=2,
        watchdog="off", model=None,
    )  # fmt: skip
    command = ui.serve_command(args, ["--max-replans", "2"], Path("/r"))
    assert command[1:4] == ["-m", "long_tamp.mission", "serve"]
    assert command[-2:] == ["--max-replans", "2"] and "/r" in command


def test_chat_tools_come_from_the_domain():
    """The tools and their hints are the domain's: no screw names."""
    from test_mission_runner import toy_runner

    runner, _ = toy_runner()
    work = MissionChat(runner)
    assert set(work.actions()) == {"pick", "place"}
    text = work.instructions()
    assert "placed_at" in text and "screw" not in text
    hints = " ".join(t.description for t in work.tools())
    assert "screw" not in hints


def test_serve_runs_a_chat_mission_of_any_domain(monkeypatch, tmp_path, capsys):
    """V2: the toy domain, planned and run from the chat, on the mock
    backend; the run folder is the runner's."""
    pytest.importorskip("unified_planning")
    import long_tamp.ai as ai

    goal = ["placed_at(mug, tray)"]
    client, _ = scripted_client(
        act("planning", ("set_goal", {"literals": goal}), ("plan", {}), done=False),
        act("planned"),
        act("running", ("run", {}), done=False),
        act("the mug is on the tray"),
    )
    monkeypatch.setattr(ai, "configure", lambda env=None: None)
    monkeypatch.setattr(ai, "make_client", lambda model=None, on_call=None: client)
    monkeypatch.setattr(sys, "stdin", io.StringIO("put the mug on the tray\nrun it\n"))
    run_dir = tmp_path / "run"
    code = ui.serve(
        [
            "--domain", "test_mission_ui:toy_domain",
            "--no-web", "--no-scene", "--watchdog", "off",
            "--run-dir", str(run_dir),
        ]
    )  # fmt: skip
    out = capsys.readouterr().out
    assert code == 0, out
    assert "model> the mug is on the tray" in out
    mission = json.loads((run_dir / "mission.json").read_text())
    assert [b["label"] for b in mission["blocks"]] == ["pick mug", "place mug"]
    assert "placed_at(mug, tray)" in (run_dir / "facts.json").read_text()
    assert json.loads((run_dir / "trajectory.json").read_text())["id"] == "pick-place"
    events = [json.loads(line) for line in (run_dir / "events.jsonl").open()]
    tools = [e["name"] for e in events if e["role"] == "tool"]
    assert tools == ["set_goal", "plan", "run"]
