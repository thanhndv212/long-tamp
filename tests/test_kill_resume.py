"""Kill a mission mid-run and restart it (issue #12).

The mission process is killed with SIGKILL partway through a step -- no
cleanup -- and started again with the same run folder. The restarted run must
skip every step completed before the kill, because its effect holds in the
recorded world state (not because of a step index), redo the step that was
interrupted, and complete. The screw-assembly version of this test is
``script/screw_assembly/kill_resume.py`` (needs HPP).
"""

import os
import signal
import subprocess
import sys
from pathlib import Path

from long_tamp.tasks.task_planning.events import read_events

MISSION = Path(__file__).parent / "fixtures" / "kill_resume_mission.py"
SRC = Path(__file__).resolve().parents[1] / "src"


def _run(run_dir, *args):
    env = {**os.environ, "PYTHONPATH": f"{SRC}{os.pathsep}{os.environ.get('PYTHONPATH', '')}"}
    return subprocess.run(
        [sys.executable, str(MISSION), str(run_dir), *args],
        env=env,
        capture_output=True,
        text=True,
        timeout=60,
    )


def _executed(events):
    return [e["ir_id"] for e in events if e["role"] == "execute"]


def test_a_killed_mission_resumes_from_world_state(tmp_path):
    first = _run(tmp_path, "b")
    assert first.returncode == -signal.SIGKILL
    killed = read_events(tmp_path / "events.jsonl")
    assert _executed(killed) == ["place-a"]  # b died mid-step, before its result
    assert killed[-1]["role"] == "attempts" and killed[-1]["ir_id"] == "place-b"

    second = _run(tmp_path)
    assert second.returncode == 0, second.stderr
    resumed = read_events(tmp_path / "events.jsonl")[len(killed) :]
    assert _executed(resumed) == ["place-b", "place-c"]  # a is not redone
    skipped = [
        e for e in resumed if e["ir_id"] == "place-a" and e["role"] == "complete"
    ]
    assert [(e["status"], e["message"]) for e in skipped] == [
        ("SUCCESS", "effect_holds")
    ]
    assert (resumed[-1]["ir_id"], resumed[-1]["status"]) == ("mission", "SUCCESS")


def test_a_run_restarted_after_completion_does_nothing(tmp_path):
    assert _run(tmp_path).returncode == 0
    done = read_events(tmp_path / "events.jsonl")
    assert _run(tmp_path).returncode == 0
    again = read_events(tmp_path / "events.jsonl")[len(done) :]
    assert _executed(again) == []
    assert {e["message"] for e in again if e["role"] == "complete"} == {"effect_holds"}


def _load_kill_resume():
    import importlib.util

    path = Path(__file__).resolve().parents[1] / "script/screw_assembly/kill_resume.py"
    spec = importlib.util.spec_from_file_location("screw_kill_resume", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _ev(ir_id, role, status, message=""):
    return {"ir_id": ir_id, "role": role, "status": status, "message": message}


def test_screw_check_passes_when_completed_steps_are_skipped():
    check = _load_kill_resume().check_resume
    before = [
        _ev("b00-grasp", "transaction", "SUCCESS"),
        _ev("b01-home", "transaction", "SUCCESS"),
        _ev("b02-grasp", "attempts", "RUNNING"),
    ]
    after = [
        _ev("b00-grasp", "complete", "SUCCESS", "effect_holds"),
        _ev("b01-home", "execute", "SUCCESS"),  # no effects: runs again
        _ev("b02-grasp", "execute", "SUCCESS"),
        _ev("mission", "sequence", "SUCCESS"),
    ]
    result = check(before, after)
    assert result["pass"]
    assert result["skipped_by_effect"] == ["b00-grasp"]
    assert result["homes_rerun"] == ["b01-home"] and result["planned_again"] == []


def test_screw_check_fails_when_a_completed_step_is_planned_again():
    check = _load_kill_resume().check_resume
    before = [_ev("b00-grasp", "transaction", "SUCCESS")]
    after = [
        _ev("b00-grasp", "execute", "SUCCESS"),
        _ev("mission", "sequence", "SUCCESS"),
    ]
    result = check(before, after)
    assert not result["pass"] and result["planned_again"] == ["b00-grasp"]


def test_screw_check_fails_when_the_resumed_run_does_not_finish():
    check = _load_kill_resume().check_resume
    after = [_ev("b02-grasp", "execute", "FAILURE"), _ev("mission", "sequence", "FAILURE")]
    assert not check([], after)["pass"]
