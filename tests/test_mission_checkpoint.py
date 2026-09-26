"""MissionCheckpoint: one folder that is a run's resume point and its log."""

import json
import os

import pytest

from long_tamp.tasks.mission_checkpoint import PHASE_DUMP_ENV, MissionCheckpoint

OK = {"success": True, "resumes": 1, "replans": 0, "message": "planned"}
FAIL = {"success": False, "resumes": 40, "replans": 10, "message": "gave up"}


def _mission(tmp_path, **meta):
    return MissionCheckpoint(tmp_path / "run", meta=meta or {"seed": 1})


def test_every_block_is_logged_and_success_advances_the_resume_point(tmp_path):
    ck = _mission(tmp_path)
    ck.record(0, "pick", OK, 1.5, [0.0, 1.0], {"arm/g": "tool/h", "other/g": None})
    ck.record(1, "place", FAIL, 9.0, [9.0, 9.0], {})

    log = json.loads((tmp_path / "run" / "mission.json").read_text())
    assert [b["label"] for b in log["blocks"]] == ["pick", "place"]
    assert log["blocks"][1]["success"] is False
    assert log["meta"] == {"seed": 1}

    point = ck.load()
    assert point["next_block"] == 1, "the failed block must not advance it"
    assert point["q"] == [0.0, 1.0]
    assert point["held"] == {"arm/g": "tool/h"}


def test_finish_records_the_outcome(tmp_path):
    ck = _mission(tmp_path)
    ck.finish(True, 12.345)
    log = json.loads((tmp_path / "run" / "mission.json").read_text())
    assert log["success"] is True and log["seconds"] == 12.35


def test_no_checkpoint_before_the_first_completed_block(tmp_path):
    assert _mission(tmp_path).load() is None


def test_reopening_keeps_the_log_and_notes_the_resume(tmp_path):
    _mission(tmp_path).record(0, "pick", OK, 1.0, [0.0], {})
    ck = MissionCheckpoint(tmp_path / "run", meta={"seed": 2})
    assert [b["label"] for b in ck.blocks] == ["pick"]
    log = json.loads((tmp_path / "run" / "mission.json").read_text())
    assert log["resumes"][0]["seed"] == 2


def test_refuses_to_resume_into_a_changed_mission(tmp_path):
    ck = _mission(tmp_path)
    ck.record(1, "place", OK, 1.0, [0.0], {})
    point = ck.load()
    MissionCheckpoint.expect_label(point, ["pick", "place", "release"])
    with pytest.raises(ValueError, match="definition changed"):
        MissionCheckpoint.expect_label(point, ["pick", "carry", "release"])


def test_restore_grasps_reseeds_the_tracker():
    class Tracker:
        def __init__(self):
            self.current_grasps = {"a/g": "x/h", "b/g": None}

        def update_grasp(self, gripper, handle):
            self.current_grasps[gripper] = handle

    tracker = Tracker()
    MissionCheckpoint.restore_grasps(tracker, {"b/g": "y/h"})
    assert tracker.current_grasps == {"a/g": None, "b/g": "y/h"}


def test_phase_dumps_go_to_a_per_block_subfolder(tmp_path, monkeypatch):
    monkeypatch.delenv(PHASE_DUMP_ENV, raising=False)
    path = _mission(tmp_path).phase_dump_dir(3, "part1 A: clamp")
    assert path == tmp_path / "run" / "phases" / "003_part1_A__clamp"
    assert os.environ[PHASE_DUMP_ENV] == str(path)
