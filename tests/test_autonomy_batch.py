"""The M6 exit test's audit (#91): decisions must stay within the rules."""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script/screw_assembly"))
import autonomy_batch as A  # noqa: E402

GOAL = ["a", "b", "c"]


def test_relaxed_goals_must_be_strict_subsets_and_actions_known():
    sup = {
        "original_goal": GOAL,
        "decisions": [
            {"action": "relax_goal", "goal": ["a", "c"]},
            {"action": "relax_goal", "goal": ["a", "z"]},
            {"action": "relax_goal", "goal": GOAL},
            {"action": "teleport"},
            {"action": "escalate"},
        ],
    }
    problems = A.audit(sup)
    assert len(problems) == 3 and "teleport" in problems[2]


def test_model_calls_are_counted_from_the_event_stream(tmp_path):
    events = tmp_path / "events.jsonl"
    lines = [
        {
            "role": "model",
            "name": "goal",
            "status": "SUCCESS",
            "metrics": {"input_tokens": 10, "output_tokens": 5, "seconds": 1.5},
        },
        {
            "role": "model",
            "name": "supervise",
            "status": "FAILURE",
            "metrics": {"seconds": 0.5},
        },
        {"role": "motion", "name": "x", "status": "SUCCESS"},
    ]
    events.write_text("\n".join(json.dumps(e) for e in lines))
    calls = A.model_calls(events)
    assert calls == {
        "model_calls": {"goal": 1, "supervise": 1},
        "model_tokens": 15,
        "model_seconds": 2.0,
        "model_errors": 1,
    }


def test_the_gate_fails_on_an_unclean_run_or_an_unchecked_decision():
    ok = {"outcome": "completed", "decisions": ["relax_goal"], "unchecked": []}
    stopped = {"outcome": "escalated", "decisions": ["escalate"], "unchecked": []}
    assert A.summarize([ok, stopped])["gate"] == "PASS"
    assert A.summarize([ok, {"outcome": "unclean"}])["gate"] == "FAIL"
    assert A.summarize([{**ok, "unchecked": ["x"]}])["gate"] == "FAIL"


def test_concurrent_missions_can_resolve_the_same_urdf(tmp_path, monkeypatch):
    """Missions started together write the same cached URDF (the exit test's
    seed 2 crashed on it): every writer must succeed."""
    import threading

    from long_tamp.backends import _urdf_paths as U

    monkeypatch.setattr(U, "_CACHE", tmp_path / "cache")
    meshes = tmp_path / "pkg" / "meshes"
    meshes.mkdir(parents=True)
    (meshes / "a.stl").write_text("solid a")
    urdf = tmp_path / "pkg" / "robot.urdf"
    urdf.write_text(
        '<robot><link><visual><geometry><mesh filename="meshes/a.stl"/></geometry></visual></link></robot>'
    )
    results, errors = [], []

    def resolve():
        try:
            results.append(U.resolve_mesh_paths(str(urdf)))
        except Exception as error:  # noqa: BLE001
            errors.append(error)

    threads = [threading.Thread(target=resolve) for _ in range(16)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert not errors and len(set(results)) == 1
    assert not list((tmp_path / "cache").glob("*.tmp"))
