"""The screw-assembly batch gate (script/screw_assembly/summarize.py).

Pure Python, no HPP: the gate decides whether a change may merge or a
milestone may close, so its thresholds are pinned here.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

_SCRIPT = Path(__file__).resolve().parents[1] / "script" / "screw_assembly"


def _load():
    spec = importlib.util.spec_from_file_location(
        "screw_summarize", _SCRIPT / "summarize.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


summarize = _load()


def _run(seed, success=True, seconds=600.0, resumes=0, replans=0, parts=4):
    blocks = [
        {"label": "part1 A0: grasp", "success": True, "seconds": 10.0,
         "resumes": resumes, "replans": replans},
        {"label": "ur10_right home (part1)", "success": True, "seconds": 5.0,
         "resumes": 0, "replans": 0},
    ]  # fmt: skip
    return {
        "seed": seed,
        "parts": parts,
        "success": success,
        "seconds": seconds,
        "blocks": blocks,
    }


def test_compute_stats_matches_results_schema():
    stats = summarize.compute_stats([_run(1), _run(2, resumes=2)])
    baseline = json.loads(
        (_SCRIPT / "results" / "pypi-wheel-batch-2026-09-26.json").read_text()
    )
    shared = {
        "completed", "attempted", "replanning_blocks", "planning_blocks",
        "recovered_failures", "failure_episodes", "seconds_median", "seeds",
    }  # fmt: skip
    assert shared <= stats.keys()
    assert shared <= baseline.keys()
    # Home moves never count as planning blocks.
    assert stats["planning_blocks"] == 2
    assert stats["failure_episodes"] == 1
    assert stats["recovered_failures"] == 1


def test_gate_passes_a_clean_batch():
    stats = summarize.compute_stats([_run(s) for s in range(1, 11)])
    assert summarize.check_gate(stats, {"seconds_median": 600.0}) == []


@pytest.mark.parametrize(
    ("runs", "expected"),
    [
        ([_run(1), _run(2, success=False)], "missions completed 1/2"),
        ([_run(1, replans=1)] + [_run(s) for s in range(2, 11)], "replanning"),
        ([_run(1, seconds=900.0)], "median mission time"),
    ],
)
def test_gate_reports_each_failure(runs, expected):
    stats = summarize.compute_stats(runs)
    failures = summarize.check_gate(stats, {"seconds_median": 600.0})
    assert any(expected in f for f in failures), failures


def test_main_writes_json_and_fails_gate(tmp_path, capsys):
    batch = tmp_path / "batch"
    batch.mkdir()
    for s in (1, 2):
        run = _run(s, success=(s == 1))
        (batch / f"seed_{s:03d}.json").write_text(json.dumps(run))
    out = tmp_path / "result.json"
    code = summarize.main([str(batch), "--json", str(out), "--gate"])
    assert code == 1
    assert json.loads(out.read_text())["attempted"] == 2
    assert "gate: FAIL" in capsys.readouterr().out
