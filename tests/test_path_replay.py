"""Unit tests for path_replay -- reading a capture back and proving it plays.

The validator's job is to be *independent* of the recorder. The recorder
computed each seam from the path objects it was handed; this re-derives
them from the files on disk, which is the only thing that says the
artifact is playable rather than that the capture code agreed with
itself. So the tests here deliberately corrupt files behind a
well-formed manifest and check the validator notices -- a validator that
trusted the manifest's own numbers would pass every one of them.

Fixtures are built through the real PathRecorder rather than hand-written
JSON: a test that invents its own manifest format stops testing the
pairing the moment either side changes.
"""

import json
import os
from typing import ClassVar

import pytest

from long_tamp.planning.path_recorder import PathRecorder
from long_tamp.planning.path_replay import load_manifest, validate


class FakePath:
    """pyhpp PathVector's shape: length/eval/initial/end."""

    def __init__(self, q0, q1, length=1.0):
        self.q0, self.q1 = list(q0), list(q1)
        self._length = float(length)

    def length(self):
        return self._length

    def eval(self, t):
        u = (t / self._length) if self._length else 0.0
        return [a + (b - a) * u for a, b in zip(self.q0, self.q1)], True

    def initial(self):
        return list(self.q0)

    def end(self):
        return list(self.q1)


def _capture(tmp_path, segments, **kw):
    """Record a run and hand back its loaded manifest."""
    rec = PathRecorder(str(tmp_path / "paths"), **kw)
    for step, (kind, q0, q1) in enumerate(segments):
        rec.begin_step(step, f"step {step}")
        rec.record_path(FakePath(q0, q1), kind=kind, edge_name=f"e{step}")
    return rec, load_manifest(rec.output_dir)


CHAIN = [
    ("grasp", [0.0, 0.0], [1.0, 1.0]),
    ("transit", [1.0, 1.0], [2.0, 0.5]),
    ("release", [2.0, 0.5], [3.0, 0.0]),
]


class TestReading:
    def test_round_trip_through_the_recorder(self, tmp_path):
        _, m = _capture(tmp_path, CHAIN)
        assert len(m) == 3
        assert [s.kind for s in m] == ["grasp", "transit", "release"]
        assert [s.index for s in m] == [0, 1, 2]
        assert m.segments[0].waypoints()[0] == [0.0, 0.0]

    def test_manifest_can_be_named_by_file_or_directory(self, tmp_path):
        rec, _ = _capture(tmp_path, CHAIN)
        assert len(load_manifest(rec.output_dir)) == 3
        assert len(load_manifest(rec.manifest_path)) == 3

    def test_selection_helpers(self, tmp_path):
        _, m = _capture(tmp_path, CHAIN)
        assert [s.index for s in m.by_kind("transit")] == [1]
        assert [s.index for s in m.by_step(2)] == [2]
        assert [s.index for s in m.matching("step 1")] == [1]
        assert [s.index for s in m.matching("e2")] == [2]
        assert m.matching("nothing here") == []

    def test_waypoints_are_not_cached(self, tmp_path):
        """A full run is thousands of configurations; playback holds one
        segment at a time, so re-reading must actually re-read."""
        _, m = _capture(tmp_path, CHAIN)
        seg = m.segments[0]
        assert seg.waypoints() is not seg.waypoints()


class TestVerdict:
    def test_a_clean_capture_validates(self, tmp_path):
        _, m = _capture(tmp_path, CHAIN)
        report = validate(m)
        assert report.ok
        assert report.num_segments == 3
        assert report.worst_seam == pytest.approx(0.0)
        assert report.total_waypoints > 0
        assert "continuous" in report.summary()

    def test_a_gap_between_segments_fails(self, tmp_path):
        _, m = _capture(
            tmp_path,
            [
                ("grasp", [0.0, 0.0], [1.0, 1.0]),
                ("transit", [1.0, 1.9], [2.0, 2.0]),  # 0.9 off the previous end
            ],
        )
        report = validate(m)
        assert not report.ok
        assert report.seam_errors[0][0] == 1
        assert report.worst_seam == pytest.approx(0.9)
        assert "NOT continuous" in report.summary()


class TestIndependenceFromTheManifest:
    """Corrupt the files, keep the manifest pristine, expect a failure.

    Every case here passes if the validator reads the manifest's own
    seam_error instead of the data.
    """

    def test_a_missing_segment_file_is_caught(self, tmp_path):
        rec, m = _capture(tmp_path, CHAIN)
        os.remove(m.segments[1].waypoint_path)
        report = validate(load_manifest(rec.output_dir))
        assert not report.ok
        assert report.missing_files == [m.segments[1].record["waypoint_file"]]

    def test_a_rewritten_segment_file_is_caught(self, tmp_path):
        """The manifest still claims the original endpoints."""
        rec, m = _capture(tmp_path, CHAIN)
        path = m.segments[1].waypoint_path
        with open(path) as f:
            data = json.load(f)
        data["waypoints"] = [[9.0, 9.0], [9.5, 9.5]]
        with open(path, "w") as f:
            json.dump(data, f)

        report = validate(load_manifest(rec.output_dir))
        assert not report.ok
        assert report.endpoint_errors, "file must be checked against its record"
        assert report.seam_errors

    def test_an_empty_segment_file_is_caught(self, tmp_path):
        rec, m = _capture(tmp_path, CHAIN)
        path = m.segments[0].waypoint_path
        with open(path) as f:
            data = json.load(f)
        data["waypoints"] = []
        with open(path, "w") as f:
            json.dump(data, f)
        report = validate(load_manifest(rec.output_dir))
        assert report.empty_segments == [0]
        assert not report.ok

    def test_a_configuration_size_change_is_caught(self, tmp_path):
        """Two captures of different scenes concatenated, or a model
        changed under a resumed run."""
        rec, m = _capture(tmp_path, CHAIN)
        path = m.segments[2].waypoint_path
        with open(path) as f:
            data = json.load(f)
        data["waypoints"] = [[2.0, 0.5, 0.0], [3.0, 0.0, 0.0]]
        with open(path, "w") as f:
            json.dump(data, f)
        report = validate(load_manifest(rec.output_dir))
        assert report.size_changes == [(2, 2, 3)]
        assert not report.ok


class TestQuaternions:
    """Same double-cover reasoning as the recorder's own seam check."""

    A: ClassVar[list[float]] = [0.0, 0.0, 0.0, 0.183, 0.683, 0.183, -0.683]
    A_NEG: ClassVar[list[float]] = [0.0, 0.0, 0.0, -0.183, -0.683, -0.183, 0.683]

    def test_a_sign_flip_is_not_a_gap(self, tmp_path):
        _, m = _capture(
            tmp_path,
            [
                ("grasp", [0.0] * 7, self.A),
                ("grasp", self.A_NEG, [1.0] * 7),
            ],
            quaternion_starts=[3],
        )
        assert validate(m, quaternion_starts=[3]).ok

    def test_the_layout_comes_from_the_manifest_by_default(self, tmp_path):
        """The reason the header carries it: --check has no scene to derive
        a layout from, and without one this capture reads as broken."""
        _, m = _capture(
            tmp_path,
            [
                ("grasp", [0.0] * 7, self.A),
                ("grasp", self.A_NEG, [1.0] * 7),
            ],
            quaternion_starts=[3],
        )
        assert m.quaternion_starts == [3]
        assert validate(m).ok

    def test_a_manifest_without_the_field_reports_none(self, tmp_path):
        """Captures written before the field existed still load."""
        rec, _ = _capture(tmp_path, [("grasp", [0.0] * 7, self.A)])
        with open(rec.manifest_path) as f:
            data = json.load(f)
        del data["quaternion_starts"]
        with open(rec.manifest_path, "w") as f:
            json.dump(data, f)
        assert load_manifest(rec.output_dir).quaternion_starts == []

    def test_without_the_layout_the_same_capture_reads_as_broken(self, tmp_path):
        """Pinned deliberately: the validator cannot infer the layout, and
        guessing at it would be worse than saying so."""
        _, m = _capture(
            tmp_path,
            [
                ("grasp", [0.0] * 7, self.A),
                ("grasp", self.A_NEG, [1.0] * 7),
            ],
            quaternion_starts=[3],
        )
        assert not validate(m, quaternion_starts=[]).ok


class TestSamplingResolution:
    """Step size inside a segment -- invisible to any seam check."""

    def test_worst_step_is_measured(self, tmp_path):
        _, m = _capture(tmp_path, [("grasp", [0.0], [1.0])], dt=0.25)
        report = validate(m)
        # 5 samples over a unit span -> steps of 0.25
        assert report.worst_step == pytest.approx(0.25)
        assert report.worst_step_at == 0

    def test_coarse_segments_are_reported_but_do_not_fail(self, tmp_path):
        """Sampling resolution is a playback-quality question, not a
        continuity one; a coarse capture is still a valid capture."""
        _, m = _capture(tmp_path, [("grasp", [0.0], [10.0])], dt=1.0)
        report = validate(m, max_step=0.5)
        assert report.coarse_segments == {0}
        assert report.ok
        assert "coarsely sampled" in report.summary()

    def test_no_max_step_means_no_flagging(self, tmp_path):
        _, m = _capture(tmp_path, [("grasp", [0.0], [10.0])], dt=1.0)
        report = validate(m)
        assert report.coarse_segments == set()


class TestEmptyCapture:
    def test_a_run_that_captured_nothing_validates_vacuously(self, tmp_path):
        """A run killed before its first path still leaves a manifest; the
        replayer must read it rather than crash on it."""
        rec = PathRecorder(str(tmp_path / "paths"))
        m = load_manifest(rec.output_dir)
        report = validate(m)
        assert len(m) == 0
        assert report.ok
        assert report.total_waypoints == 0
