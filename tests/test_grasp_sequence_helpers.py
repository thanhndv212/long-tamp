"""
Unit tests for the _play_single_phase_path dispatch extracted from
GraspSequencePlanner.replay_sequence() (refactor Phase 1B, step 1B.6).

Importing agimus_spacelab requires pyhpp (see
docs/plans/refactor-codebase.md's verification-model notes) even though the
helper under test has no HPP dependency itself, so these tests must run
inside the hpp-arm64 container.
"""

import logging

from agimus_spacelab.tasks.grasp_sequence import GraspSequencePlanner


def _make_planner():
    """Bare GraspSequencePlanner, bypassing __init__.

    _play_single_phase_path only reads self.planner (the backend), so a
    minimal object with just that attribute set is sufficient -- no need
    to construct a full planner with real graph_builder/config_gen wiring.
    """
    planner = object.__new__(GraspSequencePlanner)
    return planner


class _FakeBackend:
    """Backend double recording which playback method was dispatched to.

    Mirrors the 4-way dispatch surface: play_and_record_path_vector (record),
    play_path_vector_with_viz (visualizer), play_path_vector (plain), or none
    (unsupported-backend warning).
    """

    def __init__(self, has_record=True, has_viz=True, has_plain=True,
                 record_raises=False):
        self.record_calls = []
        self.viz_calls = []
        self.plain_calls = []
        self._record_raises = record_raises
        if has_record:
            self.play_and_record_path_vector = self._record
        if has_viz:
            self.play_path_vector_with_viz = self._viz
        if has_plain:
            self.play_path_vector = self._plain

    def _record(self, path, video_name, output_dir, framerate, dt, speed):
        if self._record_raises:
            raise RuntimeError("record boom")
        self.record_calls.append(
            (path, video_name, output_dir, framerate, dt, speed)
        )
        return (7, "/tmp/clip.mp4")

    def _viz(self, path, edge_name, visualizer, speed):
        self.viz_calls.append((path, edge_name, visualizer, speed))
        return 9

    def _plain(self, path, speed):
        self.plain_calls.append((path, speed))
        return 11


def _phase(phase_num=3):
    return {"phase": phase_num, "paths": [], "edges": []}


class TestPlaySinglePhasePath:
    def test_record_branch_returns_video_file(self, caplog):
        planner = _make_planner()
        planner.planner = _FakeBackend()
        with caplog.at_level(logging.INFO, logger="agimus_spacelab"):
            result = planner._play_single_phase_path(
                path="p0", edge_name="e0", phase=_phase(), idx=0,
                record=True, visualizer=None, output_dir="/out",
                video_prefix="pre", framerate=25, dt=0.01, speed=1.0,
            )
        assert result == "/tmp/clip.mp4"
        assert len(planner.planner.record_calls) == 1
        path, vname, outdir, fps, dt, speed = planner.planner.record_calls[0]
        assert path == "p0"
        assert vname == "pre_phase_03_path_01_e0"  # prefix + phase + path + edge
        assert outdir == "/out" and fps == 25 and dt == 0.01 and speed == 1.0
        assert planner.planner.viz_calls == []
        assert planner.planner.plain_calls == []
        assert "✓ Recorded (index 7): /tmp/clip.mp4" in caplog.text

    def test_record_video_name_without_prefix(self, capsys):
        planner = _make_planner()
        planner.planner = _FakeBackend()
        planner._play_single_phase_path(
            path="p0", edge_name=None, phase=_phase(5), idx=2,
            record=True, visualizer=None, output_dir="/out",
            video_prefix=None, framerate=25, dt=0.01, speed=1.0,
        )
        vname = planner.planner.record_calls[0][1]
        assert vname == "phase_05_path_03"  # no prefix, no edge suffix

    def test_visualizer_branch_when_not_recording(self, caplog):
        planner = _make_planner()
        planner.planner = _FakeBackend()
        with caplog.at_level(logging.INFO, logger="agimus_spacelab"):
            result = planner._play_single_phase_path(
                path="p0", edge_name="e0", phase=_phase(), idx=0,
                record=False, visualizer="viz-obj", output_dir="/out",
                video_prefix=None, framerate=25, dt=0.01, speed=2.0,
            )
        assert result is None  # nothing recorded
        assert planner.planner.record_calls == []
        assert planner.planner.viz_calls == [("p0", "e0", "viz-obj", 2.0)]
        assert planner.planner.plain_calls == []
        assert "✓ Played with visualization (stored as index 9)" in caplog.text

    def test_plain_fallback_when_no_record_and_no_viz(self, caplog):
        planner = _make_planner()
        planner.planner = _FakeBackend()
        with caplog.at_level(logging.INFO, logger="agimus_spacelab"):
            result = planner._play_single_phase_path(
                path="p0", edge_name="e0", phase=_phase(), idx=0,
                record=False, visualizer=None, output_dir="/out",
                video_prefix=None, framerate=25, dt=0.01, speed=1.0,
            )
        assert result is None
        assert planner.planner.record_calls == []
        assert planner.planner.viz_calls == []
        assert planner.planner.plain_calls == [("p0", 1.0)]
        assert "✓ Played (stored as index 11)" in caplog.text

    def test_unsupported_backend_warning(self, caplog):
        planner = _make_planner()
        # Backend with none of the three methods
        planner.planner = _FakeBackend(has_record=False, has_viz=False,
                                       has_plain=False)
        with caplog.at_level(logging.WARNING, logger="agimus_spacelab"):
            result = planner._play_single_phase_path(
                path="p0", edge_name="e0", phase=_phase(), idx=0,
                record=False, visualizer=None, output_dir="/out",
                video_prefix=None, framerate=25, dt=0.01, speed=1.0,
            )
        assert result is None
        assert "⚠ Backend does not support PathVector playback" in caplog.text
        assert "Path type: str" in caplog.text  # type("p0").__name__

    def test_record_takes_precedence_over_visualizer(self, capsys):
        # When record=True and the record method exists, the visualizer branch
        # is NOT reached even if a visualizer is passed (mirrors the original
        # if/elif ordering).
        planner = _make_planner()
        planner.planner = _FakeBackend()
        result = planner._play_single_phase_path(
            path="p0", edge_name="e0", phase=_phase(), idx=0,
            record=True, visualizer="viz-obj", output_dir="/out",
            video_prefix=None, framerate=25, dt=0.01, speed=1.0,
        )
        assert result == "/tmp/clip.mp4"
        assert len(planner.planner.record_calls) == 1
        assert planner.planner.viz_calls == []