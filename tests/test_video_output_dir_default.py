"""
Unit tests for the shared video-output-directory default, replacing the
hardcoded personal path (``/home/dvtnguyen/devel/demos``) that was
duplicated across 8 method signatures in 4 files.

See docs/plans/refactor-codebase.md's cross-cutting "hardcoded personal
path default" note (fixed 2026-08-09).

Importing agimus_spacelab requires pyhpp even though most of what's under
test here has no HPP dependency itself (see that same doc's
verification-model notes), so these tests must run inside the
hpp-arm64 container.
"""

import inspect
from pathlib import Path

from agimus_spacelab.backends.pyhpp import PyHPPBackend
from agimus_spacelab.tasks.base import ManipulationTask
from agimus_spacelab.tasks.grasp_sequence import GraspSequencePlanner
from agimus_spacelab.visualization import default_video_output_dir
from agimus_spacelab.visualization.video_recorder import (
    VideoRecorder,
    record_path_playback,
)


class TestDefaultVideoOutputDir:
    def test_falls_back_to_home_devel_demos(self, monkeypatch):
        monkeypatch.delenv("AGIMUS_VIDEO_OUTPUT_DIR", raising=False)
        monkeypatch.setattr(Path, "home", lambda: Path("/fake/home"))

        assert default_video_output_dir() == str(Path("/fake/home/devel/demos"))

    def test_env_var_overrides_fallback(self, monkeypatch):
        monkeypatch.setenv("AGIMUS_VIDEO_OUTPUT_DIR", "/custom/video/dir")

        assert default_video_output_dir() == "/custom/video/dir"


class TestVideoRecorderResolvesDefault:
    def test_none_output_dir_resolves_to_shared_default(self, monkeypatch):
        monkeypatch.setenv("AGIMUS_VIDEO_OUTPUT_DIR", "/custom/video/dir")

        recorder = VideoRecorder(viewer=object(), output_dir=None)

        assert recorder.output_dir == "/custom/video/dir"

    def test_explicit_output_dir_is_preserved(self, monkeypatch):
        monkeypatch.setenv("AGIMUS_VIDEO_OUTPUT_DIR", "/custom/video/dir")

        recorder = VideoRecorder(viewer=object(), output_dir="/explicit/dir")

        assert recorder.output_dir == "/explicit/dir"

    def test_default_output_dir_when_omitted(self, monkeypatch):
        monkeypatch.setenv("AGIMUS_VIDEO_OUTPUT_DIR", "/custom/video/dir")

        recorder = VideoRecorder(viewer=object())

        assert recorder.output_dir == "/custom/video/dir"


class TestNoHardcodedPersonalPathRemains:
    """Regression guard: none of the 6 signatures that used to hardcode
    ``/home/dvtnguyen/devel/demos`` should default to any literal path
    string again -- all must default to None (resolved later via
    default_video_output_dir())."""

    def _assert_output_dir_defaults_to_none(self, func):
        sig = inspect.signature(func)
        assert "output_dir" in sig.parameters, func
        assert sig.parameters["output_dir"].default is None, (
            func,
            sig.parameters["output_dir"].default,
        )

    def test_manipulation_task_run(self):
        self._assert_output_dir_defaults_to_none(ManipulationTask.run)

    def test_grasp_sequence_planner_replay_sequence(self):
        self._assert_output_dir_defaults_to_none(
            GraspSequencePlanner.replay_sequence
        )

    def test_pyhpp_play_and_record_path(self):
        self._assert_output_dir_defaults_to_none(PyHPPBackend.play_and_record_path)

    def test_pyhpp_play_and_record_path_vector(self):
        self._assert_output_dir_defaults_to_none(
            PyHPPBackend.play_and_record_path_vector
        )

    def test_video_recorder_init(self):
        self._assert_output_dir_defaults_to_none(VideoRecorder.__init__)

    def test_record_path_playback(self):
        self._assert_output_dir_defaults_to_none(record_path_playback)
