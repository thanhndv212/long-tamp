"""Unit tests for PathRecorder -- the capture layer for full-run replay.

What is being pinned here is not "files appear" but the four properties a
manifest has to have for a 40-step, multi-hour run to be replayable:

1. **Global ordering.** The reason capture could not just be
   ``GraspSequencePlanner.auto_save_dir``: that names files by phase index
   within one ``plan_sequence()`` call, and a block-decomposed run makes
   one call per block, so every block writes ``phase_01_edge_01`` over the
   last one. Segments here are numbered run-globally, in production order.
2. **Crash safety.** The manifest is complete and parseable after every
   segment, because a run gets SIGTERM'd on a wall-clock bound.
3. **Seam detection at write time.** A discontinuity between consecutive
   segments is what makes a replay silently wrong; it is recorded when it
   happens, not discovered at playback.
4. **Resume does not clobber.** A resumed run re-enters a directory
   holding the earlier attempt's segments and must continue after them.

The path objects are stubs: the recorder only ever calls ``length()``,
``call(t)`` and the two endpoint getters, which is exactly the contract an
HPP path satisfies. Testing against stubs keeps this a real unit test --
no scene, no solver -- while covering the code that runs in production
unchanged.
"""

import json
import os
from typing import ClassVar

import pytest

from agimus_spacelab.planning.path_recorder import PathRecorder, SeamError


class FakePath:
    """Linear path between two configs, shaped like pyhpp's PathVector.

    ``pyhpp.core.path.Vector`` exposes ``length()``, ``eval(t)``,
    ``__call__(t)``, ``initial()`` and ``end()`` -- and notably *not*
    ``call(t)`` or ``getEndConfig()``, the CORBA-shaped names this
    codebase's older call sites reach for. Getting that wrong is not
    hypothetical: the first live run of the recorder captured zero
    segments, failing every one with ``'Vector' object has no attribute
    'call'``. ``FakeCorbaPath`` below covers the other shape.
    """

    def __init__(self, q0, q1, length=1.0, failing_samples=()):
        self.q0 = list(q0)
        self.q1 = list(q1)
        self._length = float(length)
        self._failing = set(failing_samples)

    def length(self):
        return self._length

    def eval(self, t):
        # pyhpp returns (q, success); a sample can legitimately fail.
        if t in self._failing:
            return [0.0] * len(self.q0), False
        u = (t / self._length) if self._length else 0.0
        return [a + (b - a) * u for a, b in zip(self.q0, self.q1)], True

    def initial(self):
        return list(self.q0)

    def end(self):
        return list(self.q1)


class FakeCorbaPath(FakePath):
    """The other accessor spelling, plus a bare-q evaluator."""

    eval = None  # not offered by this shape
    initial = None
    end = None

    def call(self, t):
        q, _ok = FakePath.eval(self, t)
        return q  # bare q, no success flag

    def getInitialConfig(self):  # HPP binding name, not snake_case
        return list(self.q0)

    def getEndConfig(self):  # HPP binding name, not snake_case
        return list(self.q1)


class FakeCallablePath(FakePath):
    """Only ``__call__`` -- the accessor play_path() uses."""

    eval = None

    def __call__(self, t):
        return FakePath.eval(self, t)


def _rec(tmp_path, **kw):
    return PathRecorder(str(tmp_path / "paths"), **kw)


def _manifest(recorder):
    with open(recorder.manifest_path) as f:
        return json.load(f)


class TestOrdering:
    """Property 1 -- one global order across blocks and arm moves."""

    def test_segments_are_numbered_globally_in_production_order(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "block A")
        r.record_path(FakePath([0.0], [1.0]), kind="grasp", edge_name="e1")
        r.record_path(FakePath([1.0], [2.0]), kind="grasp", edge_name="e2")
        r.begin_step(1, "UR10 home")
        r.record_path(FakePath([2.0], [3.0]), kind="transit", edge_name="home")

        segs = _manifest(r)["segments"]
        assert [s["index"] for s in segs] == [0, 1, 2]
        assert [s["kind"] for s in segs] == ["grasp", "grasp", "transit"]
        assert [s["step_index"] for s in segs] == [0, 0, 1]
        assert [s["step_label"] for s in segs] == ["block A", "block A", "UR10 home"]

    def test_same_edge_name_in_two_blocks_does_not_collide(self, tmp_path):
        """The exact defect that made auto_save_dir unusable: every block's
        first phase produces the same edge, hence the same filename."""
        r = _rec(tmp_path)
        for step in (0, 1):
            r.begin_step(step, f"RS{step} A")
            r.record_path(
                FakePath([float(step)], [float(step) + 1]),
                kind="grasp",
                edge_name="ur10 > FG | f_01",
            )
        files = [s["waypoint_file"] for s in _manifest(r)["segments"]]
        assert len(set(files)) == 2
        for f in files:
            assert os.path.exists(os.path.join(r.output_dir, f))

    def test_edge_names_with_slashes_and_pipes_are_filesystem_safe(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        rec = r.record_path(
            FakePath([0.0], [1.0]),
            kind="grasp",
            edge_name="spacelab/g_ur10_tool > frame_gripper/h_FG_tool | f_01",
        )
        assert "/" not in rec["waypoint_file"]
        assert os.path.exists(os.path.join(r.output_dir, rec["waypoint_file"]))


class TestCrashSafety:
    """Property 2 -- the manifest is always complete on disk."""

    def test_manifest_is_valid_after_every_segment(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        for i in range(4):
            r.record_path(
                FakePath([float(i)], [float(i + 1)]), kind="grasp", edge_name=f"e{i}"
            )
            assert len(_manifest(r)["segments"]) == i + 1

    def test_manifest_exists_before_any_segment(self, tmp_path):
        """A run killed during its first phase still leaves a readable,
        empty manifest rather than a missing file the replayer must
        special-case."""
        r = _rec(tmp_path)
        assert _manifest(r)["segments"] == []

    def test_no_tmp_file_is_left_behind(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        r.record_path(FakePath([0.0], [1.0]), kind="grasp")
        assert not os.path.exists(r.manifest_path + ".tmp")


class TestSeams:
    """Property 3 -- discontinuities are caught while the run is going."""

    def test_continuous_segments_report_zero_seam_error(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        r.record_path(FakePath([0.0, 0.0], [1.0, 1.0]), kind="grasp")
        rec = r.record_path(FakePath([1.0, 1.0], [2.0, 2.0]), kind="transit")
        assert rec["seam_error"] == 0.0
        assert r.seam_violations == 0

    def test_a_gap_is_recorded_and_counted_but_does_not_raise(self, tmp_path):
        """A run that has already spent hours planning must not die at a
        write -- the manifest carries the evidence instead."""
        r = _rec(tmp_path, seam_tolerance=1e-3)
        r.begin_step(0, "s")
        r.record_path(FakePath([0.0], [1.0]), kind="grasp")
        rec = r.record_path(FakePath([1.5], [2.0]), kind="transit")
        assert rec["seam_error"] == pytest.approx(0.5)
        assert r.seam_violations == 1
        assert _manifest(r)["seam_violations"] == 1

    def test_strict_mode_raises(self, tmp_path):
        r = _rec(tmp_path, strict_seams=True)
        r.begin_step(0, "s")
        r.record_path(FakePath([0.0], [1.0]), kind="grasp")
        with pytest.raises(SeamError):
            r.record_path(FakePath([9.0], [10.0]), kind="transit")

    def test_first_segment_has_no_seam_to_check(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        assert r.record_path(FakePath([3.0], [4.0]), kind="grasp")["seam_error"] is None

    def test_a_config_size_change_is_a_violation_not_a_crash(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        r.record_path(FakePath([0.0, 0.0], [1.0, 1.0]), kind="grasp")
        rec = r.record_path(FakePath([1.0], [2.0]), kind="transit")
        assert rec["seam_error"] == float("inf")
        assert r.seam_violations == 1


class TestQuaternionSeams:
    """A sign flip is not a discontinuity.

    Measured on the first captured run: the RS1 FG grasp's ``_01``/``_12``
    boundary reported max |dq| = 1.366, from exactly four negated
    components (dot = -1.0) with identical translation -- HPP had simply
    picked the other half of the double cover. With six RS parts and two
    tools all on freeflyers, treating that as a gap makes the check fire
    on most seams and mean nothing.
    """

    # x,y,z then a quaternion at index 3.
    A: ClassVar[list[float]] = [0.0, 0.0, 0.0, 0.183, 0.683, 0.183, -0.683]
    A_NEG: ClassVar[list[float]] = [0.0, 0.0, 0.0, -0.183, -0.683, -0.183, 0.683]

    def test_a_negated_quaternion_is_not_a_gap(self, tmp_path):
        r = _rec(tmp_path, quaternion_starts=[3])
        r.begin_step(0, "s")
        r.record_path(FakePath([0.0] * 7, self.A), kind="grasp")
        rec = r.record_path(FakePath(self.A_NEG, [1.0] * 7), kind="grasp")
        assert rec["seam_error"] == pytest.approx(0.0, abs=1e-12)
        assert r.seam_violations == 0

    def test_the_flip_is_recorded_for_the_replayer(self, tmp_path):
        r = _rec(tmp_path, quaternion_starts=[3])
        r.begin_step(0, "s")
        r.record_path(FakePath([0.0] * 7, self.A), kind="grasp")
        rec = r.record_path(FakePath(self.A_NEG, [1.0] * 7), kind="grasp")
        assert rec["quaternion_flips"] == [3]

    def test_stored_waypoints_keep_the_planner_s_sign(self, tmp_path):
        """Canonicalization is for the comparison only -- rewriting the
        captured motion would put the manifest out of step with the
        segment files."""
        r = _rec(tmp_path, quaternion_starts=[3])
        r.begin_step(0, "s")
        r.record_path(FakePath([0.0] * 7, self.A), kind="grasp")
        rec = r.record_path(FakePath(self.A_NEG, [1.0] * 7), kind="grasp")
        assert rec["q_start"] == self.A_NEG
        with open(os.path.join(r.output_dir, rec["waypoint_file"])) as f:
            assert json.load(f)["waypoints"][0] == self.A_NEG

    def test_a_real_rotation_change_still_reports(self, tmp_path):
        """Only exact hemisphere flips are absorbed; a genuinely different
        orientation still fails the check."""
        r = _rec(tmp_path, quaternion_starts=[3])
        r.begin_step(0, "s")
        r.record_path(FakePath([0.0] * 7, self.A), kind="grasp")
        rotated = [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        rec = r.record_path(FakePath(rotated, [1.0] * 7), kind="grasp")
        assert rec["seam_error"] > 0.3
        assert r.seam_violations == 1

    def test_without_the_layout_the_flip_reads_as_a_gap(self, tmp_path):
        """The pre-fix behaviour, kept explicit: a recorder given no
        quaternion layout cannot know, and says so rather than guessing."""
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        r.record_path(FakePath([0.0] * 7, self.A), kind="grasp")
        rec = r.record_path(FakePath(self.A_NEG, [1.0] * 7), kind="grasp")
        assert rec["seam_error"] == pytest.approx(1.366)
        assert "quaternion_flips" not in rec

    def test_translation_drift_is_not_absorbed(self, tmp_path):
        """The flip must not smuggle a positional gap through with it."""
        r = _rec(tmp_path, quaternion_starts=[3])
        r.begin_step(0, "s")
        r.record_path(FakePath([0.0] * 7, self.A), kind="grasp")
        moved = [0.0, 0.5, 0.0, *self.A_NEG[3:]]
        rec = r.record_path(FakePath(moved, [1.0] * 7), kind="grasp")
        assert rec["seam_error"] == pytest.approx(0.5)
        assert r.seam_violations == 1


class TestResume:
    """Property 4 -- a resumed run continues the manifest."""

    def test_segments_of_completed_steps_survive_and_numbering_continues(
        self, tmp_path
    ):
        first = _rec(tmp_path)
        first.begin_step(0, "step0")
        first.record_path(FakePath([0.0], [1.0]), kind="grasp")
        first.begin_step(1, "step1")
        first.record_path(FakePath([1.0], [2.0]), kind="transit")

        second = _rec(tmp_path)
        assert second.resume(last_completed_step=1) == 2
        second.begin_step(2, "step2")
        rec = second.record_path(FakePath([2.0], [3.0]), kind="grasp")

        assert rec["index"] == 2
        assert [s["index"] for s in _manifest(second)["segments"]] == [0, 1, 2]

    def test_segments_past_the_checkpoint_are_dropped(self, tmp_path):
        """Step 2 was interrupted, so the run replans it -- its old
        segments describe motion that is about to be replaced."""
        first = _rec(tmp_path)
        first.begin_step(0, "step0")
        first.record_path(FakePath([0.0], [1.0]), kind="grasp")
        first.begin_step(2, "step2")
        first.record_path(FakePath([1.0], [2.0]), kind="grasp")

        second = _rec(tmp_path)
        assert second.resume(last_completed_step=0) == 1
        assert [s["step_index"] for s in _manifest(second)["segments"]] == [0]

    def test_the_kept_tail_is_the_seam_reference(self, tmp_path):
        """The first segment after a resume is checked against where the
        checkpoint actually left the robot, not against nothing."""
        first = _rec(tmp_path)
        first.begin_step(0, "step0")
        first.record_path(FakePath([0.0], [1.0]), kind="grasp")

        second = _rec(tmp_path)
        second.resume(last_completed_step=0)
        second.begin_step(1, "step1")
        rec = second.record_path(FakePath([1.25], [2.0]), kind="transit")
        assert rec["seam_error"] == pytest.approx(0.25)

    def test_resume_without_a_manifest_starts_fresh(self, tmp_path):
        r = _rec(tmp_path)
        assert r.resume(last_completed_step=5) == 0


class TestSampling:
    """The captured artifact itself -- configurations, not graph refs."""

    def test_sample_count_follows_path_length_and_dt(self, tmp_path):
        r = _rec(tmp_path, dt=0.1)
        r.begin_step(0, "s")
        short = r.record_path(FakePath([0.0], [1.0], length=1.0), kind="grasp")
        long_ = r.record_path(FakePath([1.0], [2.0], length=5.0), kind="grasp")
        assert short["num_waypoints"] == 11
        assert long_["num_waypoints"] == 51

    def test_sample_count_is_clamped(self, tmp_path):
        r = _rec(tmp_path, dt=1e-6, max_samples=50)
        r.begin_step(0, "s")
        assert (
            r.record_path(FakePath([0.0], [1.0]), kind="grasp")["num_waypoints"] == 50
        )

    def test_waypoint_file_is_self_contained(self, tmp_path):
        """Playback must not need the manifest to interpret one segment."""
        r = _rec(tmp_path, dt=0.5)
        r.begin_step(3, "RS1 A")
        rec = r.record_path(
            FakePath([0.0, 5.0], [1.0, 6.0]), kind="grasp", edge_name="e"
        )
        with open(os.path.join(r.output_dir, rec["waypoint_file"])) as f:
            data = json.load(f)
        assert data["waypoints"][0] == [0.0, 5.0]
        assert data["waypoints"][-1] == [1.0, 6.0]
        assert data["step_label"] == "RS1 A"
        assert data["edge_name"] == "e"
        assert data["length"] == 1.0

    def test_endpoints_come_from_the_path_not_the_samples(self, tmp_path):
        """A failed sample at t=0 must not silently redefine where the
        segment starts -- the seam check depends on this."""
        r = _rec(tmp_path, dt=0.5)
        r.begin_step(0, "s")
        rec = r.record_path(
            FakePath([0.0], [1.0], failing_samples=(0.0,)), kind="grasp"
        )
        assert rec["q_start"] == [0.0]
        assert rec["num_waypoints"] == 2  # 3 requested, one refused

    def test_none_path_is_ignored(self, tmp_path):
        """Skipped phases append a None placeholder to phase_results."""
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        assert r.record_path(None, kind="grasp") is None
        assert _manifest(r)["segments"] == []

    def test_an_unsamplable_path_is_skipped_not_fatal(self, tmp_path):
        class Broken:
            def length(self):
                raise RuntimeError("no length")

        r = _rec(tmp_path)
        r.begin_step(0, "s")
        assert r.record_path(Broken(), kind="grasp") is None
        assert _manifest(r)["segments"] == []


class TestPhaseResults:
    """Draining a finished block, in the shape GraspSequencePlanner leaves."""

    def test_complete_phases_are_recorded_in_phase_then_edge_order(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "RS1 A")
        phase_results = [
            {
                "phase": 1,
                "gripper": "g_fg",
                "handle": "RS1/h_RS1_FG",
                "edges": ["e_01", "e_12"],
                "complete": True,
                "paths": [FakePath([0.0], [1.0]), FakePath([1.0], [2.0])],
                "state_after": "S1",
            },
            {
                "phase": 2,
                "gripper": "g_wb",
                "handle": "RS1/h_RS1_WB",
                "edges": ["e_23"],
                "complete": True,
                "paths": [FakePath([2.0], [3.0])],
                "state_after": "S2",
            },
        ]
        written = r.record_phase_results(phase_results, block_label="RS1 A")
        assert [w["edge_name"] for w in written] == ["e_01", "e_12", "e_23"]
        assert [w["phase"] for w in written] == [1, 1, 2]
        assert all(w["kind"] == "grasp" for w in written)
        assert all(w["block_label"] == "RS1 A" for w in written)

    def test_release_phases_are_tagged_release(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "RS1 B")
        written = r.record_phase_results(
            [
                {
                    "phase": 1,
                    "gripper": "g_fg",
                    "handle": None,
                    "released": "RS1/h_RS1_FG",
                    "edges": ["e_21", "e_10"],
                    "complete": True,
                    "paths": [FakePath([0.0], [1.0]), FakePath([1.0], [2.0])],
                }
            ]
        )
        assert [w["kind"] for w in written] == ["release", "release"]
        assert written[0]["released"] == "RS1/h_RS1_FG"

    def test_a_direct_release_fallback_has_fewer_paths_than_edge_names(self, tmp_path):
        """_build_release_phase_info filters None paths out of "paths" but
        keeps both names in "edges", so the two lists can disagree."""
        r = _rec(tmp_path)
        r.begin_step(0, "RS3 B")
        written = r.record_phase_results(
            [
                {
                    "phase": 1,
                    "gripper": "g_fg",
                    "handle": None,
                    "edges": ["e_21", "e_10"],
                    "complete": True,
                    "paths": [FakePath([0.0], [1.0])],
                }
            ]
        )
        assert len(written) == 1
        assert written[0]["edge_name"] == "e_21"

    def test_skipped_phases_are_not_recorded(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        written = r.record_phase_results(
            [
                {
                    "phase": 2,
                    "gripper": "g",
                    "handle": None,
                    "edges": [],
                    "paths": [],
                    "complete": True,
                    "skipped": True,
                },
            ]
        )
        assert written == []
        assert _manifest(r)["segments"] == []


class TestAccessorShapes:
    """The recorder must not care which spelling a backend uses."""

    def test_pyhpp_shape_eval_initial_end(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        rec = r.record_path(FakePath([0.0], [1.0]), kind="grasp")
        assert (rec["q_start"], rec["q_end"]) == ([0.0], [1.0])

    def test_corba_shape_call_and_getconfig(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        rec = r.record_path(FakeCorbaPath([0.0], [1.0]), kind="grasp")
        assert (rec["q_start"], rec["q_end"]) == ([0.0], [1.0])

    def test_callable_path(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        rec = r.record_path(FakeCallablePath([0.0], [1.0]), kind="grasp")
        assert rec["num_waypoints"] > 1

    def test_endpoints_fall_back_to_the_samples(self, tmp_path):
        """A path offering neither endpoint accessor still gets a seam
        reference -- from its own first and last sample."""

        class Bare(FakePath):
            initial = None
            end = None

        r = _rec(tmp_path)
        r.begin_step(0, "s")
        rec = r.record_path(Bare([0.0], [1.0]), kind="grasp")
        assert (rec["q_start"], rec["q_end"]) == ([0.0], [1.0])


class TestStoredPathIds:
    """What ``plan_transition_edge`` actually hands back.

    With ``store=True`` (its default, and what every call site here uses)
    it returns the *index* of the path in the backend's store, not the
    path -- so ``phase_results["paths"]`` is a list of ints. A recorder
    that assumed objects captured nothing: the first live run failed every
    segment with ``'int' object has no attribute 'length'``.
    """

    class Planner:
        def __init__(self, paths):
            self._paths = paths

        def get_path(self, index=0):
            return self._paths[index]

    def test_an_int_is_resolved_through_the_planner(self, tmp_path):
        path = FakePath([0.0], [1.0])
        r = _rec(tmp_path, planner=self.Planner([path]))
        r.begin_step(0, "s")
        rec = r.record_path(0, kind="grasp")
        assert rec is not None
        assert rec["q_start"] == [0.0]
        assert rec["q_end"] == [1.0]

    def test_ids_resolve_in_phase_results_too(self, tmp_path):
        paths = [FakePath([0.0], [1.0]), FakePath([1.0], [2.0])]
        r = _rec(tmp_path, planner=self.Planner(paths))
        r.begin_step(0, "s")
        written = r.record_phase_results(
            [
                {
                    "phase": 1,
                    "gripper": "g",
                    "handle": "h",
                    "complete": True,
                    "edges": ["e0", "e1"],
                    "paths": [0, 1],
                }
            ]
        )
        assert [w["q_end"] for w in written] == [[1.0], [2.0]]

    def test_an_unresolvable_id_is_skipped_not_fatal(self, tmp_path):
        r = _rec(tmp_path, planner=self.Planner([]))
        r.begin_step(0, "s")
        assert r.record_path(7, kind="grasp") is None
        assert _manifest(r)["segments"] == []

    def test_path_objects_still_work_without_a_planner(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        assert r.record_path(FakePath([0.0], [1.0]), kind="grasp") is not None


class TestNativeSidecar:
    """Best-effort only -- the sampled JSON is the replay format."""

    def test_native_file_is_written_when_the_backend_can(self, tmp_path):
        class Planner:
            def save_path_vector(self, path, filename):
                with open(filename, "w") as f:
                    f.write("native")

        r = _rec(tmp_path, planner=Planner())
        r.begin_step(0, "s")
        rec = r.record_path(FakePath([0.0], [1.0]), kind="grasp")
        assert rec["path_file"].endswith(".path")
        assert os.path.exists(os.path.join(r.output_dir, rec["path_file"]))

    def test_a_backend_refusal_leaves_the_segment_intact(self, tmp_path):
        """Time-parameterized paths have no serializer; that is expected,
        not a capture failure."""

        class Planner:
            def save_path_vector(self, path, filename):
                raise RuntimeError("time parameterization not serializable")

        r = _rec(tmp_path, planner=Planner())
        r.begin_step(0, "s")
        rec = r.record_path(FakePath([0.0], [1.0]), kind="grasp")
        assert rec["path_file"] is None
        assert os.path.exists(os.path.join(r.output_dir, rec["waypoint_file"]))

    def test_geometric_twin_is_preferred_for_the_native_file(self, tmp_path):
        saved = {}

        class Planner:
            def save_path_vector(self, path, filename):
                saved["path"] = path
                open(filename, "w").close()

        r = _rec(tmp_path, planner=Planner())
        r.begin_step(0, "s")
        timed = FakePath([0.0], [1.0])
        geometric = FakePath([0.0], [1.0])
        r.record_path(timed, kind="grasp", geometric_path=geometric)
        assert saved["path"] is geometric


class TestResumedPhases:
    """The trajectory includes attempts that failed part-way.

    When a phase fails mid-way, resume_sequence restarts it from *where
    the failure left the robot*, not from the block's entry
    configuration -- so the edges the failed attempt did plan are what
    carried the robot there. Dropping them puts a teleport in the
    manifest.

    Found live on RS2's FG grasp: `_01` planned home -> pregrasp-A, `_12`
    hit a collision, and the resume replanned `_01` from pregrasp-A to
    pregrasp-B. Keeping only the surviving pair left a 5.08 rad jump.

    resume_sequence also *deletes* incomplete phases from phase_results
    before replanning, so this only works if the caller drains after every
    planner call -- hence the repeated record_phase_results() calls below.
    """

    def _rs2(self, tmp_path):
        """The RS2 sequence, in the shape the planner produces it."""
        r = _rec(tmp_path)
        r.begin_step(0, "RS2 A0")
        home, pre_a, pre_b, grasp = [0.0], [1.0], [1.4], [2.0]

        # plan_sequence: _01 lands, _12 fails -> partial entry
        partial = {
            "phase": 1, "gripper": "g_fg", "handle": "RS2/h_RS2_FG",
            "edges": ["e_01", "e_12"], "complete": False,
            "paths": [FakePath(home, pre_a)],
        }
        r.record_phase_results([partial], block_label="RS2 A0")

        # resume_sequence drops the partial and replans from pre_a
        complete = {
            "phase": 1, "gripper": "g_fg", "handle": "RS2/h_RS2_FG",
            "edges": ["e_01", "e_12"], "complete": True,
            "paths": [FakePath(pre_a, pre_b), FakePath(pre_b, grasp)],
        }
        r.record_phase_results([complete], block_label="RS2 A0")
        return r

    def test_the_abandoned_leg_is_kept_and_the_manifest_is_continuous(
        self, tmp_path
    ):
        r = self._rs2(tmp_path)
        segs = _manifest(r)["segments"]
        assert len(segs) == 3, "the failed attempt's _01 must survive"
        assert [s["q_start"] for s in segs] == [[0.0], [1.0], [1.4]]
        assert r.seam_violations == 0

    def test_dropping_it_is_what_produced_the_jump(self, tmp_path):
        """The old behaviour, reconstructed: skip the partial and the
        manifest reports exactly the discontinuity seen live."""
        r = _rec(tmp_path)
        r.begin_step(0, "RS2 A0")
        r.record_path(FakePath([0.0], [0.0]), kind="transit")  # arrives home
        r.record_phase_results([
            {"phase": 1, "gripper": "g", "handle": "h",
             "edges": ["e_01", "e_12"], "complete": True,
             "paths": [FakePath([1.0], [1.4]), FakePath([1.4], [2.0])]},
        ])
        assert r.seam_violations == 1
        assert _manifest(r)["segments"][1]["seam_error"] == pytest.approx(1.0)

    def test_the_phase_records_whether_it_was_complete(self, tmp_path):
        r = self._rs2(tmp_path)
        segs = _manifest(r)["segments"]
        assert [s["phase_complete"] for s in segs] == [False, True, True]

    def test_draining_repeatedly_does_not_duplicate(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        phase = {
            "phase": 1, "gripper": "g", "handle": "h", "complete": True,
            "edges": ["e_01"], "paths": [FakePath([0.0], [1.0])],
        }
        for _ in range(4):
            r.record_phase_results([phase])
        assert len(_manifest(r)["segments"]) == 1

    def test_a_phase_that_grows_between_drains_records_only_the_new_edge(
        self, tmp_path
    ):
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        phase = {
            "phase": 1, "gripper": "g", "handle": "h", "complete": False,
            "edges": ["e_01", "e_12"], "paths": [FakePath([0.0], [1.0])],
        }
        assert len(r.record_phase_results([phase])) == 1
        phase["paths"].append(FakePath([1.0], [2.0]))
        phase["complete"] = True
        assert len(r.record_phase_results([phase])) == 1
        assert len(_manifest(r)["segments"]) == 2


class TestRollback:
    """A replanned *block* restarts from its entry config, so the attempt
    it abandoned never happened -- the opposite of a resumed phase."""

    def test_segments_after_the_mark_are_dropped(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        r.record_path(FakePath([0.0], [1.0]), kind="transit")
        mark = r.mark()
        r.record_path(FakePath([1.0], [2.0]), kind="grasp")
        r.record_path(FakePath([2.0], [3.0]), kind="grasp")

        assert r.rollback(mark) == 2
        assert len(_manifest(r)["segments"]) == 1

    def test_the_seam_reference_goes_back_with_it(self, tmp_path):
        """The replanned block starts from the entry config again, so the
        next segment must be checked against that, not against where the
        abandoned attempt ended."""
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        r.record_path(FakePath([0.0], [1.0]), kind="transit")
        mark = r.mark()
        r.record_path(FakePath([1.0], [9.0]), kind="grasp")
        r.rollback(mark)

        rec = r.record_path(FakePath([1.0], [2.0]), kind="grasp")
        assert rec["seam_error"] == pytest.approx(0.0)
        assert r.seam_violations == 0

    def test_numbering_is_reused(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        mark = r.mark()
        r.record_path(FakePath([0.0], [9.0]), kind="grasp")
        r.rollback(mark)
        rec = r.record_path(FakePath([0.0], [1.0]), kind="grasp")
        assert rec["index"] == 0
        assert len(_manifest(r)["segments"]) == 1

    def test_rolling_back_to_the_current_mark_is_a_no_op(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        r.record_path(FakePath([0.0], [1.0]), kind="grasp")
        assert r.rollback(r.mark()) == 0
        assert len(_manifest(r)["segments"]) == 1

    def test_a_rolled_back_violation_stops_being_counted(self, tmp_path):
        r = _rec(tmp_path)
        r.begin_step(0, "s")
        r.record_path(FakePath([0.0], [1.0]), kind="grasp")
        mark = r.mark()
        r.record_path(FakePath([5.0], [6.0]), kind="grasp")
        assert r.seam_violations == 1
        r.rollback(mark)
        assert r.seam_violations == 0
        assert _manifest(r)["seam_violations"] == 0
