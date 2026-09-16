"""Unit tests for ``run_sequence()``, the external capability-driven orchestrator.

Exercises the sequencing *policy* (when to call ``release()`` before
``grasp()``, when to stop on failure) against a lightweight fake planner
that records calls -- mirroring ``tests/test_grasp_release_capabilities.py``'s
approach of stubbing at the capability boundary rather than wiring real HPP.
``grasp()``/``release()`` themselves are already covered there and against a
real scene in ``tests/test_grasp_release_use_case_twin.py``; this file is
only about the control flow ``run_sequence()`` adds on top of them.
"""

from long_tamp.tasks.sequence_orchestrator import run_sequence


class _FakeGraspTracker:
    def __init__(self, current_grasps):
        self.current_grasps = dict(current_grasps)


class _FakePlanner:
    """Records every grasp()/release() call, in order, and returns
    caller-scripted results keyed by (kind, gripper, handle)."""

    def __init__(self, current_grasps, responses):
        self.grasp_tracker = _FakeGraspTracker(current_grasps)
        self._responses = responses
        self.calls: list[tuple[str, str, str | None]] = []

    def grasp(self, gripper, handle, q_current):
        self.calls.append(("grasp", gripper, handle))
        result = self._responses[("grasp", gripper, handle)]
        if result["success"]:
            self.grasp_tracker.current_grasps[gripper] = handle
        return result

    def release(self, gripper, q_current):
        self.calls.append(("release", gripper, None))
        result = self._responses[("release", gripper, None)]
        if result["success"]:
            self.grasp_tracker.current_grasps[gripper] = None
        return result


def _ok(final_config, phase_results=None):
    return {
        "success": True,
        "message": "ok",
        "phase_results": phase_results if phase_results is not None else [{"ok": True}],
        "final_config": final_config,
    }


def _fail(message):
    return {"success": False, "message": message, "phase_results": [], "final_config": None}


class TestNoAutoReleaseNeeded:
    def test_two_grasps_in_order_no_release_calls(self):
        planner = _FakePlanner(
            current_grasps={"g1": None, "g2": None},
            responses={
                ("grasp", "g1", "h1"): _ok([1.0]),
                ("grasp", "g2", "h2"): _ok([2.0]),
            },
        )

        result = run_sequence(
            planner, [("g1", "h1"), ("g2", "h2")], q_init=[0.0], verbose=False
        )

        assert result["success"] is True
        assert planner.calls == [("grasp", "g1", "h1"), ("grasp", "g2", "h2")]
        assert result["final_config"] == [2.0]
        assert result["completed_phases"] == 2
        assert result["failed_phase_idx"] is None
        assert len(result["phase_results"]) == 2


class TestAutoReleasePolicy:
    def test_release_called_before_grasp_when_gripper_holds_something_else(self):
        planner = _FakePlanner(
            current_grasps={"g1": "h_old"},
            responses={
                ("release", "g1", None): _ok([1.0]),
                ("grasp", "g1", "h_new"): _ok([2.0]),
            },
        )

        result = run_sequence(planner, [("g1", "h_new")], q_init=[0.0], verbose=False)

        assert result["success"] is True
        # Order matters: release must precede grasp.
        assert planner.calls == [("release", "g1", None), ("grasp", "g1", "h_new")]

    def test_no_release_when_gripper_already_free(self):
        planner = _FakePlanner(
            current_grasps={"g1": None},
            responses={("grasp", "g1", "h1"): _ok([1.0])},
        )

        run_sequence(planner, [("g1", "h1")], q_init=[0.0], verbose=False)

        assert planner.calls == [("grasp", "g1", "h1")]

    def test_no_release_when_gripper_already_holds_the_target_handle(self):
        # grasp()'s own idempotent no-op handles this -- the orchestrator
        # must not call release() when currently_held == handle.
        planner = _FakePlanner(
            current_grasps={"g1": "h1"},
            responses={("grasp", "g1", "h1"): _ok([1.0], phase_results=[])},
        )

        result = run_sequence(planner, [("g1", "h1")], q_init=[0.0], verbose=False)

        assert planner.calls == [("grasp", "g1", "h1")]
        assert result["success"] is True

    def test_explicit_release_entry_calls_release_not_grasp(self):
        planner = _FakePlanner(
            current_grasps={"g1": "h1"},
            responses={("release", "g1", None): _ok([1.0])},
        )

        result = run_sequence(planner, [("g1", None)], q_init=[0.0], verbose=False)

        assert planner.calls == [("release", "g1", None)]
        assert result["success"] is True


class TestFailureStopsTheSequence:
    def test_grasp_failure_stops_before_later_phases(self):
        planner = _FakePlanner(
            current_grasps={"g1": None, "g2": None},
            responses={("grasp", "g1", "h1"): _fail("boom")},
        )

        result = run_sequence(
            planner, [("g1", "h1"), ("g2", "h2")], q_init=[0.0], verbose=False
        )

        assert result["success"] is False
        assert "boom" in result["message"]
        assert planner.calls == [("grasp", "g1", "h1")]  # phase 1 never attempted
        assert result["failed_phase_idx"] == 0
        assert result["completed_phases"] == 0

    def test_auto_release_failure_stops_before_the_grasp_it_was_guarding(self):
        planner = _FakePlanner(
            current_grasps={"g1": "h_old"},
            responses={("release", "g1", None): _fail("release boom")},
        )

        result = run_sequence(planner, [("g1", "h_new")], q_init=[0.0], verbose=False)

        assert result["success"] is False
        assert "release boom" in result["message"]
        # grasp() must never be reached if the guarding release failed.
        assert planner.calls == [("release", "g1", None)]

    def test_explicit_release_failure_is_reported_with_its_phase_index(self):
        planner = _FakePlanner(
            current_grasps={"g1": "h1", "g2": None},
            responses={
                ("release", "g1", None): _ok([1.0]),
                ("grasp", "g2", "h2"): _fail("boom"),
            },
        )

        result = run_sequence(
            planner, [("g1", None), ("g2", "h2")], q_init=[0.0], verbose=False
        )

        assert result["success"] is False
        assert result["failed_phase_idx"] == 1
        assert result["completed_phases"] == 1


class TestPhaseResultsConcatenation:
    def test_phase_results_from_every_call_are_concatenated_in_order(self):
        planner = _FakePlanner(
            current_grasps={"g1": "h_old"},
            responses={
                ("release", "g1", None): _ok([1.0], phase_results=[{"tag": "release"}]),
                ("grasp", "g1", "h_new"): _ok([2.0], phase_results=[{"tag": "grasp"}]),
            },
        )

        result = run_sequence(planner, [("g1", "h_new")], q_init=[0.0], verbose=False)

        assert [pr["tag"] for pr in result["phase_results"]] == ["release", "grasp"]
