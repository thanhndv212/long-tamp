"""Unit tests for ``run_sequence()``, the external capability-driven orchestrator.

Exercises the sequencing *policy* (when to call ``release()`` before
``grasp()``, when to stop on failure) against a lightweight fake planner
that records calls -- mirroring ``tests/test_grasp_release_capabilities.py``'s
approach of stubbing at the capability boundary rather than wiring real HPP.
``grasp()``/``release()`` themselves are already covered there and against a
real scene in ``tests/test_grasp_release_use_case_twin.py``; this file is
only about the control flow ``run_sequence()`` adds on top of them.
"""

import pytest

from long_tamp.tasks.sequence_orchestrator import run_sequence


class _FakeGraspTracker:
    def __init__(self, current_grasps):
        self.current_grasps = dict(current_grasps)


class _FakePlanner:
    """Records every grasp()/release()/find_feasible_phase_target() call, in
    order, and returns caller-scripted results keyed by (kind, gripper,
    handle)."""

    def __init__(self, current_grasps, responses, lookahead_responses=None):
        self.grasp_tracker = _FakeGraspTracker(current_grasps)
        self._responses = responses
        self._lookahead_responses = lookahead_responses or {}
        self.calls: list[tuple[str, str, str | None]] = []
        # (kind, gripper, handle) -> kwargs run_sequence() passed through,
        # for tests asserting on frozen-arms/q_hint plumbing specifically.
        self.call_kwargs: dict[tuple[str, str, str | None], dict] = {}

    def grasp(self, gripper, handle, q_current, **kwargs):
        self.calls.append(("grasp", gripper, handle))
        self.call_kwargs[("grasp", gripper, handle)] = kwargs
        result = self._responses[("grasp", gripper, handle)]
        if result["success"]:
            self.grasp_tracker.current_grasps[gripper] = handle
        return result

    def release(self, gripper, q_current, **kwargs):
        self.calls.append(("release", gripper, None))
        self.call_kwargs[("release", gripper, None)] = kwargs
        result = self._responses[("release", gripper, None)]
        if result["success"]:
            self.grasp_tracker.current_grasps[gripper] = None
        return result

    def find_feasible_phase_target(self, phase_n, phase_n1, **kwargs):
        self.calls.append(("lookahead", phase_n[0], phase_n[1]))
        self.call_kwargs[("lookahead", phase_n[0], phase_n[1])] = {
            "phase_n1": phase_n1,
            **kwargs,
        }
        return self._lookahead_responses.get((phase_n, phase_n1))


def _ok(final_config, phase_results=None):
    return {
        "success": True,
        "message": "ok",
        "phase_results": phase_results if phase_results is not None else [{"ok": True}],
        "final_config": final_config,
    }


def _fail(message):
    return {
        "success": False,
        "message": message,
        "phase_results": [],
        "final_config": None,
    }


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


class TestPerPhaseFrozenArms:
    def test_frozen_arms_remapped_to_phase_idx_zero_per_call(self):
        # per_phase_frozen_arms is keyed by run_sequence()'s own phase_idx,
        # but grasp()/release() each build their own graph at their
        # internal phase_idx=0 -- run_sequence() must remap.
        planner = _FakePlanner(
            current_grasps={"g1": None, "g2": None},
            responses={
                ("grasp", "g1", "h1"): _ok([1.0]),
                ("grasp", "g2", "h2"): _ok([2.0]),
            },
        )

        run_sequence(
            planner,
            [("g1", "h1"), ("g2", "h2")],
            q_init=[0.0],
            verbose=False,
            per_phase_frozen_arms={0: [], 1: ["g1"]},
        )

        assert (
            planner.call_kwargs[("grasp", "g1", "h1")]["frozen_arms_mode"] == "manual"
        )
        assert planner.call_kwargs[("grasp", "g1", "h1")]["per_phase_frozen_arms"] == {
            0: []
        }
        assert planner.call_kwargs[("grasp", "g2", "h2")]["per_phase_frozen_arms"] == {
            0: ["g1"]
        }

    def test_omitted_per_phase_frozen_arms_leaves_grasp_on_its_own_default(self):
        planner = _FakePlanner(
            current_grasps={"g1": None},
            responses={("grasp", "g1", "h1"): _ok([1.0])},
        )

        run_sequence(planner, [("g1", "h1")], q_init=[0.0], verbose=False)

        assert planner.call_kwargs[("grasp", "g1", "h1")]["frozen_arms_mode"] == "auto"
        assert (
            planner.call_kwargs[("grasp", "g1", "h1")]["per_phase_frozen_arms"] is None
        )


class TestLookahead:
    def test_lookahead_probes_next_phase_and_threads_the_candidate_as_q_hint(self):
        planner = _FakePlanner(
            current_grasps={"g1": None},
            responses={
                ("grasp", "g1", "h1"): _ok([1.0]),
                ("release", "g1", None): _ok([1.5]),
                ("grasp", "g1", "h2"): _ok([2.0]),
            },
            lookahead_responses={(("g1", "h1"), ("g1", "h2")): [[9.0], [9.5]]},
        )

        run_sequence(
            planner,
            [("g1", "h1"), ("g1", "h2")],
            q_init=[0.0],
            verbose=False,
            lookahead_pairs=[0],
        )

        assert planner.calls[0] == ("lookahead", "g1", "h1")
        assert planner.call_kwargs[("lookahead", "g1", "h1")]["phase_n1"] == (
            "g1",
            "h2",
        )
        assert planner.calls[1] == ("grasp", "g1", "h1")
        assert planner.call_kwargs[("grasp", "g1", "h1")]["q_hint"] == [[9.0], [9.5]]

    def test_no_candidate_found_falls_back_to_unprotected_grasp(self):
        planner = _FakePlanner(
            current_grasps={"g1": None},
            responses={
                ("grasp", "g1", "h1"): _ok([1.0]),
                ("release", "g1", None): _ok([1.5]),
                ("grasp", "g1", "h2"): _ok([2.0]),
            },
            lookahead_responses={(("g1", "h1"), ("g1", "h2")): None},
        )

        result = run_sequence(
            planner,
            [("g1", "h1"), ("g1", "h2")],
            q_init=[0.0],
            verbose=False,
            lookahead_pairs=[0],
        )

        assert planner.call_kwargs[("grasp", "g1", "h1")]["q_hint"] is None
        assert result["success"] is True

    def test_lookahead_pairs_rejects_a_release_as_the_next_phase(self):
        planner = _FakePlanner(current_grasps={"g1": None}, responses={})

        with pytest.raises(ValueError, match="find_feasible_phase_target"):
            run_sequence(
                planner,
                [("g1", "h1"), ("g1", None)],
                q_init=[0.0],
                verbose=False,
                lookahead_pairs=[0],
            )
