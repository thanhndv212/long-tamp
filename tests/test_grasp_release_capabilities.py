"""Unit tests for ``GraspSequencePlanner.grasp()`` / ``.release()``.

These are the standalone, capability-shaped primitives extracted from the
per-phase logic ``plan_sequence()`` drives internally via ``_run_phase_loop``
-- see ARCHITECTURE.md's task-orchestration section. Unlike a
``plan_sequence()`` phase, neither method makes an orchestration decision
(no auto-release insertion, no multi-phase bookkeeping): each is precondition-
checked and fails loudly rather than silently fixing up state, so an external
orchestrator (a BT node, a ``task_planning/`` capability, a PDDL executor) can
compose them itself.

Following this file's own established convention (test_resume_start_config.py,
test_grasp_sequence_phase_q_hints.py): stub the shared per-phase helpers
(``_build_phase_graph_and_constraints``, ``_compute_and_project_edge_sequence``,
``_plan_phase_edges``, ``_finalize_phase_result``, ``_release_frozen_arms``,
``_plan_release_subphase``) rather than wiring a real HPP graph/backend --
those helpers already have their own coverage elsewhere. These tests only
cover the new glue: precondition checks, exception-to-dict conversion, and
the returned shape.

Importing long_tamp requires pyhpp, so these run in the hpp-arm64 container
(same convention as the other grasp_sequence tests).
"""

from unittest.mock import patch

from long_tamp.planning.grasp_state import GraspStateTracker
from long_tamp.tasks.grasp_sequence import GraspSequencePlanner


def _make_planner(current_grasps=None):
    """Bare GraspSequencePlanner with just enough state for grasp()/release().

    Bypasses __init__ (no real graph_builder/config_gen/planner needed --
    the tests stub every helper that would touch them).
    """
    planner = object.__new__(GraspSequencePlanner)
    planner.grasp_tracker = GraspStateTracker(
        grippers=["g1", "g2"], handles=["h1", "h2"], initial_grasps=current_grasps
    )
    planner.phase_results = []
    planner.graph_constraints = None
    planner.planner = object()  # no configure_transition_planner attr by default
    return planner


class TestGrasp:
    def test_already_holds_handle_is_noop(self):
        planner = _make_planner({"g1": "h1"})
        result = planner.grasp("g1", "h1", q_current=[0.0])
        assert result == {
            "success": True,
            "message": "'g1' already holds 'h1'",
            "phase_results": [],
            "final_config": [0.0],
            "skipped": True,
        }

    def test_holds_different_object_fails_without_planning(self):
        planner = _make_planner({"g1": "h2"})
        planner._build_phase_graph_and_constraints = lambda **kw: (_ for _ in ()).throw(
            AssertionError("must not attempt to plan")
        )
        result = planner.grasp("g1", "h1", q_current=[0.0])
        assert result["success"] is False
        assert "holds 'h2'" in result["message"]
        assert "release('g1'" in result["message"]
        assert result["phase_results"] == []
        assert result["final_config"] == [0.0]

    def test_success_path_updates_state_and_returns_phase_result(self):
        planner = _make_planner({"g1": None})
        calls = []

        def _build(**kw):
            calls.append(("build", kw["phase_idx"], kw["gripper"], kw["handle"]))

        def _compute(**kw):
            calls.append(("compute", kw["phase_idx"]))
            return ["edge01", "edge10"], [1.0]

        def _plan_edges(**kw):
            calls.append(("plan_edges", kw["phase_idx"], kw["edge_sequence"]))
            return {
                "phase_paths": ["path01", "path10"],
                "phase_geometric_paths": [],
                "edge_stats_list": [],
                "q_start": [2.0],
                "q_pregrasp_for_cache": [1.5],
            }

        def _finalize(**kw):
            calls.append(("finalize", kw["phase_idx"]))
            planner.grasp_tracker.update_grasp("g1", "h1")
            result = {"phase": 1, "gripper": "g1", "handle": "h1", "complete": True}
            planner.phase_results.append(result)
            return kw["q_start"]

        planner._build_phase_graph_and_constraints = _build
        planner._compute_and_project_edge_sequence = _compute
        planner._plan_phase_edges = _plan_edges
        planner._finalize_phase_result = _finalize

        result = planner.grasp("g1", "h1", q_current=[0.0])

        assert [c[0] for c in calls] == ["build", "compute", "plan_edges", "finalize"]
        assert all(c[1] == 0 for c in calls)  # phase_idx=0 everywhere
        assert result["success"] is True
        assert result["final_config"] == [2.0]
        assert result["phase_results"] == [
            {"phase": 1, "gripper": "g1", "handle": "h1", "complete": True}
        ]
        assert planner.grasp_tracker.current_grasps["g1"] == "h1"

    def test_build_graph_failure_returns_failure_dict(self):
        planner = _make_planner({"g1": None})
        planner._build_phase_graph_and_constraints = lambda **kw: (_ for _ in ()).throw(
            RuntimeError("graph boom")
        )
        result = planner.grasp("g1", "h1", q_current=[0.0])
        assert result["success"] is False
        assert "graph boom" in result["message"]
        assert result["phase_results"] == []
        assert result["final_config"] == [0.0]
        # Precondition passed (gripper was free) but planning failed --
        # grasp_tracker must NOT have been updated.
        assert planner.grasp_tracker.current_grasps["g1"] is None

    def test_edge_planning_failure_returns_failure_dict(self):
        planner = _make_planner({"g1": None})
        planner._build_phase_graph_and_constraints = lambda **kw: None
        planner._compute_and_project_edge_sequence = lambda **kw: (["edge01"], [0.0])
        planner._plan_phase_edges = lambda **kw: (_ for _ in ()).throw(
            RuntimeError("collision")
        )
        result = planner.grasp("g1", "h1", q_current=[0.0])
        assert result["success"] is False
        assert "collision" in result["message"]
        assert planner.grasp_tracker.current_grasps["g1"] is None

    def test_forwards_transition_planner_config_when_supported(self):
        planner = _make_planner({"g1": None})
        configured = []
        planner.planner = type(
            "P",
            (),
            {
                "configure_transition_planner": lambda self, **kw: configured.append(
                    kw
                )
            },
        )()
        planner._build_phase_graph_and_constraints = lambda **kw: (_ for _ in ()).throw(
            RuntimeError("stop here")
        )
        planner.grasp(
            "g1", "h1", q_current=[0.0], timeout_per_edge=12.0,
            max_iterations_per_edge=99,
        )
        assert configured == [{"time_out": 12.0, "max_iterations": 99}]


class TestRelease:
    def test_already_free_is_noop(self):
        planner = _make_planner({"g1": None})
        result = planner.release("g1", q_current=[0.0])
        assert result == {
            "success": True,
            "message": "'g1' already free",
            "phase_results": [],
            "final_config": [0.0],
            "skipped": True,
        }

    def test_success_path_appends_phase_result(self):
        planner = _make_planner({"g1": "h1"})
        planner._release_frozen_arms = lambda *a, **kw: []

        def _subphase(**kw):
            assert kw["gripper"] == "g1"
            assert kw["phase_graph_constraints"] is None
            return [3.0], {"paths": ["p21", "p10"], "edges": ["e21", "e10"]}

        planner._plan_release_subphase = _subphase

        result = planner.release("g1", q_current=[0.0])

        assert result["success"] is True
        assert result["final_config"] == [3.0]
        assert result["message"] == "Released 'h1' from 'g1'"
        assert planner.phase_results == [
            {
                "phase": 1,
                "gripper": "g1",
                "handle": None,
                "released": "h1",
                "paths": ["p21", "p10"],
                "edges": ["e21", "e10"],
                "complete": True,
            }
        ]

    def test_subphase_failure_returns_failure_dict(self):
        planner = _make_planner({"g1": "h1"})
        planner._release_frozen_arms = lambda *a, **kw: []
        planner._plan_release_subphase = lambda **kw: (_ for _ in ()).throw(
            RuntimeError("no path")
        )
        result = planner.release("g1", q_current=[0.0])
        assert result["success"] is False
        assert "no path" in result["message"]
        assert result["phase_results"] == []
        assert planner.phase_results == []

    def test_frozen_arms_mode_none_skips_frozen_arm_computation(self):
        planner = _make_planner({"g1": "h1"})
        planner._release_frozen_arms = lambda *a, **kw: (_ for _ in ()).throw(
            AssertionError("must not be called when mode is 'none'")
        )
        planner._plan_release_subphase = lambda **kw: (
            [1.0],
            {"paths": [], "edges": []},
        )
        result = planner.release("g1", q_current=[0.0], frozen_arms_mode="none")
        assert result["success"] is True

    def test_frozen_arms_mode_global_uses_graph_constraints(self):
        planner = _make_planner({"g1": "h1"})
        planner.graph_constraints = ["global_lock_a"]
        seen = {}

        def _subphase(**kw):
            seen["constraints"] = kw["phase_graph_constraints"]
            return [1.0], {"paths": [], "edges": []}

        planner._plan_release_subphase = _subphase
        planner.release("g1", q_current=[0.0], frozen_arms_mode="global")
        assert seen["constraints"] == ["global_lock_a"]

    def test_frozen_arms_mode_auto_builds_locked_joint_constraints(self):
        planner = _make_planner({"g1": "h1"})
        planner.graph_builder = type(
            "GB", (), {"ps": "ps", "robot": "robot", "backend": "pyhpp"}
        )()
        planner._release_frozen_arms = lambda *a, **kw: ["arm2"]
        seen = {}

        def _subphase(**kw):
            seen["constraints"] = kw["phase_graph_constraints"]
            return [1.0], {"paths": [], "edges": []}

        planner._plan_release_subphase = _subphase

        with patch(
            "long_tamp.planning.constraints.ConstraintBuilder"
            ".create_locked_joint_constraints",
            return_value=(["locked_arm2"], ["arm2_joint"]),
        ) as mock_create:
            planner.release("g1", q_current=[0.0], frozen_arms_mode="auto")

        mock_create.assert_called_once_with(
            "ps", "robot", [0.0], ["arm2"], backend="pyhpp"
        )
        assert seen["constraints"] == ["locked_arm2"]
