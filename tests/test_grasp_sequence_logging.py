"""
Unit tests for the RunLogger logging-asymmetry fix: resume_sequence()'s
phases/edges used to be invisible to RunLogger-based analysis/replay
because every ``self.run_logger.log(...)`` call in the shared
plan_sequence()/resume_sequence() helpers was gated by ``not is_resume``.

See docs/plans/refactor-codebase.md's "Decision needed: the logging
asymmetry" section (fixed 2026-08-09).

Importing agimus_spacelab requires pyhpp (see that same doc's
verification-model notes) even though the helpers under test have no HPP
dependency themselves, so these tests must run inside the hpp-arm64
container.
"""

from agimus_spacelab.planning.grasp_state import GraspStateTracker
from agimus_spacelab.tasks.grasp_sequence import GraspSequencePlanner


class _FakeRunLogger:
    """Records every log() call and close(), for asserting on event types
    without needing a real JSONL-writing RunLogger."""

    def __init__(self):
        self.events = []
        self.closed = False

    def log(self, event, **kwargs):
        self.events.append((event, kwargs))

    def close(self):
        self.closed = True

    def event_types(self):
        return [e for e, _ in self.events]


def _make_planner():
    """Bare GraspSequencePlanner, bypassing __init__."""
    return object.__new__(GraspSequencePlanner)


class TestBuildPhaseGraphAndConstraintsLogging:
    """_build_phase_graph_and_constraints(): the ``phase_start`` RunLogger
    event must fire regardless of ``emit_logs`` (which now only gates
    console-print verbosity, not RunLogger visibility)."""

    def _make_ready_planner(self):
        planner = _make_planner()
        planner.run_logger = _FakeRunLogger()
        planner.grasp_tracker = GraspStateTracker(
            grippers=["g1"], handles=["h1"], initial_grasps=None
        )

        class _FakeGraphBuilder:
            robot = object()
            ps = object()

            def build_phase_graph(self, **kwargs):
                pass

            def get_graph(self):
                return object()

        planner.graph_builder = _FakeGraphBuilder()
        planner.planner = object()  # no .graph attr -> update branch skipped
        planner.config_gen = None
        planner.task_config = object()
        # Any non-"pyhpp" value so ConfigGenerator.__init__ doesn't call
        # ps.configurationShooter() (only done for "pyhpp").
        planner.backend = "none"
        return planner

    def test_phase_start_fires_even_when_emit_logs_false(self):
        # emit_logs=False mirrors what _run_phase_loop passes for
        # resume_sequence() (emit_logs=not is_resume).
        planner = self._make_ready_planner()

        planner._build_phase_graph_and_constraints(
            phase_idx=0,
            gripper="g1",
            handle="h1",
            q_current=[0.0],
            frozen_arms_mode="none",
            per_phase_frozen_arms=None,
            q_scene_init=None,
            verbose=True,
            emit_logs=False,
        )

        assert "phase_start" in planner.run_logger.event_types()

    def test_phase_start_fires_when_emit_logs_true(self):
        planner = self._make_ready_planner()

        planner._build_phase_graph_and_constraints(
            phase_idx=2,
            gripper="g1",
            handle="h1",
            q_current=[0.0],
            frozen_arms_mode="none",
            per_phase_frozen_arms=None,
            q_scene_init=None,
            verbose=True,
            emit_logs=True,
        )

        events = dict(planner.run_logger.events)
        assert events["phase_start"]["phase"] == 3  # 1-based


class TestFinalizePhaseResultLogging:
    """_finalize_phase_result(): the ``phase_end`` RunLogger event must
    fire regardless of ``is_resume``."""

    def _make_ready_planner(self):
        planner = _make_planner()
        planner.run_logger = _FakeRunLogger()
        planner.grasp_tracker = GraspStateTracker(
            grippers=["g1"], handles=["h1"], initial_grasps=None
        )
        planner._last_pregrasp_q = {}
        planner.auto_save_dir = None
        planner.phase_results = []
        return planner

    def test_phase_end_fires_when_is_resume_true(self):
        planner = self._make_ready_planner()

        result_q = planner._finalize_phase_result(
            phase_idx=0,
            gripper="g1",
            handle="h1",
            edge_sequence=["e01", "e12"],
            phase_paths=["path0", "path1"],
            phase_geometric_paths=[],
            edge_stats_list=[
                {"gen_time": 0.1, "plan_time": 0.2, "total_time": 0.3}
            ],
            q_start=[0.0],
            q_pregrasp_for_cache=None,
            skip_phases=None,
            verbose=True,
            is_resume=True,
        )

        assert result_q == [0.0]
        assert "phase_end" in planner.run_logger.event_types()
        events = dict(planner.run_logger.events)
        assert events["phase_end"]["success"] is True
        assert len(planner.phase_results) == 1

    def test_phase_end_fires_when_is_resume_false(self):
        planner = self._make_ready_planner()

        planner._finalize_phase_result(
            phase_idx=0,
            gripper="g1",
            handle="h1",
            edge_sequence=["e01"],
            phase_paths=["path0"],
            phase_geometric_paths=[],
            edge_stats_list=[
                {"gen_time": 0.1, "plan_time": 0.2, "total_time": 0.3}
            ],
            q_start=[0.0],
            q_pregrasp_for_cache=None,
            skip_phases=None,
            verbose=True,
            is_resume=False,
        )

        assert "phase_end" in planner.run_logger.event_types()


class _FakePath:
    """Minimal stand-in with neither getInitialConfig nor getEndConfig,
    so _plan_phase_edges falls back to q_start = q_target on success."""


class TestPlanPhaseEdgesLogging:
    """_plan_phase_edges(): ``edge_start``/``edge_end`` RunLogger events
    must fire regardless of ``is_resume`` (the collision-retry/attempt
    bookkeeping and console-print differences stay is_resume-gated,
    unchanged by this fix)."""

    def _make_ready_planner(self, plan_transition_edge):
        planner = _make_planner()
        planner.run_logger = _FakeRunLogger()
        planner._MAX_COLLISION_RETRIES = 1
        planner.total_planning_time = 0.0
        planner.edge_stats = {}
        planner.auto_save_dir = None
        planner.phase_results = []
        planner.last_failure_info = None

        class _FakeConfigGen:
            def generate_via_edge(self, edge_name, q_from, config_label, q_hint=None):
                return True, [0.1, 0.2]

        class _FakePlanner:
            viewer = None

            def plan_transition_edge(self, edge, q1, q2):
                return plan_transition_edge()

        planner.config_gen = _FakeConfigGen()
        planner.planner = _FakePlanner()
        return planner

    def test_success_emits_edge_start_and_edge_end(self):
        planner = self._make_ready_planner(
            plan_transition_edge=lambda: (_FakePath(), _FakePath())
        )

        result = planner._plan_phase_edges(
            phase_idx=0,
            gripper="g1",
            handle="h1",
            edge_sequence=["edge01"],
            q_current=[0.0, 0.0],
            skip_phases=None,
            start_edge_idx=0,
            is_resume=True,
            verbose=False,
        )

        assert len(result["phase_paths"]) == 1
        event_types = planner.run_logger.event_types()
        assert "edge_start" in event_types
        assert "edge_end" in event_types
        edge_end = dict(planner.run_logger.events)["edge_end"]
        assert edge_end["success"] is True

    def test_planning_failure_emits_edge_end_false(self):
        def _always_fail():
            raise RuntimeError("planning boom")

        planner = self._make_ready_planner(plan_transition_edge=_always_fail)

        try:
            planner._plan_phase_edges(
                phase_idx=0,
                gripper="g1",
                handle="h1",
                edge_sequence=["edge01"],
                q_current=[0.0, 0.0],
                skip_phases=None,
                start_edge_idx=0,
                is_resume=True,
                verbose=False,
            )
        except RuntimeError:
            pass
        else:
            raise AssertionError("expected RuntimeError from failed planning")

        assert "edge_start" in planner.run_logger.event_types()
        events = dict(planner.run_logger.events)
        assert events["edge_end"]["success"] is False


class TestResumeSequenceEpilogue:
    """resume_sequence(): on success, must emit ``run_end`` and close the
    logger -- previously only plan_sequence() did this, leaving a
    successfully-resumed run's log dangling open forever."""

    def _make_ready_planner(self):
        planner = _make_planner()
        planner.run_logger = _FakeRunLogger()
        planner.grasp_tracker = GraspStateTracker(
            grippers=["g1"], handles=["h1"], initial_grasps=None
        )
        planner.resume_attempt_count = 0
        planner.total_planning_time = 1.23
        planner.planner = object()  # no configure_transition_planner attr
        planner.last_failure_info = {
            "phase_idx": 0,
            "edge_idx": 0,
            "edge_name": "edge01",
            "q_current": [0.0],
            "error": "boom",
            "completed_phases": 0,
            "completed_edges_in_phase": 0,
        }
        planner.phase_results = [
            {
                "phase": 1,
                "complete": False,
                "failed_edge_idx": 0,
                "failed_edge_name": "edge01",
                "error_message": "boom",
                "paths": [],
                "edges": ["edge01"],
                "last_q_start": [0.0],
            }
        ]
        planner.original_sequence = [("g1", "h1")]

        # Bypass the real per-phase planning entirely -- this test is only
        # about the epilogue's run_end/close, already covered by the
        # _run_phase_loop sub-methods' own tests above.
        def _fake_run_phase_loop(**kwargs):
            assert kwargs["is_resume"] is True
            assert kwargs["loop_start_time"] is not None
            return [1.0, 2.0]

        planner._run_phase_loop = _fake_run_phase_loop
        return planner

    def test_emits_run_end_and_closes_on_success(self):
        planner = self._make_ready_planner()

        result = planner.resume_sequence(verbose=False)

        assert result["success"] is True
        assert result["final_config"] == [1.0, 2.0]
        assert "run_end" in planner.run_logger.event_types()
        run_end = dict(planner.run_logger.events)["run_end"]
        assert run_end["success"] is True
        assert run_end["final_config"] == [1.0, 2.0]
        assert planner.run_logger.closed is True
