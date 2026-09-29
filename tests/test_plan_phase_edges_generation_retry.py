"""
Unit tests for retrying the initial ``generate_via_edge()`` call inside
``_plan_phase_edges()``.

Found live: driving TWIN's real bimanual scene through the new
``GraspSequencePlanner.grasp()`` primitive (see
``tests/test_grasp_release_use_case_twin.py``), the
``panda_left/gripper > ball/handle | f_12`` edge intermittently failed
with "Target generation failed" even though the exact same call had
succeeded on 4 of 5 other real runs -- a single unlucky random-restart IK
draw. Reading ``_plan_phase_edges()`` confirmed why: unlike the RRT
planning step just below it (which already retries up to
``_MAX_COLLISION_RETRIES`` times, regenerating the target between
attempts), the *initial* ``generate_via_edge()`` call had no retry of its
own -- one failed draw aborted the whole phase immediately. This is
shared, pre-existing code: ``plan_sequence()``, ``resume_sequence()``, and
``grasp()`` all hit the identical gap, since all three drive
``_plan_phase_edges()``.

The fix wraps that initial call in the same retry-and-regenerate shape the
planning step already uses: up to ``_MAX_COLLISION_RETRIES`` attempts,
warm-start hint applied only on the first attempt (consistent with the
planning retry loop's own regeneration, which is also always unhinted).

Importing long_tamp requires pyhpp even though the method under test has
no HPP dependency itself, so these tests must run inside the hpp-arm64
container (same convention as the other _plan_phase_edges test files).
"""

from long_tamp.tasks.grasp_sequence import GraspSequencePlanner


class _FakePath:
    """Minimal stand-in with neither getInitialConfig nor getEndConfig,
    so _plan_phase_edges falls back to q_start = q_target on success."""


class _FakeRunLogger:
    def log(self, event, **kwargs):
        pass

    def close(self):
        pass


class _FailNTimesThenSucceed:
    """generate_via_edge() stand-in: fails on the first ``n_failures``
    calls (ok=False), then returns a successful draw forever after.
    Records every (edge_name, q_hint) call, in order."""

    def __init__(self, n_failures, success_config=(0.1, 0.2)):
        self.n_failures = n_failures
        self.success_config = list(success_config)
        self.calls = []

    def generate_via_edge(self, edge_name, q_from, config_label, q_hint=None):
        self.calls.append((edge_name, q_hint))
        if len(self.calls) <= self.n_failures:
            return False, None
        return True, self.success_config


class _AlwaysFails:
    """generate_via_edge() stand-in: never succeeds. Records every call."""

    def __init__(self):
        self.calls = []

    def generate_via_edge(self, edge_name, q_from, config_label, q_hint=None):
        self.calls.append((edge_name, q_hint))
        return False, None


def _make_planner(config_gen, max_generation_retries=2):
    """Bare GraspSequencePlanner, bypassing __init__ (matches the pattern
    in test_grasp_sequence_phase_q_hints.py / test_grasp_sequence_logging.py).

    ``max_generation_retries`` maps to ``_MAX_GENERATION_RETRIES`` -- the
    small, dedicated budget for the initial generate_via_edge() retry loop
    (deliberately separate from ``_MAX_COLLISION_RETRIES``, the RRT-planning
    retry count, since generate_via_edge() already does its own internal
    random-restart search; see that attribute's definition in __init__).
    """
    planner = object.__new__(GraspSequencePlanner)
    planner.run_logger = _FakeRunLogger()
    planner._MAX_GENERATION_RETRIES = max_generation_retries
    planner._MAX_COLLISION_RETRIES = 1  # not under test here; the RRT-planning
    # retry loop is reached once generation succeeds -- give it just enough
    # budget to run once (fake plan_transition_edge always succeeds).
    planner.total_planning_time = 0.0
    planner.edge_stats = {}
    planner.auto_save_dir = None
    planner.phase_results = []
    planner.last_failure_info = None
    planner.invalidated_phase_hints = set()
    planner.config_gen = config_gen

    class _FakePlanner:
        viewer = None

        def plan_transition_edge(self, edge, q1, q2):
            return (_FakePath(), _FakePath())

    planner.planner = _FakePlanner()
    return planner


class TestGenerationRetriesOnFailure:
    def test_succeeds_after_one_failed_draw(self):
        # n_failures=1 fits _MAX_GENERATION_RETRIES's default budget of 2 --
        # deliberately small (see __init__): generate_via_edge() already
        # does its own internal random-restart search, so this budget
        # exists to catch a one-off unlucky draw, not to retry an
        # already-expensive full search many times over.
        config_gen = _FailNTimesThenSucceed(n_failures=1)
        planner = _make_planner(config_gen)

        result = planner._plan_phase_edges(
            phase_idx=0,
            gripper="g1",
            handle="h1",
            edge_sequence=["edge01"],
            q_current=[0.0, 0.0],
            skip_phases=None,
            start_edge_idx=0,
            is_resume=False,
            verbose=False,
        )

        assert len(result["phase_paths"]) == 1
        assert len(config_gen.calls) == 2  # 1 failure + 1 success

    def test_only_the_first_attempt_receives_the_warm_start_hint(self):
        config_gen = _FailNTimesThenSucceed(n_failures=1)
        planner = _make_planner(config_gen)
        hint = [9.9, 9.9]

        planner._plan_phase_edges(
            phase_idx=0,
            gripper="g1",
            handle="h1",
            edge_sequence=["edge01"],
            q_current=[0.0, 0.0],
            skip_phases=None,
            start_edge_idx=0,
            is_resume=False,
            verbose=False,
            phase_q_hints={0: hint},
        )

        assert [q_hint for _, q_hint in config_gen.calls] == [hint, None]

    def test_a_failed_hinted_attempt_invalidates_the_phase_hint(self):
        config_gen = _FailNTimesThenSucceed(n_failures=1)
        planner = _make_planner(config_gen)

        planner._plan_phase_edges(
            phase_idx=2,
            gripper="g1",
            handle="h1",
            edge_sequence=["edge01"],
            q_current=[0.0, 0.0],
            skip_phases=None,
            start_edge_idx=0,
            is_resume=False,
            verbose=False,
            phase_q_hints={2: [1.0, 2.0]},
        )

        assert planner.invalidated_phase_hints == {2}

    def test_unhinted_retry_does_not_touch_invalidated_phase_hints(self):
        config_gen = _FailNTimesThenSucceed(n_failures=1)
        planner = _make_planner(config_gen)

        planner._plan_phase_edges(
            phase_idx=0,
            gripper="g1",
            handle="h1",
            edge_sequence=["edge01"],
            q_current=[0.0, 0.0],
            skip_phases=None,
            start_edge_idx=0,
            is_resume=False,
            verbose=False,
        )

        assert planner.invalidated_phase_hints == set()

    def test_exhausting_all_attempts_still_raises_with_the_original_message(self):
        config_gen = _AlwaysFails()
        planner = _make_planner(config_gen, max_generation_retries=3)

        try:
            planner._plan_phase_edges(
                phase_idx=0,
                gripper="g1",
                handle="h1",
                edge_sequence=["edge01"],
                q_current=[0.0, 0.0],
                skip_phases=None,
                start_edge_idx=0,
                is_resume=False,
                verbose=False,
            )
            raise AssertionError("expected RuntimeError")
        except RuntimeError as e:
            assert "Target generation failed" in str(e)
            assert "edge01" in str(e)

        assert len(config_gen.calls) == 3  # exactly _MAX_GENERATION_RETRIES
        assert planner.phase_results[-1]["complete"] is False
        assert planner.phase_results[-1]["error_message"].startswith(
            "Target generation failed"
        )
        assert planner.last_failure_info["error"].startswith(
            "Target generation failed"
        )

    def test_resume_mode_uses_its_own_error_message_on_exhaustion(self):
        config_gen = _AlwaysFails()
        planner = _make_planner(config_gen, max_generation_retries=2)

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
            raise AssertionError("expected RuntimeError")
        except RuntimeError as e:
            assert "Failed to generate target via edge 'edge01'" in str(e)

        assert len(config_gen.calls) == 2  # exactly _MAX_GENERATION_RETRIES

    def test_first_attempt_success_matches_pre_fix_call_count(self):
        """No regression for the common case: generation succeeds
        immediately, exactly one generate_via_edge() call, same as before
        this retry loop existed."""
        config_gen = _FailNTimesThenSucceed(n_failures=0)
        planner = _make_planner(config_gen)

        planner._plan_phase_edges(
            phase_idx=0,
            gripper="g1",
            handle="h1",
            edge_sequence=["edge01", "edge12"],
            q_current=[0.0, 0.0],
            skip_phases=None,
            start_edge_idx=0,
            is_resume=False,
            verbose=False,
        )

        assert len(config_gen.calls) == 2  # one per edge, no retries needed
