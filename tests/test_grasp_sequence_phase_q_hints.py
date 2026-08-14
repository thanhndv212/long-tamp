"""
Unit tests for the ``phase_q_hints`` warm-start plumbing through
``_plan_phase_edges()``.

Added alongside ``find_feasible_phase_target()`` (see grasp_sequence.py),
built to fix RS6's CON0 grasp being provably unreachable: Phase 2's WB
grasp is a randomized target-generation call whose result pins RS6's final
orientation, and this time the random draw happened to leave CON0 facing
away from the screwdriver. ``find_feasible_phase_target()`` searches for a
Phase-2 candidate that also leaves Phase 3 reachable, then this plumbing
warm-starts the real ``generate_via_edge()`` call with that candidate via
``q_hint`` so ``plan_sequence()``/``resume_sequence()`` reproduce it
instead of drawing a fresh (and possibly bad) random target.

The hint is the candidate's whole per-edge config chain, one entry per
edge of the phase's edge sequence. A single config (last edge only) is
still accepted but does not actually pin the phase's committed config --
see ``_edge_hints_for_phase()`` for why, and the RS5/2026-08-13 case where
that shape let the lookahead report success and CON0 fail anyway.

These tests only cover the plumbing: which of ``_plan_phase_edges()``'s
``generate_via_edge()`` call sites receives which hint, and when a
collision-retry redraw breaks the chain. They do not exercise
``find_feasible_phase_target()`` itself
(needs a real HPP graph/solver -- see test_lookahead_phase_target.py) or
prove the hint actually fixes RS6 (see that same integration test).

Importing agimus_spacelab requires pyhpp even though the method under test
has no HPP dependency itself, so these tests must run inside the
hpp-arm64 container (same convention as test_grasp_sequence_logging.py /
test_grasp_sequence_resume_state.py).
"""

from agimus_spacelab.tasks.grasp_sequence import GraspSequencePlanner


class _FakePath:
    """Minimal stand-in with neither getInitialConfig nor getEndConfig,
    so _plan_phase_edges falls back to q_start = q_target on success."""


class _FakeRunLogger:
    def log(self, event, **kwargs):
        pass

    def close(self):
        pass


class _RecordingConfigGen:
    """Records (edge_name, q_hint) for every generate_via_edge() call, in
    call order, so tests can assert exactly which call sites received a
    hint and which didn't."""

    def __init__(self):
        self.calls = []

    def generate_via_edge(self, edge_name, q_from, config_label, q_hint=None):
        self.calls.append((edge_name, q_hint))
        return True, [0.1, 0.2]


class _FailNTimesThenSucceed:
    """plan_transition_edge stand-in: raises on the first ``n_failures``
    calls, then returns a successful path pair forever after."""

    def __init__(self, n_failures):
        self.n_failures = n_failures
        self.calls = 0

    def __call__(self):
        self.calls += 1
        if self.calls <= self.n_failures:
            raise RuntimeError("planning boom")
        return (_FakePath(), _FakePath())


def _make_planner(plan_transition_edge, max_collision_retries=1):
    """Bare GraspSequencePlanner, bypassing __init__ (matches the pattern
    in test_grasp_sequence_logging.py's TestPlanPhaseEdgesLogging)."""
    planner = object.__new__(GraspSequencePlanner)
    planner.run_logger = _FakeRunLogger()
    planner._MAX_COLLISION_RETRIES = max_collision_retries
    planner.total_planning_time = 0.0
    planner.edge_stats = {}
    planner.auto_save_dir = None
    planner.phase_results = []
    planner.last_failure_info = None
    planner.invalidated_phase_hints = set()

    class _FakePlanner:
        viewer = None

        def plan_transition_edge(self, edge, q1, q2):
            return plan_transition_edge()

    planner.planner = _FakePlanner()
    return planner


class TestPhaseQHintsChain:
    """A chain hint (one config per edge, as find_feasible_phase_target
    returns) warm-starts EVERY edge -- that's what makes an uninterrupted
    run reproduce the probed candidate exactly instead of drifting off it
    via a randomized pregrasp edge."""

    def test_every_edge_gets_its_own_chain_entry(self):
        planner = _make_planner(lambda: (_FakePath(), _FakePath()))
        config_gen = _RecordingConfigGen()
        planner.config_gen = config_gen
        chain = [[1.0, 2.0], [3.0, 4.0]]

        planner._plan_phase_edges(
            phase_idx=0,
            gripper="g1",
            handle="h1",
            edge_sequence=["edge01", "edge12"],
            q_current=[0.0, 0.0],
            skip_phases=None,
            start_edge_idx=0,
            is_resume=True,
            verbose=False,
            phase_q_hints={0: chain},
        )

        assert [q_hint for _, q_hint in config_gen.calls] == chain
        assert planner.invalidated_phase_hints == set()

    def test_chain_length_mismatch_falls_back_to_last_edge_only(self):
        # A 3-config chain against a 2-edge phase can't be aligned; rather
        # than seeding edges with configs solved for different edges, only
        # the terminal config is used (legacy behavior).
        planner = _make_planner(lambda: (_FakePath(), _FakePath()))
        config_gen = _RecordingConfigGen()
        planner.config_gen = config_gen
        chain = [[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]]

        planner._plan_phase_edges(
            phase_idx=0,
            gripper="g1",
            handle="h1",
            edge_sequence=["edge01", "edge12"],
            q_current=[0.0, 0.0],
            skip_phases=None,
            start_edge_idx=0,
            is_resume=True,
            verbose=False,
            phase_q_hints={0: chain},
        )

        assert [q_hint for _, q_hint in config_gen.calls] == [None, [5.0, 6.0]]

    def test_chain_broken_by_retry_redraw_is_recorded(self):
        # First plan_transition_edge() attempt raises -> the hinted edge's
        # target is redrawn at random -> the candidate's guarantee about the
        # NEXT phase no longer holds, so the phase must be flagged even
        # though it goes on to succeed.
        planner = _make_planner(
            _FailNTimesThenSucceed(n_failures=1), max_collision_retries=2
        )
        planner.config_gen = _RecordingConfigGen()
        planner.invalidated_phase_hints = set()

        planner._plan_phase_edges(
            phase_idx=3,
            gripper="g1",
            handle="h1",
            edge_sequence=["edge01", "edge12"],
            q_current=[0.0, 0.0],
            skip_phases=None,
            start_edge_idx=0,
            is_resume=True,
            verbose=False,
            phase_q_hints={3: [[1.0, 2.0], [3.0, 4.0]]},
        )

        assert planner.invalidated_phase_hints == {3}

    def test_retry_redraw_on_an_unhinted_phase_is_not_recorded(self):
        planner = _make_planner(
            _FailNTimesThenSucceed(n_failures=1), max_collision_retries=2
        )
        planner.config_gen = _RecordingConfigGen()

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

        assert planner.invalidated_phase_hints == set()


class TestPhaseQHintsAppliedToLastEdgeOnly:
    """The legacy single-config shape, still accepted: applied to the
    phase's last edge only."""

    def test_single_edge_phase_gets_hint(self):
        planner = _make_planner(lambda: (_FakePath(), _FakePath()))
        config_gen = _RecordingConfigGen()
        planner.config_gen = config_gen
        hint = [9.9, 9.9]

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
            phase_q_hints={0: hint},
        )

        assert [q_hint for _, q_hint in config_gen.calls] == [hint]

    def test_multi_edge_phase_only_last_edge_gets_hint(self):
        planner = _make_planner(lambda: (_FakePath(), _FakePath()))
        config_gen = _RecordingConfigGen()
        planner.config_gen = config_gen
        hint = [1.0, 2.0]

        planner._plan_phase_edges(
            phase_idx=0,
            gripper="g1",
            handle="h1",
            edge_sequence=["edge01", "edge12"],
            q_current=[0.0, 0.0],
            skip_phases=None,
            start_edge_idx=0,
            is_resume=True,
            verbose=False,
            phase_q_hints={0: hint},
        )

        assert [q_hint for _, q_hint in config_gen.calls] == [None, hint]

    def test_hint_for_a_different_phase_idx_is_not_applied(self):
        planner = _make_planner(lambda: (_FakePath(), _FakePath()))
        config_gen = _RecordingConfigGen()
        planner.config_gen = config_gen

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
            phase_q_hints={5: [1.0, 2.0]},
        )

        assert [q_hint for _, q_hint in config_gen.calls] == [None]

    def test_omitted_phase_q_hints_reproduces_todays_behavior(self):
        planner = _make_planner(lambda: (_FakePath(), _FakePath()))
        config_gen = _RecordingConfigGen()
        planner.config_gen = config_gen

        planner._plan_phase_edges(
            phase_idx=0,
            gripper="g1",
            handle="h1",
            edge_sequence=["edge01", "edge12"],
            q_current=[0.0, 0.0],
            skip_phases=None,
            start_edge_idx=0,
            is_resume=True,
            verbose=False,
            # phase_q_hints omitted entirely -> defaults to None
        )

        assert [q_hint for _, q_hint in config_gen.calls] == [None, None]


class TestPhaseQHintsNeverAppliedToCollisionRetryRegeneration:
    def test_retry_regeneration_call_does_not_receive_hint(self):
        # _MAX_COLLISION_RETRIES=2: first plan_transition_edge() attempt
        # raises -> triggers exactly one regeneration generate_via_edge()
        # call -> second attempt succeeds.
        planner = _make_planner(
            _FailNTimesThenSucceed(n_failures=1), max_collision_retries=2
        )
        config_gen = _RecordingConfigGen()
        planner.config_gen = config_gen
        hint = [9.9, 9.9]

        planner._plan_phase_edges(
            phase_idx=0,
            gripper="g1",
            handle="h1",
            edge_sequence=["edge01"],  # single edge -> it IS the last edge
            q_current=[0.0, 0.0],
            skip_phases=None,
            start_edge_idx=0,
            is_resume=True,
            verbose=False,
            phase_q_hints={0: hint},
        )

        # Initial target generation for the (only, last) edge gets the
        # hint; the regeneration after the first RRT failure must not.
        assert [q_hint for _, q_hint in config_gen.calls] == [hint, None]
