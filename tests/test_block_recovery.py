"""Escalation ladder of ``run_block_with_recovery()``.

The planner is a stub: what's under test is when the ladder stops resuming
and replans the block, what a replan discards, what it replans with, and
when it gives up -- none of which needs a solver. Adapted from
agimus_spacelab's tests for the runner this module was distilled from.
"""

import pytest

from long_tamp.tasks.block_recovery import (
    make_lookahead_hints_factory,
    run_block_with_recovery,
    unreachable_failed_edge,
)

ENTRY = [0.0]
DONE = [1.0]
EDGE = "tool/g_tip > part/h_hole2 | ..._01"
BLOCK = [("arm/g", "part/h_seat"), ("tool/g_tip", "part/h_hole1")]


class FakeConfigGen:
    def __init__(self, failure=None):
        self.last_edge_failure = failure


class FakePlanner:
    """A block whose later phase fails on a schedule.

    ``resume_succeeds_after``: resume call number that succeeds (None:
    never). ``collisions``: attempts that reached collision checking in the
    failing phase (0 means *unreachable*). ``completed``: phases committed
    before the failing one. ``broken_hints``: phases whose hint chain the
    FIRST plan breaks.
    """

    def __init__(
        self,
        resume_succeeds_after=None,
        collisions=5,
        completed=1,
        broken_hints=(),
        plan_succeeds=False,
    ):
        self.resume_succeeds_after = resume_succeeds_after
        self.completed = completed
        self.broken_hints = set(broken_hints)
        self.plan_succeeds = plan_succeeds
        self.plan_calls = []  # hints each plan_sequence got
        self.frozen_seen = []  # per_phase_frozen_arms of each resume
        self.resume_calls = 0
        self.resets_at = []
        self.phase_results = []
        self.invalidated_phase_hints = set()
        self.config_gen = FakeConfigGen(
            {
                "edge_name": EDGE,
                "attempts": 100,
                "solver_failed": 100 - collisions,
                "collision_invalid": collisions,
            }
        )
        self.error = "Target generation failed: no target"

    def plan_sequence(self, **kw):
        self.plan_calls.append(kw.get("phase_q_hints"))
        self.invalidated_phase_hints = (
            set(self.broken_hints) if len(self.plan_calls) == 1 else set()
        )
        if self.plan_succeeds:
            return {"success": True, "final_config": DONE}
        self.phase_results = [{"phase": 1, "complete": True, "final_config": [0.5]}]
        raise RuntimeError("Phase 2, edge 1: Target generation failed")

    def resume_sequence(self, **kw):
        self.resume_calls += 1
        self.frozen_seen.append(
            {k: list(v) for k, v in (kw.get("per_phase_frozen_arms") or {}).items()}
        )
        if (
            self.resume_succeeds_after is not None
            and self.resume_calls >= self.resume_succeeds_after
        ):
            return {"success": True, "final_config": DONE}
        return {"success": False}

    def get_resumable_state(self):
        return {
            "phase_idx": 1,
            "edge_idx": 0,
            "edge_name": EDGE,
            "completed_phases": self.completed,
            "error": self.error,
        }

    def reset_grasp_tracker_to_call_start(self):
        self.resets_at.append(self.resume_calls)
        self.phase_results = []


def _run(planner, **kw):
    kw.setdefault("verbose", False)
    return run_block_with_recovery(planner, BLOCK, ENTRY, **kw)


class TestLadder:
    def test_first_plan_success_touches_nothing(self):
        p = FakePlanner(plan_succeeds=True)
        r = _run(p)
        assert r["success"] and r["final_config"] == DONE
        assert p.resume_calls == 0 and p.resets_at == []

    def test_resume_recovers_without_a_replan(self):
        p = FakePlanner(resume_succeeds_after=3)
        r = _run(p, resume_limit=10)
        assert r["success"] and r["resumes"] == 3 and r["replans"] == 0

    def test_replans_at_the_resume_limit_with_fresh_hints(self):
        hints = iter([{0: [[1.0]]}, {0: [[2.0]]}])
        p = FakePlanner(resume_succeeds_after=11)
        r = _run(p, resume_limit=10, hints_factory=lambda: next(hints))
        assert p.resets_at == [10], "replan at exactly the limit"
        assert p.plan_calls == [{0: [[1.0]]}, {0: [[2.0]]}]
        assert r["success"] and r["replans"] == 1

    def test_broken_hint_chain_replans_before_any_resume(self):
        hints = iter([{0: [[1.0]]}, {0: [[2.0]]}])
        p = FakePlanner(resume_succeeds_after=1, broken_hints={0})
        r = _run(p, hints_factory=lambda: next(hints))
        assert p.resets_at == [0]
        assert r["success"] and r["replans"] == 1


class TestUnreachableFastPath:
    def test_solver_only_failures_replan_early(self):
        p = FakePlanner(resume_succeeds_after=5, collisions=0)
        r = _run(p, resume_limit=40, unreachable_resumes=3)
        assert p.resets_at == [3]
        assert r["success"]

    def test_any_collision_keeps_the_slow_path(self):
        p = FakePlanner(resume_succeeds_after=12, collisions=1)
        _run(p, resume_limit=10, unreachable_resumes=3)
        assert p.resets_at == [10]

    def test_nothing_committed_means_nothing_to_undo(self):
        p = FakePlanner(resume_succeeds_after=12, collisions=0, completed=0)
        _run(p, resume_limit=10, unreachable_resumes=3)
        assert p.resets_at == [10]

    def test_disabled_with_zero(self):
        p = FakePlanner(resume_succeeds_after=12, collisions=0)
        _run(p, resume_limit=10, unreachable_resumes=0)
        assert p.resets_at == [10]

    def test_a_different_failure_breaks_the_streak(self):
        p = FakePlanner(resume_succeeds_after=4, collisions=0)
        real = p.get_resumable_state

        def state():
            s = real()
            if p.resume_calls == 2:
                s["error"] = "TransitionPlanner.planPath failed"
            return s

        p.get_resumable_state = state
        r = _run(p, resume_limit=40, unreachable_resumes=3)
        assert p.resets_at == [] and r["success"]

    def test_unreachable_failed_edge_reads_the_generator_record(self):
        assert unreachable_failed_edge(FakePlanner(collisions=0)) == EDGE
        assert unreachable_failed_edge(FakePlanner(collisions=2)) is None


class TestBudgetAndUnfreeze:
    def test_gives_up_after_max_replans(self):
        p = FakePlanner(resume_succeeds_after=None)
        r = _run(p, resume_limit=2, max_replans=3)
        assert not r["success"]
        assert r["replans"] == 3 and r["final_config"] == ENTRY
        assert "gave up after 3 replans" in r["message"]

    def test_unfreezes_only_listed_arms_after_n_failures(self):
        p = FakePlanner(resume_succeeds_after=4)
        _run(
            p,
            per_phase_frozen_arms={1: ["arm_a", "arm_b"]},
            unfreeze_after=2,
            unfreezable_arms=["arm_b"],
            resume_limit=40,
        )
        assert p.frozen_seen[0][1] == ["arm_a", "arm_b"]
        assert p.frozen_seen[-1][1] == ["arm_a"]

    def test_caller_frozen_dict_is_not_mutated(self):
        frozen = {1: ["arm_a", "arm_b"]}
        _run(
            FakePlanner(resume_succeeds_after=4),
            per_phase_frozen_arms=frozen,
            unfreeze_after=2,
            unfreezable_arms=["arm_b"],
        )
        assert frozen == {1: ["arm_a", "arm_b"]}

    def test_the_default_order_is_unfreeze_then_replan(self):
        import inspect

        sig = inspect.signature(run_block_with_recovery).parameters
        assert sig["unfreeze_after"].default < sig["resume_limit"].default


class TestLookaheadFactory:
    class _Probe:
        def __init__(self, results):
            self.results = list(results)
            self.calls = []

        def find_feasible_phase_target(self, **kw):
            self.calls.append(kw)
            return self.results.pop(0)

    SEQ = (("a/g", "p/h"), ("t/g", "p/h1"), ("t/g", None), ("t/g", "p/h2"))

    def test_returns_the_first_round_that_finds_a_candidate(self):
        probe = self._Probe([None, [[3.0]]])
        f = make_lookahead_hints_factory(
            probe,
            self.SEQ,
            [0.0],
            also_protect=[3],
            verbose=False,
            per_phase_frozen_arms={3: ["x"]},
        )
        assert f() == {0: [[3.0]]}
        assert len(probe.calls) == 2
        assert probe.calls[0]["also_reachable"] == [(("t/g", "p/h2"), ["x"])]

    def test_returns_none_when_every_round_fails(self):
        probe = self._Probe([None] * 3)
        f = make_lookahead_hints_factory(
            probe, self.SEQ, [0.0], max_rounds=3, verbose=False
        )
        assert f() is None and len(probe.calls) == 3

    def test_skips_release_and_out_of_range_protect_entries(self):
        probe = self._Probe([[[1.0]]])
        make_lookahead_hints_factory(
            probe, self.SEQ, [0.0], also_protect=[2, 9], verbose=False
        )()
        assert probe.calls[0]["also_reachable"] == []


@pytest.mark.parametrize("n", [1, 2])
def test_every_resume_is_counted(n):
    r = _run(FakePlanner(resume_succeeds_after=n))
    assert r["resumes"] == n
