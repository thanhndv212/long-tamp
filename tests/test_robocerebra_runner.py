import numpy as np
import pytest

from long_tamp.eval.robocerebra_runner import (
    evaluate_case,
    evaluate_cases,
    merge_results,
)


class FakeEnv:
    def __init__(self):
        self.t = 0
        self.progress = {"cube_1": 0}
        self.seeds = []
        self.resumes = []

    def load(self, case):
        return {
            "case": case,
            "model": "bddl",
            "steps": ["pick cube", "place cube"],
            "spans": [[0, 10], [10, 20]],
            "goal_steps": {"cube_1": [0, 1]},
        }

    def reset(self, step=0, seed=None):
        self.t = 0
        self.progress = {"cube_1": 0}
        self.seeds.append(seed)
        return self.check()

    def resume(self, step):
        self.resumes.append(step)
        self.progress["cube_1"] = step
        return self.check()

    def step(self, action):
        self.t += 1
        self.progress["cube_1"] = min(2, self.progress["cube_1"] + 1)
        return self.check()

    def check(self):
        done = self.progress["cube_1"]
        return {
            "progress": dict(self.progress),
            "total_completed": done,
            "total": 2,
            "all_done": done == 2,
        }


def test_evaluate_case_uses_segment_budget_resume_and_seed():
    env = FakeEnv()
    calls = []

    def policy(client, info, segment, limit):
        calls.append((segment, limit))
        return np.zeros((5, 7))

    result = evaluate_case(env, "Ideal/case1", policy, seeds=[9], step_limit=5)
    rollout = result["rollouts"][0]
    assert env.seeds == [9]
    assert env.resumes == [1]
    assert calls == [(0, 5), (1, 5)]
    assert [segment["steps_used"] for segment in rollout["segments"]] == [1, 1]
    assert all(segment["segment_success"] for segment in rollout["segments"])
    assert result["success_rate"] == 1.0


def test_evaluate_case_continuous_carries_state_and_scores_share():
    env = FakeEnv()

    def policy(client, info, segment, limit):
        # The pick fails: segment 0 spends nothing, segment 1 runs on.
        return np.zeros((0 if segment == 0 else 1, 7))

    result = evaluate_case(
        env, "Ideal/case1", policy, seeds=[0], step_limit=5, resume=False
    )
    rollout = result["rollouts"][0]
    assert env.resumes == []
    assert result["resume"] is False
    assert [s["segment_success"] for s in rollout["segments"]] == [False, False]
    assert rollout["completed_share"] == 0.5
    summary = evaluate_cases(
        FakeEnv(), ["Ideal/case1"], policy, seeds=[0], step_limit=5, resume=False
    )["summary"]
    assert summary["completed_share"] == 0.5
    assert summary["completed_share_by_case"] == {"Ideal/case1": 0.5}


class Finishing:
    """A controller with stages left after its predicate registers."""

    def __init__(self, steps):
        self.left = steps

    @property
    def done(self):
        return self.left == 0

    def __call__(self, obs):
        self.left -= 1
        return np.zeros(7)


def test_continuous_runs_controller_past_success_until_done():
    env = FakeEnv()
    resumed = evaluate_case(
        FakeEnv(), "Ideal/case1", lambda *_: Finishing(3), seeds=[0], step_limit=5
    )
    assert [s["steps_used"] for s in resumed["rollouts"][0]["segments"]] == [1, 1]
    result = evaluate_case(
        env,
        "Ideal/case1",
        lambda *_: Finishing(3),
        seeds=[0],
        step_limit=5,
        resume=False,
    )
    # Segment 0 registers at step 1 and runs on; its next step completes
    # the case, which ends the rollout.
    segments = result["rollouts"][0]["segments"]
    assert [(s["steps_used"], s["success_step"]) for s in segments] == [(2, 1)]
    assert result["rollouts"][0]["success"]


def test_continuous_replans_a_controller_done_without_success():
    class Stalls(Finishing):
        info = {"status": "planned"}

        def __call__(self, obs):
            self.left -= 1
            return np.zeros(7)

    class StallingEnv(FakeEnv):
        def step(self, action):
            return self.check()  # nothing ever registers

    calls = []

    def policy(env, info, segment, limit):
        calls.append((segment, limit))
        return Stalls(2)

    result = evaluate_case(
        StallingEnv(), "Ideal/case1", policy, seeds=[0], step_limit=5, resume=False
    )
    segments = result["rollouts"][0]["segments"]
    assert calls == [(0, 5), (0, 3), (0, 1), (1, 5), (1, 3), (1, 1)]
    assert [s["steps_used"] for s in segments] == [5, 5]
    assert [len(s["replans"]) for s in segments] == [2, 2]
    # A controller done from the start (holding still) is not replanned.
    holding = evaluate_case(
        StallingEnv(),
        "Ideal/case1",
        lambda *_: Stalls(0),
        seeds=[0],
        step_limit=5,
        resume=False,
    )
    assert all("replans" not in s for s in holding["rollouts"][0]["segments"])
    resumed = StallingEnv()
    calls.clear()
    evaluate_case(resumed, "Ideal/case1", policy, seeds=[0], step_limit=5)
    assert calls == [(0, 5), (1, 5)]


def test_evaluate_case_rejects_budget_overrun():
    env = FakeEnv()
    with pytest.raises(ValueError, match="limit is 1"):
        evaluate_case(
            env,
            "Ideal/case1",
            lambda *_: np.zeros((2, 7)),
            seeds=[1],
            step_limit=1,
        )


class Statused:
    """An open-loop policy whose record carries a status, as HppPolicy's does."""

    def __init__(self, actions, status):
        self.actions, self.info = actions, {"status": status}

    def __len__(self):
        return len(self.actions)

    def __getitem__(self, index):
        return self.actions[index]


def test_merge_results_joins_seeds_and_counts_supported_segments():
    def policy(env, info, segment, limit):
        if segment == 0:
            return np.tile([1.0, 0, 0, 0, 0, 0, 1], (10, 1))
        return Statused(np.empty((0, 7)), "unsupported")

    parts = [
        evaluate_cases(
            FakeEnv(),
            ["Ideal/case10", "Ideal/case2"],
            policy,
            seeds=[seed],
            step_limit=20,
        )
        for seed in (1, 0)
    ]
    merged = merge_results(parts)
    assert list(merged["cases"]) == ["Ideal/case2", "Ideal/case10"]
    assert [r["seed"] for r in merged["cases"]["Ideal/case2"]["rollouts"]] == [0, 1]
    summary = merged["summary"]
    assert summary["rollouts_per_case"] == {"Ideal/case2": 2, "Ideal/case10": 2}
    assert summary["segments"] == 8 and summary["unsupported_segments"] == 4
    assert summary["supported_segment_success_rate"] == 1.0
    assert summary["supported_segment_success_by_case"] == {
        "Ideal/case2": "2/2",
        "Ideal/case10": "2/2",
    }
    with pytest.raises(ValueError, match="seed repeated"):
        merge_results([parts[0], parts[0]])
    continuous = evaluate_cases(
        FakeEnv(), ["Ideal/case2"], policy, seeds=[5], step_limit=20, resume=False
    )
    with pytest.raises(ValueError, match="continuous runs mixed"):
        merge_results([parts[0], continuous])
