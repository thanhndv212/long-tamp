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
