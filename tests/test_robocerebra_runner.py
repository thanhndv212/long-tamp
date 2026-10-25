import numpy as np
import pytest

from long_tamp.eval.robocerebra_runner import evaluate_case


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
