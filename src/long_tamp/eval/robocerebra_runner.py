"""Segmented RoboCerebra evaluation with the benchmark's official resume.

The policy callback receives the live client and one annotated task segment,
and returns OSC_POSE actions for that segment. This keeps planning in the
caller (for example, a MissionRunner-backed policy) while this module owns
seeding, the 150-step budget, benchmark resume semantics, and result records.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Any

import numpy as np

from .robocerebra import RoboCerebraClient

Policy = Callable[[RoboCerebraClient, dict[str, Any], int, int], Any]


def evaluate_case(
    env: RoboCerebraClient,
    case: str,
    policy: Policy,
    *,
    seeds: Sequence[int],
    step_limit: int = 150,
) -> dict[str, Any]:
    """Evaluate one case over ``seeds``; each policy call is one segment.

    The policy returns an ``(N, 7)`` array of normalized OSC_POSE actions.
    Segment actions are applied one at a time so the first successful check
    stops the segment immediately. Unused budget is not carried forward.
    """
    if step_limit <= 0:
        raise ValueError("step_limit must be positive")
    info = env.load(case)
    spans = info["spans"]
    if not spans:
        raise ValueError(f"{case} has no annotated task segments")
    rollouts: list[dict[str, Any]] = []
    for seed in seeds:
        env.reset(step=0, seed=int(seed))
        segments = []
        for index, span in enumerate(spans):
            raw = np.asarray(policy(env, info, index, step_limit), dtype=float)
            if raw.size == 0:
                actions = np.empty((0, 7), dtype=float)
            elif raw.ndim == 1:
                actions = raw[None, :]
            else:
                actions = raw
            if actions.ndim != 2 or actions.shape[1] != 7:
                raise ValueError(
                    f"policy for {case} segment {index} returned {actions.shape}; expected (N, 7)"
                )
            if len(actions) > step_limit:
                raise ValueError(
                    f"policy for {case} segment {index} used {len(actions)} actions; limit is {step_limit}"
                )
            used = 0
            result = env.check()
            segment_done = _segment_done(info, result, index)
            for action in actions:
                result = env.step(action)
                used += 1
                segment_done = _segment_done(info, result, index)
                if result["all_done"] or segment_done:
                    break
            segments.append(
                {
                    "segment": index,
                    "description": info["steps"][index],
                    "demo_span": list(span),
                    "step_limit": step_limit,
                    "steps_used": used,
                    "total_completed": result["total_completed"],
                    "total": result["total"],
                    "all_done": result["all_done"],
                    "segment_success": segment_done,
                }
            )
            if result["all_done"]:
                break
            if index + 1 < len(spans):
                env.resume(index + 1)
        final = env.check()
        rollouts.append(
            {
                "seed": int(seed),
                "success": bool(final["all_done"]),
                "total_completed": final["total_completed"],
                "total": final["total"],
                "segments": segments,
            }
        )
    return {
        "case": case,
        "model": info.get("model", "bddl"),
        "step_limit": step_limit,
        "rollouts": rollouts,
        "success_rate": (
            sum(row["success"] for row in rollouts) / len(rollouts) if rollouts else 0.0
        ),
    }


def _segment_done(info: dict[str, Any], result: dict[str, Any], segment: int) -> bool:
    """Whether every predicate annotated for this task step has registered."""
    goal_steps = info.get("goal_steps", {})
    progress = result.get("progress", {})
    required = {
        obj: sum(task_step <= segment for task_step in steps)
        for obj, steps in goal_steps.items()
        if any(task_step == segment for task_step in steps)
    }
    return bool(required) and all(progress.get(obj, 0) >= count for obj, count in required.items())


def evaluate_cases(
    env: RoboCerebraClient,
    cases: Sequence[str],
    policy: Policy,
    *,
    seeds: Sequence[int],
    step_limit: int = 150,
) -> dict[str, Any]:
    """Evaluate multiple cases and report seed records and per-case SR."""
    rows = {
        case: evaluate_case(env, case, policy, seeds=seeds, step_limit=step_limit)
        for case in cases
    }
    return {
        "cases": rows,
        "summary": {
            "cases": len(rows),
            "rollouts_per_case": len(seeds),
            "success_rate_by_case": {case: row["success_rate"] for case, row in rows.items()},
        },
    }


__all__ = ["Policy", "evaluate_case", "evaluate_cases"]
