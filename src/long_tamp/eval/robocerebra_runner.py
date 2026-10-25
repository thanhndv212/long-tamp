"""Segmented RoboCerebra evaluation with the benchmark's official resume.

The policy callback receives the live client and one annotated task segment,
and returns OSC_POSE actions for that segment: an ``(N, 7)`` array (open
loop), or a controller ``controller(obs) -> (7,)`` stepped closed loop from
each observation (an ``info`` dict on it is recorded with the segment). This keeps planning in the
caller (for example, a MissionRunner-backed policy) while this module owns
seeding, the 150-step budget, benchmark resume semantics, and result records.
"""

from __future__ import annotations

import time
from collections.abc import Callable, Sequence
from typing import Any

import numpy as np

from .robocerebra import RoboCerebraClient

Policy = Callable[[RoboCerebraClient, dict[str, Any], int, int], Any]
Controller = Callable[[dict[str, Any]], Any]


def evaluate_case(
    env: RoboCerebraClient,
    case: str,
    policy: Policy,
    *,
    seeds: Sequence[int],
    step_limit: int = 150,
) -> dict[str, Any]:
    """Evaluate one case over ``seeds``; each policy call is one segment.

    The policy returns an ``(N, 7)`` array of normalized OSC_POSE actions,
    or a controller called with each observation for the next action.
    Actions are applied one at a time so the first successful check stops
    the segment immediately; a controller runs until then or the budget is
    spent. Unused budget is not carried forward.
    """
    if step_limit <= 0:
        raise ValueError("step_limit must be positive")
    info = env.load(case)
    spans = info["spans"]
    if not spans:
        raise ValueError(f"{case} has no annotated task segments")
    rollouts: list[dict[str, Any]] = []
    for seed in seeds:
        obs = env.reset(step=0, seed=int(seed)).get("obs")
        segments = []
        for index, span in enumerate(spans):
            started = time.time()
            plan = policy(env, info, index, step_limit)
            policy_seconds = time.time() - started
            actions = (
                plan if callable(plan) else _actions(plan, case, index, step_limit)
            )
            used, monitor = 0, []
            result = env.check()
            segment_done = _segment_done(info, result, index)
            while used < step_limit and not (result["all_done"] or segment_done):
                if callable(actions):
                    action = np.asarray(actions(obs), dtype=float)
                elif used < len(actions):
                    action = actions[used]
                else:
                    break
                result = env.step(action)
                obs = result.get("obs", obs)
                monitor.extend(result.get("monitor", []))
                used += 1
                segment_done = _segment_done(info, result, index)
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
                    "policy_seconds": round(policy_seconds, 3),
                    "monitor": monitor,
                    **({"policy": plan.info} if hasattr(plan, "info") else {}),
                }
            )
            if result["all_done"]:
                break
            if index + 1 < len(spans):
                obs = env.resume(index + 1).get("obs", obs)
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


def _actions(raw: Any, case: str, index: int, step_limit: int) -> np.ndarray:
    """An open-loop policy's actions as ``(N, 7)``, within the budget."""
    actions = np.asarray(raw, dtype=float)
    if actions.size == 0:
        return np.empty((0, 7), dtype=float)
    if actions.ndim == 1:
        actions = actions[None, :]
    if actions.ndim != 2 or actions.shape[1] != 7:
        raise ValueError(
            f"policy for {case} segment {index} returned {actions.shape}; expected (N, 7)"
        )
    if len(actions) > step_limit:
        raise ValueError(
            f"policy for {case} segment {index} used {len(actions)} actions; limit is {step_limit}"
        )
    return actions


def _segment_done(info: dict[str, Any], result: dict[str, Any], segment: int) -> bool:
    """Whether every predicate annotated for this task step has registered."""
    goal_steps = info.get("goal_steps", {})
    progress = result.get("progress", {})
    required = {
        obj: sum(task_step <= segment for task_step in steps)
        for obj, steps in goal_steps.items()
        if any(task_step == segment for task_step in steps)
    }
    return bool(required) and all(
        progress.get(obj, 0) >= count for obj, count in required.items()
    )


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
    return {"cases": rows, "summary": summarize(rows)}


def summarize(rows: dict[str, dict[str, Any]]) -> dict[str, Any]:
    """Per-case rollout success rates, plus segment-level counts.

    Rollout success is the benchmark's metric, but the official resume marks
    earlier steps done, so it says little about the policy; each segment's
    own success (its predicates registered within its budget) does. Segments
    whose policy reported ``status: unsupported`` are counted apart.
    """
    segments = [
        segment
        for row in rows.values()
        for rollout in row["rollouts"]
        for segment in rollout["segments"]
    ]
    unsupported = [
        s for s in segments if s.get("policy", {}).get("status") == "unsupported"
    ]
    supported = [
        s for s in segments if s.get("policy", {}).get("status") != "unsupported"
    ]
    passed = [s for s in supported if s["segment_success"]]
    by_case = {}
    for case, row in rows.items():
        case_supported = [
            segment
            for rollout in row["rollouts"]
            for segment in rollout["segments"]
            if segment.get("policy", {}).get("status") != "unsupported"
        ]
        case_passed = sum(segment["segment_success"] for segment in case_supported)
        by_case[case] = f"{case_passed}/{len(case_supported)}"
    return {
        "cases": len(rows),
        "rollouts_per_case": {case: len(row["rollouts"]) for case, row in rows.items()},
        "success_rate_by_case": {
            case: row["success_rate"] for case, row in rows.items()
        },
        "segments": len(segments),
        "unsupported_segments": len(unsupported),
        "supported_segments": len(supported),
        "supported_segment_success": len(passed),
        "supported_segment_success_rate": round(len(passed) / len(supported), 4)
        if supported
        else 0.0,
        "supported_segment_success_by_case": by_case,
        "mean_steps_used_on_success": (
            round(sum(s["steps_used"] for s in passed) / len(passed), 1)
            if passed
            else None
        ),
    }


def merge_results(parts: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Join ``evaluate_cases`` results (for example, one per case and seed)
    into one, with each case's rollouts sorted by seed."""
    rows: dict[str, dict[str, Any]] = {}
    for part in parts:
        for case, row in part["cases"].items():
            merged = rows.setdefault(case, {**row, "rollouts": []})
            if merged["step_limit"] != row["step_limit"]:
                raise ValueError(f"{case}: step limits differ")
            merged["rollouts"].extend(row["rollouts"])
    for case, row in rows.items():
        seeds = [rollout["seed"] for rollout in row["rollouts"]]
        if len(set(seeds)) != len(seeds):
            raise ValueError(f"{case}: seed repeated in {sorted(seeds)}")
        row["rollouts"].sort(key=lambda rollout: rollout["seed"])
        row["success_rate"] = sum(r["success"] for r in row["rollouts"]) / len(
            row["rollouts"]
        )
    rows = dict(sorted(rows.items(), key=lambda item: _case_key(item[0])))
    return {"cases": rows, "summary": summarize(rows)}


def _case_key(case: str) -> tuple[str, int, str]:
    suite, _, name = case.rpartition("/")
    digits = "".join(ch for ch in name if ch.isdigit())
    return suite, int(digits) if digits else -1, name


__all__ = [
    "Controller",
    "Policy",
    "evaluate_case",
    "evaluate_cases",
    "merge_results",
    "summarize",
]
