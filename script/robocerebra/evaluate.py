#!/usr/bin/env python3
"""Run a policy against RoboCerebra's official segmented episode protocol.

The policy is an importable callback ``module:callable`` with the signature
``policy(env, case_info, segment_index, step_limit) -> (N, 7) OSC_POSE actions``.
It can wrap a MissionRunner and its scene-specific executor. This command
handles seeded resets, segment checks, official ``resume(k)``, and JSON output.
"""

from __future__ import annotations

import argparse
import importlib
import json
import os
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(HERE))

from replay_demo import server_command  # noqa: E402

from long_tamp.eval import RoboCerebraClient, evaluate_cases  # noqa: E402


def _load_policy(spec: str):
    module_name, separator, name = spec.partition(":")
    if not separator or not module_name or not name:
        raise ValueError("policy must be written as module:callable")
    policy = getattr(importlib.import_module(module_name), name)
    if not callable(policy):
        raise TypeError(f"{spec} is not callable")
    return policy


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("cases", nargs="*", help="cases such as Ideal/case1")
    parser.add_argument("--policy", required=True, help="import path module:callable")
    parser.add_argument("--rollouts", type=int, default=10)
    parser.add_argument("--seed", type=int, default=0, help="first rollout seed")
    parser.add_argument("--step-limit", type=int, default=150)
    parser.add_argument("--env-sh", default="~/devel/hpp/robocerebra/env.sh")
    parser.add_argument("--bench", default=os.environ.get("RC_BENCH"))
    parser.add_argument("--out", required=True, help="JSON result path")
    args = parser.parse_args(argv)
    if args.rollouts < 1:
        parser.error("--rollouts must be positive")
    if not args.bench:
        parser.error("set --bench or RC_BENCH")
    cases = args.cases
    policy = _load_policy(args.policy)
    seeds = list(range(args.seed, args.seed + args.rollouts))
    command = server_command(args.env_sh, args.bench, no_render=True)
    with RoboCerebraClient.spawn(command) as env:
        cases = cases or [case for case in env.cases() if case.startswith("Ideal/")]
        result: dict[str, Any] = evaluate_cases(
            env, cases, policy, seeds=seeds, step_limit=args.step_limit
        )
    result["policy"] = args.policy
    result["seeds"] = seeds
    result["protocol"] = "reset(seed); per-segment policy; resume(k); OSC_POSE"
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result["summary"], indent=2))
    print(f"wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
