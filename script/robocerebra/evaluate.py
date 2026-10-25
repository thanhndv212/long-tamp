#!/usr/bin/env python3
"""Run a policy against RoboCerebra's official segmented episode protocol.

The policy is an importable callback ``module:callable`` with the signature
``policy(env, case_info, segment_index, step_limit)``, returning the
segment's ``(N, 7)`` OSC_POSE actions or a controller ``controller(obs)`` for
the next action. This command handles seeded resets, segment checks,
official ``resume(k)``, and JSON output.

Long runs can be split, one case and seed per process run in parallel, and
joined afterwards::

    python script/robocerebra/evaluate.py Ideal/case1 --rollouts 1 --seed 3 ... --out parts/case1-s3.json
    python script/robocerebra/evaluate.py --merge parts/*.json --out results/<date>-ideal-hpp.json
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
from long_tamp.eval.robocerebra_runner import merge_results  # noqa: E402


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
    parser.add_argument("--policy", help="import path module:callable")
    parser.add_argument("--merge", nargs="+", metavar="JSON", help="join earlier results instead of running")
    parser.add_argument("--rollouts", type=int, default=10)
    parser.add_argument("--seed", type=int, default=0, help="first rollout seed")
    parser.add_argument("--step-limit", type=int, default=150)
    parser.add_argument("--env-sh", default="~/devel/hpp/robocerebra/env.sh")
    parser.add_argument(
        "--bench", default=os.environ.get("RC_BENCH"), help="RoboCerebra_Bench (default: env.sh's RC_BENCH)"
    )
    parser.add_argument("--out", required=True, help="JSON result path")
    args = parser.parse_args(argv)
    if args.merge:
        parts = [json.loads(Path(path).read_text(encoding="utf-8")) for path in args.merge]
        policies = sorted({part.get("policy") for part in parts})
        if len(policies) != 1:
            parser.error(f"the results come from different policies: {policies}")
        result = merge_results(parts)
        result["policy"] = policies[0]
        result["seeds"] = sorted({seed for part in parts for seed in part.get("seeds", [])})
        result["protocol"] = parts[0].get("protocol")
        return _write(result, Path(args.out))
    if not args.policy:
        parser.error("--policy is required unless --merge")
    if args.rollouts < 1:
        parser.error("--rollouts must be positive")
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
    return _write(result, Path(args.out))


def _write(result: dict[str, Any], output: Path) -> int:
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result["summary"], indent=2))
    print(f"wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
