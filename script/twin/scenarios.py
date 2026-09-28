#!/usr/bin/env python3
"""Initial-state scenarios for the TWIN regrasp plan (validation level V4).

The same regrasp TaskPlan (``twin_bt_session.build_twin_regrasp_session``) runs
from different initial grasp states, reached by planning real grasps first.
A scenario passes when the plan completes and the grasp its setup already made
is skipped, not planned again:

    python3 scenarios.py --all --seed 1          # one process per scenario
    python3 scenarios.py --scenario left_holds_ball

Runs inside the hpp-agimus-arm64 container. See docs/development/validation.md.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).parent

#: name -> (grasps planned before the mission, labels expected to be skipped).
SCENARIOS: dict[str, dict[str, Any]] = {
    "nominal": {"setup": [], "skip": []},
    "left_holds_ball": {
        "setup": [("panda_left/gripper", "ball/handle")],
        "skip": ["Grasp ball/handle with panda_left/gripper"],
    },
    # Both arms hold the ball (TWIN's own two grasps, in the order that plans:
    # panda_right on handle2 alone fails target generation on f_01, see
    # build_twin_regrasp_session). The regrasp then releases and regrasps the
    # left arm while the right arm holds the ball.
    "both_hold": {
        "setup": [
            ("panda_left/gripper", "ball/handle"),
            ("panda_right/gripper", "ball/handle2"),
        ],
        "skip": ["Grasp ball/handle with panda_left/gripper"],
    },
}


def run_scenario(name: str, seed: int) -> dict[str, Any]:
    import ctypes
    import random

    import numpy as np
    import pinocchio

    ctypes.CDLL(None).srand(seed)
    pinocchio.seed(seed)
    random.seed(seed)
    np.random.seed(seed)  # noqa: NPY002

    from long_tamp.tasks.task_planning.runner import run_plan
    from twin_bt_session import build_twin_regrasp_session

    spec = SCENARIOS[name]
    session = build_twin_regrasp_session()
    planner, state = session.seq_planner, session.state
    for gripper, handle in spec["setup"]:
        for _attempt in range(3):
            r = planner.grasp(gripper, handle, state["q_current"])
            if r["success"]:
                state["q_current"] = r["final_config"]
                break
        else:
            return {"scenario": name, "seed": seed, "pass": False,
                    "reason": f"setup grasp {gripper} > {handle} failed: {r['message']}"}  # fmt: skip
    run = run_plan(session)
    missing = [label for label in spec["skip"] if label not in run.skipped]
    passed = run.success and not missing
    return {
        "scenario": name,
        "seed": seed,
        "pass": passed,
        "reason": "" if passed else (run.message or f"not skipped: {missing}"),
        "skipped": run.skipped,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scenario", choices=sorted(SCENARIOS))
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--json", type=Path)
    args = ap.parse_args()
    if args.scenario:
        result = run_scenario(args.scenario, args.seed)
        print("SCENARIO_RESULT " + json.dumps(result))
        return 0 if result["pass"] else 1
    if not args.all:
        ap.error("pass --scenario NAME or --all")
    results = []
    for name in SCENARIOS:
        proc = subprocess.run(
            [
                sys.executable,
                "-u",
                __file__,
                "--scenario",
                name,
                "--seed",
                str(args.seed),
            ],
            capture_output=True,
            text=True,
            cwd=HERE,
        )
        line = next(
            (l for l in proc.stdout.splitlines() if l.startswith("SCENARIO_RESULT ")),
            None,
        )
        result = (
            json.loads(line.split(" ", 1)[1])
            if line
            else {"scenario": name, "seed": args.seed, "pass": False,
                  "reason": f"crashed (exit {proc.returncode}): {proc.stderr[-300:]}"}  # fmt: skip
        )
        results.append(result)
        print(
            f"{'PASS' if result['pass'] else 'FAIL'}  {name:<20} "
            f"{result.get('reason') or 'skipped: ' + ', '.join(result.get('skipped', []))}",
            flush=True,
        )
    if args.json:
        args.json.write_text(json.dumps(results, indent=1) + "\n")
    passed = sum(r["pass"] for r in results)
    print(f"\n{passed}/{len(results)} scenarios passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
