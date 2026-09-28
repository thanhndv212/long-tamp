#!/usr/bin/env python3
"""Initial-state scenarios (validation level V4): one plan, several start states.

Each scenario reaches its start state by planning some of the mission's own
blocks from the nominal start (the *setup*), then runs the unchanged TaskPlan
from there. It passes when the mission completes, the work the setup already
did is skipped (effects that hold, parts already done), and none of it is
planned again:

    python3 build_scene.py --parts 2
    python3 scenarios.py --all --seed 1                  # every scenario, one process each
    python3 scenarios.py --scenario part1_assembled      # one, in this process

Runs inside the hpp-agimus-arm64 container. See docs/development/validation.md.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

HERE = Path(__file__).parent

_PICK = "bootstrap: pick driver"
_HOME0 = "ur10_right home (bootstrap)"

#: name -> (setup blocks, recorded facts after setup, expected skips, the
#: start state as atoms for the load-time check).
SCENARIOS: dict[str, dict[str, Any]] = {
    "nominal": {
        "setup": [],
        "facts": [],
        "skip": [],
        "atoms": [],
    },
    "driver_in_hand": {
        "setup": [_PICK],
        "facts": [],
        "skip": [_PICK],
        "atoms": ["holds(ur10_right/gripper, driver/h_grip)"],
    },
    "part_in_hand": {
        "setup": ["part1 A0: grasp"],
        "facts": [],
        "skip": ["part1 A0: grasp"],
        "atoms": ["holds(ur10_left/gripper, part1/h_grasp)"],
    },
    "part1_assembled": {
        "setup": [
            _PICK,
            _HOME0,
            "part1 A0: grasp",
            "part1 A: clamp + screw",
            "ur10_right home (part1)",
            "part1 B: release",
        ],
        "facts": ["screwed(part1, part1/h_hole1)", "screwed(part1, part1/h_hole2)"],
        "skip": [_PICK, "part1 clamped and screwed"],
        "atoms": [
            "holds(ur10_right/gripper, driver/h_grip)",
            "holds(fixtures/clamp1, part1/h_seat)",
            "screwed(part1, part1/h_hole1)",
            "screwed(part1, part1/h_hole2)",
        ],
    },
}


def run_scenario(name: str, seed: int, max_replans: int = 10) -> dict[str, Any]:
    """Set up ``name`` in a fresh scene and run the mission from it."""
    import task_screw_assembly as T
    from long_tamp.tasks.task_planning import RecordedFacts
    from long_tamp.tasks.task_planning.predicates import Literal

    spec = SCENARIOS[name]
    T.seed_everything(seed)
    task, planner = T.setup(log_dir=str(HERE / "runs" / f"scenario_{name}_seed{seed}"))
    n_parts = sum(1 for o in task.task_config.OBJECTS if o.startswith("part"))
    blocks = T.blocks_by_label(n_parts)
    index = {label: i for i, label in enumerate(blocks)}
    ctx: dict[str, Any] = {
        "q": list(task.q_init),
        "records": [],
        "trajectory": None,
        "checkpoint": None,
        "live_viewer": None,
        "n_parts": n_parts,
    }
    t0 = time.time()
    for label in spec["setup"]:
        r = T.run_block(task, planner, ctx, index[label], blocks[label])
        if not r["success"]:
            return {"scenario": name, "seed": seed, "pass": False,
                    "reason": f"setup block {label!r} failed: {r['message']}"}  # fmt: skip
    setup_seconds = round(time.time() - t0, 2)
    recorded = RecordedFacts(None, predicates=T.RECORDED_PREDICATES)
    recorded.apply([Literal.parse(fact) for fact in spec["facts"]])
    result = T.run_mission(
        task, planner, n_parts, max_replans=max_replans, q_start=ctx["q"],
        recorded=recorded,
    )  # fmt: skip
    planned = [b["label"] for b in result["blocks"]]
    redone = [
        label for label in spec["setup"] if label in planned and "home" not in label
    ]
    missing = [label for label in spec["skip"] if label not in result["skipped"]]
    passed = result["success"] and not redone and not missing
    reason = "" if passed else (
        "mission failed" if not result["success"]
        else f"redone: {redone}" if redone else f"not skipped: {missing}"
    )  # fmt: skip
    return {
        "scenario": name,
        "seed": seed,
        "pass": passed,
        "reason": reason,
        "setup_seconds": setup_seconds,
        "mission_seconds": result["seconds"],
        "skipped": result["skipped"],
        "planned": planned,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scenario", choices=sorted(SCENARIOS))
    ap.add_argument("--all", action="store_true", help="run every scenario")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--json", type=Path, help="write the results to this file")
    args = ap.parse_args()
    if args.scenario:
        result = run_scenario(args.scenario, args.seed)
        print("SCENARIO_RESULT " + json.dumps(result))
        return 0 if result["pass"] else 1
    if not args.all:
        ap.error("pass --scenario NAME or --all")
    results = []
    for name in SCENARIOS:  # one process each: a fresh HPP scene per scenario
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
            f"{'PASS' if result['pass'] else 'FAIL'}  {name:<18} "
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
