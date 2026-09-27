#!/usr/bin/env python3
"""Aggregate run_batch.sh's per-seed summaries into mission statistics.

    python3 summarize.py batch/                          # report only
    python3 summarize.py batch/ --json results/x.json    # also record a result file
    python3 summarize.py batch/ --gate \\
        --baseline results/pypi-wheel-batch-2026-09-26.json   # pass/fail gate

Metrics (the same definitions as the agimus_spacelab mission audit):

- replanning trigger rate: planning blocks with >= 1 replan-from-entry,
  over all planning blocks (home moves excluded -- they never replan);
- recovery rate: blocks and moves that hit any failure (a resume, a retry
  or a replan) and still finished, over all such episodes.

``--gate`` exits non-zero unless every mission completed, the replanning
trigger rate is below ``--max-replan-rate``, the recovery rate is above
``--min-recovery-rate``, and (with ``--baseline``) the median mission time is
at most ``--max-slowdown`` times the baseline's. This is the batch gate in
docs/development/validation.md.
"""

from __future__ import annotations

import argparse
import json
import statistics as st
from pathlib import Path
from typing import Any


def compute_stats(runs: list[dict[str, Any]]) -> dict[str, Any]:
    """Batch statistics, in the schema of the committed results/*.json files."""
    blocks = [b for r in runs for b in r["blocks"]]
    planning = [b for b in blocks if "home" not in b["label"]]
    replanned = [b for b in planning if b["replans"] > 0]
    episodes = [b for b in blocks if b["replans"] > 0 or b["resumes"] > 0]
    recovered = [b for b in episodes if b["success"]]
    ok = [r for r in runs if r["success"]]
    secs = sorted(r["seconds"] for r in ok)
    return {
        "parts": sorted({r["parts"] for r in runs}),
        "completed": len(ok),
        "attempted": len(runs),
        "replanning_blocks": len(replanned),
        "planning_blocks": len(planning),
        "recovered_failures": len(recovered),
        "failure_episodes": len(episodes),
        "seconds_median": round(st.median(secs), 2) if secs else None,
        "seconds_min": secs[0] if secs else None,
        "seconds_max": secs[-1] if secs else None,
        "seeds": [
            {
                "seed": r["seed"],
                "success": r["success"],
                "seconds": r["seconds"],
                "blocks": len(r["blocks"]),
                "resumes": sum(b["resumes"] for b in r["blocks"]),
                "replans": sum(b["replans"] for b in r["blocks"]),
            }
            for r in runs
        ],
    }


def check_gate(
    stats: dict[str, Any],
    baseline: dict[str, Any] | None = None,
    max_replan_rate: float = 0.02,
    min_recovery_rate: float = 0.95,
    max_slowdown: float = 1.25,
) -> list[str]:
    """Return the gate's failures; an empty list means the batch passes."""
    failures = []
    if stats["attempted"] == 0:
        return ["no runs"]
    if stats["completed"] < stats["attempted"]:
        failures.append(f"missions completed {stats['completed']}/{stats['attempted']}")
    if stats["planning_blocks"]:
        rate = stats["replanning_blocks"] / stats["planning_blocks"]
        if rate >= max_replan_rate:
            failures.append(
                f"replanning trigger rate {rate:.1%} >= {max_replan_rate:.0%}"
            )
    if stats["failure_episodes"]:
        rate = stats["recovered_failures"] / stats["failure_episodes"]
        if rate <= min_recovery_rate:
            failures.append(f"recovery rate {rate:.1%} <= {min_recovery_rate:.0%}")
    if baseline and baseline.get("seconds_median") and stats["seconds_median"]:
        limit = max_slowdown * baseline["seconds_median"]
        if stats["seconds_median"] > limit:
            failures.append(
                f"median mission time {stats['seconds_median']:.0f}s > "
                f"{max_slowdown}x baseline ({baseline['seconds_median']:.0f}s)"
            )
    return failures


def print_report(runs: list[dict[str, Any]], stats: dict[str, Any]) -> None:
    def pct(a, b):
        return f"{100 * a / b:.1f}% ({a}/{b})" if b else "n/a"

    print(f"runs: {stats['attempted']} (parts: {stats['parts']})")
    print(f"missions completed:      {pct(stats['completed'], stats['attempted'])}")
    print(
        "replanning trigger rate: "
        f"{pct(stats['replanning_blocks'], stats['planning_blocks'])}  [target < 2%]"
    )
    print(
        "recovery rate:           "
        f"{pct(stats['recovered_failures'], stats['failure_episodes'])}"
        "  [target > 95%]"
    )
    if stats["seconds_median"] is not None:
        print(
            f"mission time (s):        median {stats['seconds_median']:.0f}, "
            f"min {stats['seconds_min']:.0f}, max {stats['seconds_max']:.0f}"
        )
    print("\nper run:")
    for s in stats["seeds"]:
        print(
            f"  seed {s['seed']:>3}  {'ok  ' if s['success'] else 'FAIL'} "
            f"{s['seconds']:>7.0f}s  blocks {s['blocks']:>2}  "
            f"resumes {s['resumes']:>3}  replans {s['replans']:>2}"
        )
    blocks = [b for r in runs for b in r["blocks"]]
    print("\nslowest blocks:")
    for b in sorted(blocks, key=lambda b: -b["seconds"])[:5]:
        print(
            f"  {b['seconds']:>6.0f}s  {b['label']}  "
            f"(resumes {b['resumes']}, replans {b['replans']})"
        )


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("folder", type=Path, nargs="?", default=Path("batch"))
    ap.add_argument("--json", type=Path, help="write the statistics to this file")
    ap.add_argument("--gate", action="store_true", help="exit 1 if the gate fails")
    ap.add_argument("--baseline", type=Path, help="results/*.json to compare against")
    ap.add_argument("--max-replan-rate", type=float, default=0.02)
    ap.add_argument("--min-recovery-rate", type=float, default=0.95)
    ap.add_argument("--max-slowdown", type=float, default=1.25)
    args = ap.parse_args(argv)

    paths = sorted(args.folder.glob("seed_*.json"))
    runs = [json.loads(p.read_text()) for p in paths]
    if not runs:
        print(f"no seed_*.json in {args.folder}")
        return 1
    stats = compute_stats(runs)
    print_report(runs, stats)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(stats, indent=1) + "\n")
        print(f"\nwrote {args.json}")
    if args.gate:
        baseline = json.loads(args.baseline.read_text()) if args.baseline else None
        failures = check_gate(
            stats,
            baseline,
            args.max_replan_rate,
            args.min_recovery_rate,
            args.max_slowdown,
        )
        print("\ngate: " + ("PASS" if not failures else "FAIL"))
        for f in failures:
            print(f"  - {f}")
        return 1 if failures else 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
