#!/usr/bin/env python3
"""Aggregate run_batch.sh's per-seed summaries into mission statistics.

    python3 summarize.py batch/            # reads batch/seed_*.json

Metrics (the same definitions as the agimus_spacelab mission audit):

- replanning trigger rate: planning blocks with >= 1 replan-from-entry,
  over all planning blocks (home moves excluded -- they never replan);
- recovery rate: blocks and moves that hit any failure (a resume, a retry
  or a replan) and still finished, over all such episodes.
"""

from __future__ import annotations

import json
import statistics as st
import sys
from pathlib import Path


def main() -> int:
    folder = Path(sys.argv[1] if len(sys.argv) > 1 else "batch")
    runs = [json.loads(p.read_text()) for p in sorted(folder.glob("seed_*.json"))]
    if not runs:
        print(f"no seed_*.json in {folder}")
        return 1
    blocks = [b for r in runs for b in r["blocks"]]
    planning = [b for b in blocks if "home" not in b["label"]]
    replanned = [b for b in planning if b["replans"] > 0]
    episodes = [b for b in blocks if b["replans"] > 0 or b["resumes"] > 0]
    recovered = [b for b in episodes if b["success"]]
    ok = [r for r in runs if r["success"]]
    secs = sorted(r["seconds"] for r in ok)

    def pct(a, b):
        return f"{100 * a / b:.1f}% ({a}/{b})" if b else "n/a"

    print(f"runs: {len(runs)} (parts: {sorted({r['parts'] for r in runs})})")
    print(f"missions completed:      {pct(len(ok), len(runs))}")
    print(
        f"replanning trigger rate: {pct(len(replanned), len(planning))}  [target < 2%]"
    )
    print(
        f"recovery rate:           {pct(len(recovered), len(episodes))}  [target > 95%]"
    )
    if secs:
        print(
            f"mission time (s):        median {st.median(secs):.0f}, "
            f"min {secs[0]:.0f}, max {secs[-1]:.0f}"
        )
    print("\nper run:")
    for r in runs:
        rp = sum(b["replans"] for b in r["blocks"])
        rs = sum(b["resumes"] for b in r["blocks"])
        print(
            f"  seed {r['seed']:>3}  {'ok  ' if r['success'] else 'FAIL'} "
            f"{r['seconds']:>7.0f}s  blocks {len(r['blocks']):>2}  "
            f"resumes {rs:>3}  replans {rp:>2}"
        )
    slow = sorted(blocks, key=lambda b: -b["seconds"])[:5]
    print("\nslowest blocks:")
    for b in slow:
        print(
            f"  {b['seconds']:>6.0f}s  {b['label']}  "
            f"(resumes {b['resumes']}, replans {b['replans']})"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
