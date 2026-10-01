#!/usr/bin/env python3
"""The M6 exit test (#91): an instruction runs to the end without a human.

For each seed, ``task_screw_assembly.py`` gets one instruction, plans it with
the model roles (grounder, goal writer, plan reviewer), and runs it in MuJoCo
under the execution supervisor (``--supervise``), with a part's clamping
failure injected (part 1 on odd seeds, part 2 on even ones). On the
two-clamp scene (one clamp per part), repair can't route around it, so the
mission needs a goal-level decision. Nobody answers anything.

Each run is classified:

- **completed**: the mission succeeded (the supervisor relaxed the goal, or
  a retry worked);
- **escalated**: it stopped cleanly, with ``escalation.md`` in its folder;
- **unclean**: anything else (a crash, a stop without a report).

and audited: every supervisor decision is an allowed action, and every
relaxed goal is a strict subset of the original goal. The gate passes when
no run is unclean and no decision is unchecked.

    python3 build_scene.py --parts 2 --clamps 2        # in a scratch copy
    python3 autonomy_batch.py --seeds 10 --par 2 --env ~/devel/hpp/.anthropic/env \\
        --model openai:my-model --out autonomy --json autonomy.json

Run inside the planning container.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
INSTRUCTION = "assemble both parts and rack the driver"
ACTIONS = {"retry", "relax_goal", "abort", "escalate"}


def run_seed(seed: int, args: argparse.Namespace) -> dict[str, Any]:
    out = Path(args.out).resolve()
    run_dir = out / f"seed{seed:03d}"
    summary = out / f"seed_{seed:03d}.json"
    part = "part1" if seed % 2 else "part2"
    cmd = [
        sys.executable,
        "-u",
        str(HERE / "task_screw_assembly.py"),
        "--seed",
        str(seed),
        "--no-viewer",
        "--backend",
        args.backend,
        "--ai-env",
        str(args.env),
        "--goal-model",
        args.model,
        "--instruction",
        args.instruction,
        "--supervise",
        "--inject-failure",
        f"clamp_and_screw:part={part}",
        "--run-dir",
        str(run_dir),
        "--summary",
        str(summary),
    ]
    t0 = time.monotonic()
    with open(out / f"seed_{seed:03d}.log", "w") as log:
        code = subprocess.call(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=HERE)
    record: dict[str, Any] = {
        "seed": seed,
        "injected": part,
        "exit": code,
        "wall_s": round(time.monotonic() - t0, 1),
    }
    try:
        result = json.loads(summary.read_text())
    except (OSError, json.JSONDecodeError):
        return {**record, "outcome": "unclean", "why": "no summary"}
    sup = result.get("supervisor") or {}
    report = run_dir / "escalation.md"
    if result.get("success"):
        outcome = "completed"
    elif sup.get("stopped") and report.exists():
        outcome = "escalated"
    else:
        outcome = "unclean"
    record.update(
        outcome=outcome,
        mission_s=result.get("seconds"),
        stopped=sup.get("stopped"),
        repair_loops=sup.get("repair_loops"),
        decisions=[d["action"] for d in sup.get("decisions", [])],
        final_goal=sup.get("final_goal"),
        unchecked=audit(sup),
        **model_calls(run_dir / "events.jsonl"),
    )
    return record


def audit(sup: dict[str, Any]) -> list[str]:
    """Supervisor decisions that break the rules (empty: none)."""
    problems = []
    original = set(sup.get("original_goal") or [])
    for i, d in enumerate(sup.get("decisions", []), 1):
        if d["action"] not in ACTIONS:
            problems.append(f"decision {i}: unknown action {d['action']!r}")
        if d["action"] == "relax_goal":
            goal = set(d.get("goal") or [])
            if not goal or not goal < original:
                problems.append(f"decision {i}: relaxed goal is not a strict subset")
    return problems


def model_calls(events: Path) -> dict[str, Any]:
    calls: dict[str, int] = {}
    tokens = errors = 0
    seconds = 0.0
    try:
        lines = events.read_text().splitlines()
    except OSError:
        lines = []
    for line in lines:
        event = json.loads(line)
        if event.get("role") != "model":
            continue
        calls[event["name"]] = calls.get(event["name"], 0) + 1
        m = event.get("metrics") or {}
        tokens += (m.get("input_tokens") or 0) + (m.get("output_tokens") or 0)
        seconds += m.get("seconds") or 0.0
        errors += event.get("status") == "FAILURE"
    return {
        "model_calls": calls,
        "model_tokens": tokens,
        "model_seconds": round(seconds, 1),
        "model_errors": errors,
    }


def summarize(records: list[dict[str, Any]]) -> dict[str, Any]:
    n = len(records)
    by = {
        k: sum(r["outcome"] == k for r in records)
        for k in ("completed", "escalated", "unclean")
    }
    unchecked = [p for r in records for p in r.get("unchecked", [])]
    actions: dict[str, int] = {}
    for r in records:
        for a in r.get("decisions", []):
            actions[a] = actions.get(a, 0) + 1
    return {
        "runs": n,
        **by,
        "decisions": actions,
        "unchecked_decisions": unchecked,
        "model_calls": sum(sum(r.get("model_calls", {}).values()) for r in records),
        "model_tokens": sum(r.get("model_tokens", 0) for r in records),
        "gate": "PASS" if by["unclean"] == 0 and not unchecked and n else "FAIL",
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--first", type=int, default=1)
    ap.add_argument("--par", type=int, default=2)
    ap.add_argument("--env", type=Path, required=True)
    ap.add_argument("--model", required=True)
    ap.add_argument("--instruction", default=INSTRUCTION)
    ap.add_argument("--backend", default="mujoco")
    ap.add_argument("--out", type=Path, default=Path("autonomy"))
    ap.add_argument("--json", type=Path)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    seeds = range(args.first, args.first + args.seeds)
    with ThreadPoolExecutor(args.par) as pool:
        records = sorted(
            pool.map(lambda s: run_seed(s, args), seeds), key=lambda r: r["seed"]
        )
    for r in records:
        print(
            f"seed {r['seed']:3d} ({r['injected']} fails): {r['outcome']:9s} "
            f"decisions {r.get('decisions')} calls {sum(r.get('model_calls', {}).values())} "
            f"tokens {r.get('model_tokens')} mission {r.get('mission_s')} s"
            + (f"  UNCHECKED {r['unchecked']}" if r.get("unchecked") else "")
            + (
                f"  ({r.get('why') or r.get('stopped')})"
                if r["outcome"] != "completed"
                else ""
            ),
            flush=True,
        )
    summary = summarize(records)
    print(json.dumps(summary, indent=2))
    if args.json:
        args.json.write_text(
            json.dumps({"summary": summary, "runs": records}, indent=2)
        )
    return 0 if summary["gate"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
