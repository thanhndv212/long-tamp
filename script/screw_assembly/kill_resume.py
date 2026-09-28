#!/usr/bin/env python3
"""Kill the screw-assembly mission mid-run, restart it, check it resumes (#12).

Runs ``task_screw_assembly.py`` in a fresh run folder, watches its event stream
(``events.jsonl``) and kills it with SIGKILL -- no cleanup, like a crash or an
OOM kill -- either while a given step is being planned (``--kill-during``, the
default, after ``--delay`` seconds) or right after a step completes
(``--kill-after``). Then it restarts the mission with ``--resume`` in the same
folder and checks, from the restarted run's events:

- the mission completes;
- no step completed before the kill is planned again: each is skipped because
  its effect holds in the world (grasp tracker + recorded facts), or because
  a guard condition skips the part it belongs to. Home moves declare no
  effects, so they run again by design (cheap); they are reported, not failed.

Usage (in the HPP environment, scene built with ``build_scene.py``)::

    python kill_resume.py --seed 1                          # kill while b03 plans
    python kill_resume.py --seed 1 --kill-after b03-clamp_and_screw

Exit code 0 when the check passes; a JSON summary is printed (``--json`` also
writes it to a file).
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

HERE = Path(__file__).parent


def _events(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    events = []
    for line in path.read_text().splitlines():
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            break  # a line cut by the kill: nothing after it was written
    return events


def _mission(run_dir: Path, seed: int, backend: str, resume: bool) -> list[str]:
    command = [
        sys.executable,
        "-u",
        str(HERE / "task_screw_assembly.py"),
        "--seed",
        str(seed),
        "--run-dir",
        str(run_dir),
        "--no-viewer",
        "--backend",
        backend,
    ]
    return command + (["--resume"] if resume else [])


def _trigger(args: argparse.Namespace):
    """A predicate on events: True once the kill point is reached."""
    if args.kill_after:
        return lambda e: (e["ir_id"], e["role"], e["status"]) == (
            args.kill_after,
            "transaction",
            "SUCCESS",
        )
    return lambda e: (e["ir_id"], e["role"], e["status"]) == (
        args.kill_during,
        "attempts",
        "RUNNING",
    )


def first_run(args: argparse.Namespace, run_dir: Path) -> dict[str, Any]:
    """Run the mission until the kill point, then SIGKILL it."""
    events_path = run_dir / "events.jsonl"
    log = open(run_dir / "first.log", "w")
    process = subprocess.Popen(
        _mission(run_dir, args.seed, args.backend, resume=False),
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,  # its own process group, killed as a whole
    )
    reached = _trigger(args)
    deadline = time.time() + args.timeout
    fired_at = None
    while process.poll() is None and time.time() < deadline:
        if fired_at is None and any(reached(e) for e in _events(events_path)):
            fired_at = time.time()
        delay = args.delay if not args.kill_after else 0.0
        if fired_at is not None and time.time() >= fired_at + delay:
            os.killpg(process.pid, signal.SIGKILL)
            break
        time.sleep(0.2)
    code = process.wait()
    log.close()
    return {"exit": code, "killed": code == -signal.SIGKILL}


def check_resume(
    before: list[dict[str, Any]], after: list[dict[str, Any]]
) -> dict[str, Any]:
    """Compare what completed before the kill with what the restart planned."""
    completed = [
        e["ir_id"]
        for e in before
        if e["role"] == "transaction" and e["status"] == "SUCCESS"
    ]
    planned_again = sorted(
        {e["ir_id"] for e in after if e["role"] == "execute"} & set(completed)
    )
    homes = [s for s in planned_again if s.endswith("-home")]
    redone = [s for s in planned_again if s not in homes]
    skipped = sorted(
        {
            e["ir_id"]
            for e in after
            if e["role"] == "complete" and e["status"] == "SUCCESS"
        }
    )
    guards = sorted(
        {
            e["ir_id"]
            for e in after
            if e["role"] == "condition" and e["status"] == "SUCCESS"
        }
    )
    finished = bool(after) and (after[-1]["role"], after[-1]["status"]) == (
        "sequence",
        "SUCCESS",
    )
    return {
        "completed_before_kill": completed,
        "skipped_by_effect": [s for s in skipped if s in completed],
        "skipped_by_guard": guards,
        "planned_again": redone,
        "homes_rerun": homes,
        "resumed_run_completed": finished,
        "pass": finished and not redone,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--seed", type=int, default=1)
    kill = parser.add_mutually_exclusive_group()
    kill.add_argument("--kill-during", default="b03-clamp_and_screw")
    kill.add_argument("--kill-after", default=None)
    parser.add_argument("--delay", type=float, default=5.0,
                        help="seconds into the step before --kill-during kills")  # fmt: skip
    parser.add_argument("--backend", default="mock")
    parser.add_argument("--timeout", type=float, default=1800.0)
    parser.add_argument("--run-dir", type=Path, default=None)
    parser.add_argument("--json", type=Path, default=None)
    args = parser.parse_args()

    run_dir = args.run_dir or HERE / "runs" / f"kill_resume_seed{args.seed}_{time.strftime('%Y%m%d_%H%M%S')}"
    run_dir.mkdir(parents=True, exist_ok=True)
    point = f"after {args.kill_after}" if args.kill_after else (
        f"{args.delay:.0f}s into {args.kill_during}"
    )  # fmt: skip
    print(f"run folder: {run_dir}\nkilling {point}", flush=True)

    t0 = time.time()
    first = first_run(args, run_dir)
    before = _events(run_dir / "events.jsonl")
    summary: dict[str, Any] = {"seed": args.seed, "kill": point, **first}
    if not first["killed"]:
        summary.update(pass_=False, reason="the kill point was never reached")
        print(json.dumps(summary, indent=2))
        return 1
    print(f"killed after {time.time() - t0:.0f}s, {len(before)} events; resuming",
          flush=True)  # fmt: skip

    t1 = time.time()
    with open(run_dir / "resume.log", "w") as log:
        code = subprocess.call(
            _mission(run_dir, args.seed, args.backend, resume=True),
            stdout=log,
            stderr=subprocess.STDOUT,
        )
    after = _events(run_dir / "events.jsonl")[len(before) :]
    summary.update(resume_exit=code, resume_seconds=round(time.time() - t1, 1))
    summary.update(check_resume(before, after))
    summary["pass"] = summary["pass"] and code == 0
    print(json.dumps(summary, indent=2))
    if args.json:
        args.json.write_text(json.dumps(summary, indent=2) + "\n")
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
