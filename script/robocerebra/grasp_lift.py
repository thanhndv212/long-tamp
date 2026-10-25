#!/usr/bin/env python3
"""Grasp and lift one object of every class in the benchmark env (#158 AC2).

Runs in the HPP environment and starts the benchmark server in its Python
3.10 environment. For each class in the R3 handle inventory
(``grasp_library.py``'s results), its example instance's case is reset at
step 0 with the goal ``pick <instance>`` (``set_goal``), the pick is planned
in HPP on the exported scene (:class:`~long_tamp.eval.robocerebra_policy.HppPolicy`:
the generated handles, then a straight lift) and tracked with ``OSC_POSE``
actions until the benchmark's pick checker passes or the step budget runs
out. The checker passes once the object has left every region with its
orientation within the benchmark's threshold over the last steps; it prints
``pick suppressed: quaternion change`` when the object turned too much.
An object that starts in no region, or only in its own (``start_regions``),
can never leave one: its row is ``unregistrable`` and records how high the
object was ``lifted``, the grasp's evidence instead of the checker's.

    python script/robocerebra/grasp_lift.py
    python script/robocerebra/grasp_lift.py --classes butter popcorn

The result file is rewritten after every class; ``--resume`` keeps the classes
already in it, so a run stopped part-way can continue. ``--at CLASS=CASE[:INSTANCE]`` tries a class
at another instance than the inventory's example (default ``<class>_1``),
for one whose example sits where the arm cannot reach it; the earlier rows
of that class are kept under ``attempts``.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from replay_demo import server_command  # noqa: E402

from long_tamp.eval import RoboCerebraClient  # noqa: E402
from long_tamp.eval.robocerebra_policy import HppPolicy  # noqa: E402

INVENTORY = HERE / "results" / "2026-10-25-r3-grasp-handles.json"


def grasp_lift(
    env: RoboCerebraClient, policy: HppPolicy, case: str, instance: str, seed: int, step_limit: int
) -> dict[str, Any]:
    """One pick of ``instance`` from ``case``'s start, as a one-step task."""
    goal = env.set_goal({instance: [["pick", instance]]})
    obs = env.reset(step=0, seed=seed)["obs"]
    start_regions = env.regions(instance)
    started = time.time()
    controller = policy(env, goal, 0, step_limit)
    plan_seconds = time.time() - started
    monitor: list[str] = []
    result: dict[str, Any] = {"all_done": False}
    used = 0
    while used < step_limit and not result["all_done"]:
        result = env.step(np.asarray(controller(obs), dtype=float))
        obs = result["obs"]
        monitor.extend(result.get("monitor", []))
        used += 1
    info = dict(controller.info)
    info.pop("handles", None)  # in the inventory already
    start, end = info.get("target_start"), info.get("target_end")
    return {
        "case": case,
        "instance": instance,
        "seed": seed,
        "success": bool(result["all_done"]),
        "start_regions": start_regions,
        "unregistrable": not any(not r.startswith(f"{instance}_") for r in start_regions),
        "lifted": round(end[2] - start[2], 4) if start and end else None,
        "steps_used": used,
        "suppressed": [line for line in monitor if "suppressed" in line],
        "plan_seconds": round(plan_seconds, 3),
        "monitor": monitor,
        "policy": info,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--inventory", default=str(INVENTORY), help="grasp_library.py results")
    parser.add_argument("--classes", nargs="*", help="default: every class in the inventory")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--step-limit", type=int, default=150)
    parser.add_argument("--env-sh", default="~/devel/hpp/robocerebra/env.sh")
    parser.add_argument("--bench", default=None)
    parser.add_argument("--work-dir", default="/tmp/robocerebra-grasp-lift")
    parser.add_argument("--resume", action="store_true", help="skip classes already in --out")
    parser.add_argument("--at", nargs="*", default=[], metavar="CLASS=CASE[:INSTANCE]",
                        help="try a class at another instance, amending --out")
    parser.add_argument(
        "--out", default=str(HERE / "results" / "2026-10-25-r3-grasp-lift.json"), help="JSON result path"
    )
    args = parser.parse_args(argv)

    classes = json.loads(Path(args.inventory).read_text())["classes"]
    unknown = sorted(set(args.classes or []) - set(classes))
    if unknown:
        parser.error(f"not in the inventory: {unknown}")
    at: dict[str, tuple[str, str]] = {}
    for spec in args.at:
        name, _, where = spec.partition("=")
        case, _, instance = where.partition(":")
        if name not in classes or not case:
            parser.error(f"--at {spec}: expected CLASS=CASE[:INSTANCE] with a class in the inventory")
        at[name] = (case, instance or f"{name}_1")
    by_case: dict[str, list[tuple[str, str]]] = defaultdict(list)
    for name in args.classes or sorted(at or classes):
        example = classes[name]["example"]
        case, instance = at.get(name, (example["case"], example["instance"]))
        by_case[case].append((name, instance))

    out = Path(args.out)
    rows: dict[str, dict[str, Any]] = {}
    if (args.resume or at) and out.exists():  # --at amends an earlier run
        rows = json.loads(out.read_text())["classes"]
    command = server_command(args.env_sh, args.bench, no_render=True)
    with RoboCerebraClient.spawn(command) as env:
        for case, instances in sorted(by_case.items()):
            instances = [(name, instance) for name, instance in instances if name in at or name not in rows]
            if not instances:
                continue
            env.load(case)
            for name, instance in instances:
                policy = HppPolicy(work_dir=Path(args.work_dir) / case / instance)
                try:
                    row = grasp_lift(env, policy, case, instance, args.seed, args.step_limit)
                except Exception as error:
                    row = {"case": case, "instance": instance, "success": False,
                           "error": f"{type(error).__name__}: {error}"}
                if name in at and name in rows:
                    earlier = rows[name]
                    row["attempts"] = [*earlier.pop("attempts", []), earlier]
                rows[name] = row
                write(out, rows, args.step_limit)
                status = "OK" if row["success"] else "FAIL"
                detail = row.get("error") or row["policy"].get("error") or row["policy"].get("stage")
                print(f"{name:32s} {status:4s} {row.get('steps_used', '-'):>4} {case} {instance} {detail}",
                      flush=True)

    result = write(out, rows, args.step_limit)
    print(json.dumps(result["summary"], indent=1))
    print(f"wrote {out}")
    return 1 if result["summary"]["failed"] else 0


def write(out: Path, rows: dict[str, dict[str, Any]], step_limit: int) -> dict[str, Any]:
    failed = sorted(name for name, row in rows.items() if not row["success"])
    result = {
        "protocol": "reset(step=0, seed); goal = pick <instance>; HppPolicy pick + lift; OSC_POSE",
        "step_limit": step_limit,
        "classes": dict(sorted(rows.items())),
        "summary": {
            "classes": len(rows),
            "picked": len(rows) - len(failed),
            "suppressed": sorted(name for name, row in rows.items() if row.get("suppressed")),
            "unregistrable": sorted(name for name, row in rows.items() if row.get("unregistrable")),
            "failed": failed,
        },
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=1) + "\n")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
