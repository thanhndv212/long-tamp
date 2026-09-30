#!/usr/bin/env python3
"""The #21 exit test on identical plans: sequential vs concurrent motion.

Plans the mission once (``--planner up``, independent steps in parallel
lanes) without executing it, keeping every step's motion. Then runs the same
motions twice on fresh MuJoCo backends: one command after another, as a
sequential executor would, and with each parallel group merged, as
``PlanExecutor(concurrent=True)`` does. Planning is shared, so the two
missions' wall-clock differs exactly by their motion time, which is
simulated time (what a real-time robot takes).

    python3 concurrency_ab.py --seed 1 --json ab_seed1.json

Run inside the planning container, after ``build_scene.py``.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import task_screw_assembly as T  # noqa: E402

from long_tamp.execution import ExecutionPolicy, ExecutionStatus  # noqa: E402
from long_tamp.execution.concurrent import merge_lanes  # noqa: E402
from long_tamp.execution.executor import PlanExecutor  # noqa: E402
from long_tamp.execution.sampled import sampled  # noqa: E402
from long_tamp.execution.supervisor import run_command  # noqa: E402
from long_tamp.tasks.task_planning.world_state import RecordedFacts  # noqa: E402


class _Recorder:
    """Stands in for a backend while planning: nothing runs."""


class RecordingExecutor(PlanExecutor):
    """Plans the mission, keeping each unit's motion instead of running it:
    a step's commands, or a parallel group's (sequential and merged)."""

    def __init__(self, session, *args, **kwargs):
        kwargs["backend"] = _Recorder()
        kwargs["plan_ahead"] = False
        super().__init__(session, *args, **kwargs)
        self.units: list[dict[str, Any]] = []

    def _run_motion(self, nodes, report, commands):
        sampled_commands = [
            type(c)(c.step_id, c.duration, sampled(c.payload)) for c in commands
        ]
        if self._pending_group is not None:
            unit, self._pending_group = self._pending_group, None
            unit["merged"] = sampled_commands if unit["merged"] else None
        else:
            unit = {"label": report.get("label", report["id"]), "merged": None}
            unit["group"] = False
            unit["sequential"] = sampled_commands
        self.units.append(unit)
        for done in nodes:
            self._commit(done)
        return None

    _pending_group = None

    def _on_group(self, node, ok):
        if ok is None or self._group is None:
            return super()._on_group(node, ok)
        planned = self._group["planned"]
        lanes: dict[int, list] = {}
        for step, commands in planned:
            lanes.setdefault(self._group["lane_of"][step["id"]], []).extend(commands)
        merged = merge_lanes([lanes[k] for k in sorted(lanes)], self.validate_config)
        self._pending_group = {
            "label": node.get("label", node["id"]),
            "sequential": [
                type(c)(c.step_id, c.duration, sampled(c.payload))
                for _, cs in planned
                for c in cs
            ],
            "merged": merged is not None,
            "group": True,
        }
        return super()._on_group(node, ok)


def replay(task, run_dir: Path, units, merged: bool) -> dict[str, float]:
    """Run the units' motions on a fresh MuJoCo backend: simulated seconds."""
    backend = T.make_backend("mujoco", task, None, run_dir, speed=math.inf)
    sim = 0.0
    try:
        for unit in units:
            commands = (
                unit["merged"] if merged and unit["merged"] else unit["sequential"]
            )
            for command in commands:
                result = run_command(backend, command, ExecutionPolicy(poll_interval=0))
                if result.status is not ExecutionStatus.SUCCESS:
                    return {"success": False, "motion_s": sim, "failed": result.message}
                sim += float(result.metrics.get("sim_seconds", 0.0))
    finally:
        backend.close()
    return {"success": True, "motion_s": round(sim, 3)}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--json", type=Path)
    args = ap.parse_args()

    run_dir = HERE / "runs" / f"ab_seed{args.seed}_{int(time.time())}"
    run_dir.mkdir(parents=True, exist_ok=True)
    T.seed_everything(args.seed)
    task, planner = T.setup(log_dir=str(run_dir))
    n_parts = sum(1 for o in task.task_config.OBJECTS if o.startswith("part"))
    recorded = RecordedFacts(None, predicates=T.RECORDED_PREDICATES)
    document = T.plan_from_goal(
        n_parts,
        T.world_atoms(planner, recorded),
        clamps=T.clamp_seats(task.task_config.VALID_PAIRS),
    )
    recorder: dict[str, RecordingExecutor] = {}

    def make(session, **kwargs):
        recorder["x"] = RecordingExecutor(session, **kwargs)
        return recorder["x"]

    T.PlanExecutor = make  # run_mission builds its executor through this name
    t0 = time.time()
    result = T.run_mission(
        task,
        planner,
        n_parts,
        verbose=False,
        recorded=recorded,
        document=document,
        concurrent=True,
        backend=_Recorder(),
    )
    planning = time.time() - t0
    if not result["success"]:
        print("planning failed:", result.get("failure"))
        return 1
    units = recorder["x"].units
    seq = replay(task, run_dir / "seq", units, merged=False)
    conc = replay(task, run_dir / "conc", units, merged=True)
    out = {
        "seed": args.seed,
        "planning_s": round(planning, 1),
        "groups": sum(1 for u in units if u["group"]),
        "merged_groups": sum(1 for u in units if u["merged"]),
        "sequential": seq,
        "concurrent": conc,
        "wall_s": {
            "sequential": round(planning + seq["motion_s"], 1),
            "concurrent": round(planning + conc["motion_s"], 1),
        },
    }
    print(json.dumps(out, indent=2))
    if args.json:
        args.json.write_text(json.dumps(out, indent=2))
    return 0 if seq["success"] and conc["success"] else 1


if __name__ == "__main__":
    sys.exit(main())
