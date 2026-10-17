"""Replay a RoboCerebra demo through the env server; measure drift (#156).

Three checks, all through :class:`long_tamp.eval.RoboCerebraClient`:

0. **Recorded**: set each recorded state in turn and run the checker. What
   the demo itself achieves against the case's goal, with no replay at all.
1. **Continuous replay**: reset to step 0 (no settling: the demo was recorded
   without it) and apply every demo action in order. Is the simulation
   deterministic enough that the replay passes the benchmark's checker, and
   how far does it drift from the recorded states?
2. **Resumed replay**: each benchmark step ``k`` from the demo's state at
   its start, replaying that step's actions. How far is it from the demo at
   its end, and does it complete step ``k``'s subtasks? Three ways: the
   benchmark's resume chain (``resume(k)`` mid-episode), a fresh episode per
   step, and a fresh episode restoring the demo's controller and gripper
   commands.

Drift is the end-effector and per-object position error against the demo's
own state at the same index (the reference observations are computed first by
setting each recorded state), plus the largest joint-coordinate difference.

Run in the HPP env (the server runs in the benchmark env)::

    python script/robocerebra/replay_demo.py Ideal/case1 \
        --out script/robocerebra/results/<date>-r1-replay-ideal-case1.json
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import time
from pathlib import Path
from typing import Any

import numpy as np

from long_tamp.eval import RoboCerebraClient

HERE = Path(__file__).resolve().parent
DEFAULT_ENV_SH = "~/devel/hpp/robocerebra/env.sh"


def server_command(env_sh: str, bench: str | None, no_render: bool) -> list[str]:
    # The HPP env's PYTHONPATH / LD_LIBRARY_PATH would shadow the benchmark
    # env's numpy<2 and libraries: start the server from a clean slate.
    args = ['"$0"', '"$@"']
    script = (
        "unset PYTHONPATH LD_LIBRARY_PATH; "
        f"source {env_sh} && exec python -I {' '.join(args)}"
    )
    command = ["bash", "-c", script, str(HERE / "env_server.py")]
    if bench:
        command += ["--bench", bench]
    if no_render:
        command.append("--no-render")
    return command


def positions(obs: dict[str, Any], objects: list[str]) -> dict[str, np.ndarray]:
    out = {"eef": np.asarray(obs["robot0_eef_pos"])}
    for name in objects:
        if f"{name}_pos" in obs:
            out[name] = np.asarray(obs[f"{name}_pos"])
    return out


def drift(
    obs: dict[str, Any], ref: dict[str, np.ndarray], objects: list[str]
) -> dict[str, float]:
    here = positions(obs, objects)
    errors = {k: float(np.linalg.norm(here[k] - ref[k])) for k in ref if k in here}
    objs = [v for k, v in errors.items() if k != "eef"]
    return {
        "eef_m": errors["eef"],
        "max_object_m": max(objs) if objs else 0.0,
        "objects_m": {k: v for k, v in errors.items() if k != "eef"},
    }


def replay_case(env: RoboCerebraClient, case: str, args: argparse.Namespace) -> dict[str, Any]:
    started = time.monotonic()
    info = env.load(case, model=args.model)
    objects, spans = info["objects"], info["spans"]
    demo = env.demo()
    states, actions = demo["states"], demo["actions"]
    length = len(actions)
    print(f"{case}: {info['task']!r}, {length} demo steps, spans {spans}")

    # Reference observations at the sample points and segment boundaries.
    marks = sorted(
        set(range(0, length, args.every))
        | {s for s, _ in spans}
        | {min(e, length - 1) for _, e in spans}
        | {length - 1}
    )
    refs = {}
    for index in marks:
        obs = env.set_state(states[index])["obs"]
        refs[index] = positions(obs, objects)

    # 0. The recorded states themselves, scored by the checker: what the
    # demo achieves against goal.json, independent of any replay.
    env.reset(step=0, settle=0)
    recorded: dict[str, Any] = {}
    for index in range(length):
        env.set_state(states[index])
        recorded = env.check()
    print(f"recorded: {recorded['total_completed']}/{recorded['total']} subtasks")

    # 1. Continuous replay from step 0.
    env.reset(step=0, settle=0)
    t0 = time.monotonic()
    samples, completed_at = [], []
    index = 0
    result: dict[str, Any] = {}
    while index < length:
        # Stop on every mark: drift is sampled at states[index].
        upcoming = [m for m in marks if m > index]
        stop = min(index + args.chunk, upcoming[0] if upcoming else length)
        result = env.step(actions[index:stop])
        for done in result["completed"]:
            completed_at.append({**done, "demo_step": stop})
        index = stop
        if index in refs:
            state = env.get_state()
            sample = drift(state["obs"], refs[index], objects)
            sample["max_qpos_diff"] = float(
                np.max(np.abs(state["sim_state"][1:] - states[index][1:]))
            )
            samples.append({"demo_step": index, **sample})
    replay_seconds = time.monotonic() - t0
    continuous = {
        "all_done": result["all_done"],
        "total_completed": result["total_completed"],
        "total": result["total"],
        "completed_at": completed_at,
        "final_drift": samples[-1] if samples else None,
        "max_eef_drift_m": max(s["eef_m"] for s in samples),
        "max_object_drift_m": max(s["max_object_m"] for s in samples),
        "samples": samples,
        "seconds": round(replay_seconds, 2),
        "steps_per_second": round(length / replay_seconds, 1),
    }
    print(
        f"continuous: {result['total_completed']}/{result['total']} subtasks, "
        f"all_done={result['all_done']}, max object drift "
        f"{continuous['max_object_drift_m']:.4f} m, {continuous['steps_per_second']} steps/s"
    )

    # 2. Each step on its own, from the demo's state at its start, three
    # ways: the benchmark's chain (one episode; every step k > 0 is
    # entered by resume(k) after replaying step k - 1, so the controller
    # and gripper carry over), a fresh episode per step holding the
    # current pose, and a fresh episode restoring the demo's own
    # controller and gripper commands (what's left is the simulation's).
    goal_steps = info["goal_steps"]

    def segment(k: int, start: dict[str, Any]) -> dict[str, Any]:
        begin, end = spans[k]
        end = min(end, length - 1)
        start_state = env.get_state()
        expected = [
            f"{obj}_{i}"
            for obj, steps in goal_steps.items()
            for i, step in enumerate(steps)
            if step == k
        ]
        done_here: list[str] = []
        result: dict[str, Any] = {}
        for a in range(begin, end, args.chunk):
            result = env.step(actions[a : min(a + args.chunk, end)])
            done_here += [f"{c['object']}_{c['subtask']}" for c in result["completed"]]
        at_end = drift(env.get_state()["obs"], refs[end], objects)
        return {
            "step": k,
            "description": info["steps"][k],
            "span": [begin, end],
            "resumed_subtasks": start["resumed"],
            "start_drift": drift(start_state["obs"], refs[begin], objects),
            "start_max_qpos_diff": float(
                np.max(np.abs(start_state["sim_state"][1:] - states[begin][1:]))
            ),
            "end_drift": at_end,
            "expected_subtasks": expected,
            "completed_subtasks": done_here,
            "completed_expected": set(expected) <= set(done_here),
            "total_completed": result.get("total_completed"),
        }

    resumed: dict[str, list[dict[str, Any]]] = {}
    for mode in ("benchmark_resume", "fresh", "fresh_restore_demo"):
        resumed[mode] = segments = []
        for k in range(len(spans)):
            if mode == "benchmark_resume":
                start = env.reset(step=0, settle=0) if k == 0 else env.resume(k)
            else:
                start = env.reset(step=k, settle=0, restore_demo=mode == "fresh_restore_demo")
            segments.append(segment(k, start))
            seg = segments[-1]
            print(
                f"{mode} step {k}: end drift eef {seg['end_drift']['eef_m']:.4f} m, "
                f"object {seg['end_drift']['max_object_m']:.4f} m, "
                f"completed {seg['completed_subtasks']} (expected {seg['expected_subtasks']})"
            )

    report = {
        "issue": 156,
        "case": case,
        "model": args.model,
        "task": info["task"],
        "demo_length": length,
        "spans": spans,
        "recorded": {
            "total_completed": recorded["total_completed"],
            "total": recorded["total"],
            "progress": recorded["progress"],
        },
        "continuous": continuous,
        "resumed": resumed,
        "resumed_summary": {
            mode: {
                "completed_expected": sum(s["completed_expected"] for s in segments),
                "steps": len(segments),
                "max_end_eef_drift_m": max(s["end_drift"]["eef_m"] for s in segments),
                "max_end_object_drift_m": max(
                    s["end_drift"]["max_object_m"] for s in segments
                ),
            }
            for mode, segments in resumed.items()
        },
        "wall_seconds": round(time.monotonic() - started, 1),
    }
    return report


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "cases", nargs="*",
        help="cases (Ideal/case1), task types (Ideal) or 'all'; default Ideal/case1",
    )
    parser.add_argument("--env-sh", default=DEFAULT_ENV_SH)
    parser.add_argument("--bench", default=None, help="default: $RC_BENCH from env.sh")
    parser.add_argument("--every", type=int, default=50, help="drift sample period (steps)")
    parser.add_argument("--chunk", type=int, default=10, help="actions per step call")
    parser.add_argument(
        "--model", choices=("demo", "bddl"), default="demo",
        help="the demo's recorded scene (default) or the benchmark's BDDL scene",
    )
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)

    command = server_command(os.path.expanduser(args.env_sh), args.bench, no_render=True)
    print("server:", shlex.join(command))
    reports = {}
    with RoboCerebraClient.spawn(command) as env:
        cases = args.cases or ["Ideal/case1"]
        if cases == ["all"]:
            cases = env.cases()
        elif all("/" not in c for c in cases):  # task types: all their cases
            cases = [c for c in env.cases() if c.split("/")[0] in cases]
        for case in cases:
            reports[case] = replay_case(env, case, args)
        server = env.server_info
    summary = [
        {
            "case": case,
            "recorded": f"{r['recorded']['total_completed']}/{r['recorded']['total']}",
            "continuous": f"{r['continuous']['total_completed']}/{r['continuous']['total']}",
            "continuous_max_eef_drift_m": round(r["continuous"]["max_eef_drift_m"], 4),
            "continuous_max_object_drift_m": round(r["continuous"]["max_object_drift_m"], 4),
            **{
                mode: f"{v['completed_expected']}/{v['steps']}"
                for mode, v in r["resumed_summary"].items()
            },
            "benchmark_resume_max_end_eef_drift_m": round(
                r["resumed_summary"]["benchmark_resume"]["max_end_eef_drift_m"], 4
            ),
        }
        for case, r in reports.items()
    ]
    print(json.dumps(summary, indent=1))
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        out = {"issue": 156, "model": args.model, "server": server, "summary": summary, "cases": reports}
        args.out.write_text(json.dumps(out, indent=2) + "\n")
        print("wrote", args.out)


if __name__ == "__main__":
    main()
