#!/usr/bin/env python3
"""Screw-assembly mission: two UR10 arms fasten N plates to a jig, two screws each.

A long-horizon, multi-arm TAMP example built from generic primitives (see
README.md and build_scene.py). The mission is a chain of short *blocks*,
each planned with ``run_block_with_recovery()``:

  Bootstrap   ur10_right picks the driver (a cordless drill) off its dock.
  Per part i  A0  ur10_left grasps part i from the staging row.
              A   the jig clamp takes part i (ur10_left moves it there),
                  then the driver tip screws hole 1 and hole 2 -- grasp +
                  release per hole -- while ur10_left still holds the part.
              B   ur10_left releases part i; it stays clamped.
  Return      the dock takes the driver back; ur10_right lets go.

Block A is where long-horizon planning gets hard: the clamp phase commits
ur10_left's pose around the part, and that pose decides whether the driver
can still reach *both* holes. So A is planned with a lookahead that checks
the clamp candidate against hole 1 and hole 2 before committing, and with
the recovery ladder behind it (resume, then replan the block).

Runs inside the hpp-agimus-arm64 container (pyhpp). A Viser view opens before
planning and plays completed HPP paths; pass ``--no-viewer`` for an unattended run:

    python3 build_scene.py --parts 4
    python3 task_screw_assembly.py --seed 1 --summary out.json
"""

from __future__ import annotations

import argparse
import ctypes
import json
import random
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

HERE = Path(__file__).parent
CONFIG = HERE / "config" / "screw_assembly_config.yaml"

from long_tamp.config.yaml_loader import YamlTaskLoader  # noqa: E402
from long_tamp.tasks import ManipulationTask  # noqa: E402
from long_tamp.tasks.block_recovery import (  # noqa: E402
    make_lookahead_hints_factory,
    run_block_with_recovery,
)
from long_tamp.tasks.grasp_sequence import GraspSequencePlanner  # noqa: E402
from long_tamp.tasks.mission_checkpoint import MissionCheckpoint  # noqa: E402
from long_tamp.tasks.task_planning import (  # noqa: E402
    CapabilityRegistry,
    CompositeWorldState,
    GraspTrackerState,
    RecordedFacts,
    TaskPlan,
    TaskPlanningSession,
)
from long_tamp.tasks.task_planning.predicates import holds  # noqa: E402
from long_tamp.tasks.task_planning.runner import run_plan  # noqa: E402
from screw_domain import (  # noqa: E402
    LEFT,
    RECORDED_PREDICATES,
    RIGHT,
    blocks_by_label,
    build_plan_document,
    descriptors,
)

# Planning keeps the fingers frozen open (a grasp is a rigid TCP constraint);
# how far they close on each handle comes from the grasp planner, see
# finger_closures().
FREEZE_JOINT_SUBSTRINGS = ["finger_joint", "knuckle_joint"]

ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


class ScrewAssemblyTask(ManipulationTask):
    FREEZE_JOINT_SUBSTRINGS = FREEZE_JOINT_SUBSTRINGS

    def __init__(
        self, loader: YamlTaskLoader, backend: str = "pyhpp", log_dir: str = "auto"
    ):
        super().__init__(
            task_name="Screw assembly cell",
            backend=backend,
            FILE_PATHS=loader.file_paths,
            joint_bounds=loader.joint_bounds_class,
            log_dir=log_dir,
        )
        self._loader = loader
        # Unfiltered: with_grasp_goals() drops objects that only appear on
        # a handle side, which would lose the parts' seat/hole handles.
        self.task_config = loader.task_config
        self.use_factory = True

    def build_initial_config(self) -> list[float]:
        return self._loader.build_initial_config(objects=self.task_config.OBJECTS)


def seed_everything(seed: int) -> None:
    """HPP's configuration shooter draws from libc rand() and pinocchio's
    RNG, which nothing seeds: unseeded, every process replays the same
    sequence. Seed both, plus Python's."""
    import pinocchio

    ctypes.CDLL(None).srand(seed)
    pinocchio.seed(seed)
    random.seed(seed)
    # The legacy global RNG is what library code draws from, so seed it.
    np.random.seed(seed)  # noqa: NPY002


def setup(
    backend: str = "pyhpp", log_dir: str = "auto"
) -> tuple[ScrewAssemblyTask, GraspSequencePlanner]:
    task = ScrewAssemblyTask(YamlTaskLoader(CONFIG), backend=backend, log_dir=log_dir)
    task.setup(
        validation_step=task.task_config.PATH_VALIDATION_STEP,
        projector_step=task.task_config.PATH_PROJECTOR_STEP,
        freeze_joint_substrings=task.FREEZE_JOINT_SUBSTRINGS,
        skip_graph=True,
    )
    # Paths here are short arm moves; optimization rarely pays off, and at
    # the 30 s default each part release spent ~60 s in two optimizer passes.
    # No spline optimizer: its inner QP solve ignores the timeout, and a
    # mission hung 11+ minutes in a single solve. Shortcuts suffice here.
    task.planner.configure_transition_planner(
        path_optimizer_timeout=5.0, spline_optimizer=False
    )
    planner = GraspSequencePlanner(
        graph_builder=task.graph_builder,
        config_gen=task.config_gen,
        planner=task.planner,
        task_config=task.task_config,
        backend=task.backend,
        graph_constraints=getattr(task, "_graph_constraints", None),
        freeze_joint_substrings=task.FREEZE_JOINT_SUBSTRINGS,
        auto_save_dir=None,
        run_logger=getattr(task, "run_logger", None),
    )
    return task, planner


def finger_closures():
    """Per (arm gripper, handle) Robotiq closure, from the grasp planner.

    Both arms carry a Robotiq 2F-85; the driver tip and the fixtures'
    clamps are virtual grippers with no fingers, so they have no entry.
    """
    from long_tamp.grasping import ROBOTIQ_2F85, FingerClosureTable

    return FingerClosureTable.from_task_yaml(
        CONFIG, {f"{LEFT}/gripper": ROBOTIQ_2F85, f"{RIGHT}/gripper": ROBOTIQ_2F85}
    )


def sample_phases(
    task: ScrewAssemblyTask, phases: list[dict[str, Any]], label: str, dt: float = 0.05
) -> list[dict[str, Any]]:
    """Sample the completed phases' paths every ``dt`` seconds of path time
    (paths are time-parameterized), for replay.py."""
    segments = []
    for phase in phases:
        if not phase.get("complete", True) or phase.get("skipped"):
            continue
        configs: list[list[float]] = []
        for path in phase.get("paths", []):
            if path is None:
                continue
            if isinstance(path, int):
                path = task.planner.get_path(path)
            length = path.length()
            t0 = path.timeRange().first if hasattr(path, "timeRange") else 0.0
            n = max(2, int(length / dt) + 1)
            for i in range(n):
                q, ok = path.eval(t0 + length * i / (n - 1))
                if ok:
                    configs.append([round(float(v), 5) for v in q])
        if configs:
            segments.append(
                {
                    "block": label,
                    "gripper": phase.get("gripper"),
                    "handle": phase.get("handle"),
                    "configs": configs,
                }
            )
    return segments


def write_trajectory(path: Path, segments: list[dict[str, Any]], parts: int) -> None:
    """Write replay.py's input (atomically: it is rewritten after each block)."""
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps({"parts": parts, "dt": 0.05, "segments": segments}))
    tmp.replace(path)


def home_target(task: ScrewAssemblyTask, q: list[float], arm: str, carried: str | None):
    """``q`` with ``arm`` at its home pose and ``carried`` (held by that arm's
    gripper) moved along, so the target satisfies the held grasp."""
    import build_scene
    import pinocchio as pin

    home = build_scene.LEFT_HOME if arm == LEFT else build_scene.RIGHT_HOME
    rank = task.robot.rankInConfiguration
    qt = np.array(q, dtype=float)
    for joint, value in zip(ARM_JOINTS, home):
        qt[rank[f"{arm}/{joint}"]] = value
    if carried is not None:
        # Grasp: gripper frame == handle frame, so
        # world<-object = world<-gripper * (object<-handle)^-1.
        grip = pin.XYZQUATToSE3(np.array(build_scene.DRIVER_GRIP_XYZQUAT))
        model = task.planner.device.model()
        data = model.createData()
        pin.framesForwardKinematics(model, data, qt)
        o_m_obj = data.oMf[model.getFrameId(f"{arm}/gripper")] * grip.inverse()
        r = rank[f"{carried}/root_joint"]
        qt[r : r + 7] = pin.SE3ToXYZQUAT(o_m_obj)
    return qt.tolist()


def run_home_move(
    task: ScrewAssemblyTask,
    planner: GraspSequencePlanner,
    q: list[float],
    arm: str,
    carried: str | None,
    attempts: int = 5,
    verbose: bool = True,
) -> dict[str, Any]:
    """Move ``arm`` home within the current grasp state (other arm frozen)."""
    other = RIGHT if arm == LEFT else LEFT
    target = home_target(task, q, arm, carried)
    ok, report = task.planner.problem.isConfigValid(np.asarray(target, dtype=float))
    if ok:  # isConfigValid may run without JointBoundValidation registered
        model = task.planner.device.model()
        t = np.asarray(target, dtype=float)
        lo, hi = model.lowerPositionLimit, model.upperPositionLimit
        out = np.where((t < lo - 1e-9) | (t > hi + 1e-9))[0]
        if len(out):
            ok, report = False, f"out of bounds at config index {out.tolist()}"
    if not ok:
        return {
            "success": False,
            "final_config": q,
            "resumes": 0,
            "replans": 0,
            "message": f"home target invalid: {report}",
        }
    last = ""
    for attempt in range(1, attempts + 1):
        r = planner.plan_loop(
            f"{arm}/gripper",
            q,
            target,
            frozen_arms_mode="manual",
            per_phase_frozen_arms={0: [other]},
            q_scene_init=task.q_init,
            verbose=verbose,
        )
        if r.get("success"):
            return {
                "success": True,
                "final_config": r["final_config"],
                "resumes": attempt - 1,
                "replans": 0,
                "message": "moved",
                "phase_results": r.get("phase_results", []),
            }
        last = r.get("message", "")
    return {
        "success": False,
        "final_config": q,
        "resumes": attempts,
        "replans": 0,
        "message": f"home move failed {attempts}x: {last}",
    }


def run_block(
    task: ScrewAssemblyTask,
    planner: GraspSequencePlanner,
    ctx: dict[str, Any],
    index: int,
    block: dict[str, Any],
    max_replans: int = 10,
    verbose: bool = True,
) -> dict[str, Any]:
    """Plan one block from ``ctx["q"]``, then log and record it.

    ``ctx`` carries the mission's running state: ``q`` (updated only on
    success), ``records`` (one per block), ``trajectory``, ``checkpoint``,
    ``live_viewer`` and ``n_parts``.
    """
    q = ctx["q"]
    checkpoint, trajectory = ctx["checkpoint"], ctx["trajectory"]
    live_viewer = ctx["live_viewer"]
    if checkpoint is not None:
        checkpoint.phase_dump_dir(index, block["label"])
    t0 = time.time()
    print(f"\n=== {block['label']} ===", flush=True)
    if "move" in block:
        r = run_home_move(task, planner, q, *block["move"], verbose=verbose)
        phase_results = r.get("phase_results", [])
    else:
        hints_factory = None
        if "lookahead" in block:
            hints_factory = make_lookahead_hints_factory(
                planner,
                block["seq"],
                q,
                q_scene_init=task.q_init,
                per_phase_frozen_arms=block["frozen"],
                phase_pair=block["lookahead"]["pair"],
                also_protect=block["lookahead"]["also"],
                # Path-check the clamp move too: a clamp target the arm
                # can't reach by path gets redrawn in the real plan, which
                # voids the hints and costs a block replan.
                verify_paths=True,
                verbose=verbose,
            )
        r = run_block_with_recovery(
            planner,
            block["seq"],
            q,
            q_scene_init=task.q_init,
            per_phase_frozen_arms=block["frozen"],
            label=block["label"],
            hints_factory=hints_factory,
            max_replans=max_replans,
            verbose=verbose,
        )
        phase_results = planner.phase_results
    record = {
        "label": block["label"],
        "success": r["success"],
        "replans": r["replans"],
        "resumes": r["resumes"],
        "seconds": round(time.time() - t0, 2),
        "message": r["message"],
    }
    ctx["records"].append(record)
    if "move" in block:
        detail = f"{r['resumes']} retries) {r['message']}"
    else:
        detail = f"{r['resumes']} resumes, {r['replans']} replans)"
    print(
        f"--- {block['label']}: {'ok' if r['success'] else 'FAILED'} "
        f"({record['seconds']}s, {detail}",
        flush=True,
    )
    if checkpoint is not None:
        checkpoint.record(
            index,
            block["label"],
            r,
            record["seconds"],
            r["final_config"],
            planner.grasp_tracker.current_grasps,
        )
    if r["success"]:
        if trajectory is not None:
            trajectory += sample_phases(task, phase_results, block["label"])
            # After every block, like the run log, so a killed run keeps its
            # motion and a resume can append to it.
            if checkpoint is not None:
                write_trajectory(
                    checkpoint.dir / "trajectory.json", trajectory, ctx["n_parts"]
                )
        if live_viewer is not None:
            live_viewer.completed(phase_results)
        ctx["q"] = r["final_config"]
    return r


def mission_session(
    task: ScrewAssemblyTask,
    planner: GraspSequencePlanner,
    ctx: dict[str, Any],
    recorded: RecordedFacts,
    max_replans: int = 10,
    verbose: bool = True,
) -> TaskPlanningSession:
    """The mission's TaskPlan, bound to this scene (one capability per block kind).

    Every capability runs its block through ``run_block``; a failed block
    raises, leaving ``ctx["q"]`` unchanged (the transaction contract). The
    world state is the grasp tracker plus the recorded facts, so a step whose
    effect already holds (after a resume, or from a scenario start) is
    skipped.
    """
    n_parts = ctx["n_parts"]
    blocks = blocks_by_label(n_parts)
    index = {label: i for i, label in enumerate(blocks)}

    def run(parameters: dict[str, Any]) -> dict[str, Any]:
        label = parameters["block"]
        r = run_block(
            task,
            planner,
            ctx,
            index[label],
            blocks[label],
            max_replans=max_replans,
            verbose=verbose,
        )
        if not r["success"]:
            raise RuntimeError(f"{label}: {r['message']}")
        return {"replans": r["replans"], "resumes": r["resumes"]}

    world = CompositeWorldState(GraspTrackerState(planner), recorded)

    def check(descriptor):
        """A guard condition: true when its literals hold in the world."""

        def evaluate(parameters: dict[str, Any]) -> bool:
            state = world()
            return all(
                holds(literal.ground(parameters), state)
                for literal in descriptor.precondition_literals
            )

        return evaluate

    registry = CapabilityRegistry()
    for name, descriptor in descriptors(n_parts).items():
        is_guard = name in ("part_done", "all_parts_done")
        registry.register(descriptor, check(descriptor) if is_guard else run)
    plan = TaskPlan.from_dict(build_plan_document(n_parts), registry)
    return TaskPlanningSession(plan, registry, world_state=world, recorded=recorded)


def run_mission(
    task: ScrewAssemblyTask,
    planner: GraspSequencePlanner,
    n_parts: int,
    max_replans: int = 10,
    verbose: bool = True,
    trajectory: list[dict[str, Any]] | None = None,
    checkpoint: MissionCheckpoint | None = None,
    q_start: list[float] | None = None,
    live_viewer=None,
    recorded: RecordedFacts | None = None,
) -> dict[str, Any]:
    """Run the mission's TaskPlan from ``q_start`` (default: the scene start).

    Steps run in plan order through a TaskPlanningSession; a step whose
    effect already holds in the world (grasp tracker + ``recorded`` facts) is
    skipped, which is how ``--resume`` continues a run. If ``trajectory`` is a
    list, each completed block's motion is sampled into it; if ``checkpoint``
    is given, every block is logged to it.
    """
    if recorded is None:
        recorded = RecordedFacts(None, predicates=RECORDED_PREDICATES)
    ctx: dict[str, Any] = {
        "q": list(q_start if q_start is not None else task.q_init),
        "records": [],
        "trajectory": trajectory,
        "checkpoint": checkpoint,
        "live_viewer": live_viewer,
        "n_parts": n_parts,
    }
    session = mission_session(task, planner, ctx, recorded, max_replans, verbose)
    t_mission = time.time()
    run = run_plan(
        session,
        on_skip=lambda label, why: print(f"\n=== {label} === skipped ({why})"),
    )
    if not run.success:
        print(f"\n=== {run.failed_step} stopped the mission: {run.message}")
    success, skipped = run.success, run.skipped
    return {
        "success": success,
        "seconds": round(time.time() - t_mission, 2),
        "blocks": ctx["records"],
        "skipped": skipped,
        "final_config": ctx["q"],
    }


from long_tamp.visualization.mission_viewer import MissionViewer


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--summary", type=Path, help="write a JSON run summary here")
    ap.add_argument("--max-replans", type=int, default=10)
    ap.add_argument(
        "--no-viewer",
        action="store_true",
        help="run without the live Viser viewer or path playback",
    )
    ap.add_argument(
        "--viewer-port",
        type=int,
        default=8081,
        help="port for the Viser result view (default: 8081)",
    )
    ap.add_argument(
        "--run-dir",
        type=Path,
        help="the run's folder: mission.json (run log), checkpoint.json (resume "
        "point), run.log, phases/, trajectory.json. Default: "
        "runs/seed<S>_<timestamp>/",
    )
    ap.add_argument(
        "--resume",
        action="store_true",
        help="continue the run in --run-dir: restore its configuration, grasps and "
        "recorded facts, then run the plan; steps whose effects hold are skipped",
    )
    ap.add_argument(
        "--check",
        action="store_true",
        help="load the scene, validate the start configuration, then exit",
    )
    args = ap.parse_args()
    if args.resume and args.run_dir is None:
        ap.error("--resume needs --run-dir")
    run_dir = args.run_dir or HERE / "runs" / (
        f"seed{args.seed}_{time.strftime('%Y%m%d_%H%M%S')}"
    )

    seed_everything(args.seed)
    task, planner = setup(log_dir=str(run_dir))
    n_parts = sum(1 for o in task.task_config.OBJECTS if o.startswith("part"))
    ok, err = task.planner.problem.isConfigValid(np.asarray(task.q_init, dtype=float))
    print(
        f"\nscene: {n_parts} part(s), {len(task.q_init)} config DOF; "
        f"start configuration valid: {ok}{'' if ok else f' ({err})'}"
    )
    if args.check or not ok:
        return 0 if ok else 1

    checkpoint = MissionCheckpoint(
        run_dir, meta={"seed": args.seed, "parts": n_parts, "commit": _commit()}
    )
    traj_path = run_dir / "trajectory.json"
    trajectory: list[dict[str, Any]] = []
    q_start = None
    # Recorded facts (screws driven) live in the run folder, next to the
    # checkpoint, so a resumed run knows what earlier attempts achieved.
    recorded = RecordedFacts(run_dir / "facts.json", predicates=RECORDED_PREDICATES)
    if args.resume:
        point = checkpoint.load()
        if point is not None:
            MissionCheckpoint.restore_grasps(planner.grasp_tracker, point["held"])
            q_start = point["q"]
            if traj_path.exists():
                trajectory = json.loads(traj_path.read_text())["segments"]
            print(
                f"resuming from the world state after {point['last_label']!r}: "
                "steps whose effects hold will be skipped"
            )
    print(f"run folder: {run_dir}")

    import atexit

    closures = finger_closures()
    for row in closures.report():
        if row["handle"].startswith(("driver", "part1")):
            print(
                f"grasp {row['gripper']} > {row['handle']}: close to "
                f"{row['width'] * 1000:.1f} mm (finger_joint {row['q']:.3f})"
                + ("" if row["feasible"] else f"  WARNING: {'; '.join(row['reasons'])}")
            )
    live_viewer = None
    if not args.no_viewer:
        live_viewer = MissionViewer(
            task,
            args.viewer_port,
            q_start or task.q_init,
            camera=((2.15, -2.30, 1.55), (0.75, 0.0, 0.58)),
            closures=closures,
        )
        atexit.register(live_viewer.close)
    try:
        result = run_mission(
            task,
            planner,
            n_parts,
            max_replans=args.max_replans,
            trajectory=trajectory,
            checkpoint=checkpoint,
            q_start=q_start,
            live_viewer=live_viewer,
            recorded=recorded,
        )
    except BaseException:
        if live_viewer is not None:
            live_viewer.close()
        raise
    checkpoint.finish(result["success"], result["seconds"])
    write_trajectory(traj_path, trajectory, n_parts)
    result["seed"] = args.seed
    result["parts"] = n_parts
    print(
        f"\n{'MISSION COMPLETE' if result['success'] else 'MISSION FAILED'} "
        f"in {result['seconds']}s  (run log: {run_dir / 'mission.json'})"
    )
    if args.summary:
        summary = {k: v for k, v in result.items() if k != "final_config"}
        args.summary.write_text(json.dumps(summary, indent=2))
    if live_viewer is not None:
        try:
            live_viewer.finish()
        finally:
            live_viewer.close()
    return 0 if result["success"] else 1


def _commit() -> str:
    import subprocess

    try:
        return subprocess.run(
            ["git", "-C", str(HERE), "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    except Exception:
        return "unknown"


if __name__ == "__main__":
    sys.exit(main())
