#!/usr/bin/env python3
"""Screw-assembly mission: two UR10 arms fasten N plates to a jig, two screws each.

A long-horizon, multi-arm TAMP example built from generic primitives (see
README.md and build_scene.py). The mission is a chain of short *blocks*,
each refined by a ``GraspSequenceRefiner`` (``run_block_with_recovery()``
plus a lookahead, see ``long_tamp.tasks.refiner``):

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
import math
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
from long_tamp.tasks.grasp_sequence import GraspSequencePlanner  # noqa: E402
from long_tamp.tasks.mission_checkpoint import MissionCheckpoint  # noqa: E402
from long_tamp.tasks.refiner import GraspSequenceRefiner  # noqa: E402
from long_tamp.tasks.task_planning import (  # noqa: E402
    CapabilityRegistry,
    CompositeWorldState,
    GraspTrackerState,
    RecordedFacts,
    TaskPlan,
    TaskPlanningSession,
    parallelize,
)
from long_tamp.tasks.task_planning.events import (  # noqa: E402
    JsonlEventWriter,
    make_event,
    plan_event,
)
from long_tamp.tasks.task_planning.skills import SkillCommand  # noqa: E402
from long_tamp.sim.skills import SCREW  # noqa: E402
from long_tamp.tasks.task_planning.predicates import holds  # noqa: E402
from long_tamp.execution import (  # noqa: E402
    ExecutionCommand,
    ExecutionPolicy,
    MockBackend,
    PathPlaybackBackend,
    PlanExecutor,
    ProcessBackend,
)
from screw_domain import (  # noqa: E402
    LEFT,
    RECORDED_PREDICATES,
    RIGHT,
    blocks_by_label,
    build_plan_document,
    block_for,
    clamp_seats,
    descriptors,
    goal_vocabulary,
    pddl_problem,
    planned_document,
    refinement_step,
    repair_policy,
)

# Planning keeps the fingers frozen open (a grasp is a rigid TCP constraint);
# how far they close on each handle comes from the grasp planner, see
# finger_closures().
FREEZE_JOINT_SUBSTRINGS = ["knuckle_joint", "finger_tip_joint"]

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


#: SplineGradientBased's QP iteration cap (proxsuite's default is 10000 and
#: doesn't bound a hard problem).
QP_MAX_ITERATIONS = 1000


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
    # The spline optimizer's inner QP solve ignores that timeout (a mission
    # hung 11+ minutes in one solve): its iterations are capped, which needs
    # an hpp-core with SplineGradientBased/QPMaxIterations. Without one, the
    # backend drops the spline optimizer, as before (#26).
    task.planner.configure_transition_planner(
        path_optimizer_timeout=5.0, qp_max_iterations=QP_MAX_ITERATIONS
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
        refiner = ctx.get("refiner") or GraspSequenceRefiner(
            planner, q_scene_init=task.q_init, max_replans=max_replans, verbose=verbose
        )
        refined = refiner.refine(refinement_step(block), q)
        r = refined.as_dict()
        phase_results = refined.phases
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
            # With a playback backend the backend drives the viewer; record only.
            live_viewer.completed(phase_results, play=not ctx.get("backend_displays"))
        ctx["q"] = r["final_config"]
        ctx["last_phases"] = phase_results
    return r


def block_commands(
    task: ScrewAssemblyTask, label: str, phases: list[dict[str, Any]]
) -> list[ExecutionCommand]:
    """The motion a block produced, one command per completed path.

    Driving a screw is a skill: the planned insertion of the driver's tip into
    a hole (the grasp edge ``_12``, from pregrasp to grasp) is sent as a
    ``SkillCommand`` for the screw skill. A backend that runs skills (MuJoCo)
    drives the screw; any other plays the path.
    """
    commands = []
    for phase in phases:
        if not phase.get("complete", True) or phase.get("skipped"):
            continue
        paths = [p for p in phase.get("paths", []) if p is not None]
        edges = list(phase.get("edges", []))
        # A resumed phase planned only its last edges: align from the end.
        edges = (
            [None] * (len(paths) - len(edges)) + edges[-len(paths) :] if paths else []
        )
        for path, edge in zip(paths, edges):
            if isinstance(path, int):
                path = task.planner.get_path(path)
            payload = path
            hole = phase.get("handle") or ""
            if (
                phase.get("gripper") == "driver/tip"
                and edge is not None
                and str(edge).endswith("_12")
            ):
                payload = SkillCommand(
                    SCREW,
                    {"tool": "driver", "part": hole.split("/")[0], "hole": hole},
                    approach=path,
                )
            commands.append(
                ExecutionCommand(step_id=label, duration=path.length(), payload=payload)
            )
    return commands


def mission_session(
    task: ScrewAssemblyTask,
    planner: GraspSequencePlanner,
    ctx: dict[str, Any],
    recorded: RecordedFacts,
    max_replans: int = 10,
    verbose: bool = True,
    document: dict[str, Any] | None = None,
) -> TaskPlanningSession:
    """The mission's TaskPlan, bound to this scene (one capability per block kind).

    Every capability runs its block through ``run_block``; a failed block
    raises, leaving ``ctx["q"]`` unchanged (the transaction contract). The
    world state is the grasp tracker plus the recorded facts, so a step whose
    effect already holds (after a resume, or from a scenario start) is
    skipped.
    """
    n_parts = ctx["n_parts"]

    def runner(capability: str):
        def run(parameters: dict[str, Any]) -> dict[str, Any]:
            return run_step(capability, parameters)

        return run

    def run_step(capability: str, parameters: dict[str, Any]) -> dict[str, Any]:
        # The block comes from the step itself, so a planner-built plan (e.g.
        # part 1 in clamp 2) runs the same way as the hand-written one.
        block = block_for(capability, parameters)
        label = block["label"]
        # Where the step starts (grasps held), to replan it from if the robot
        # has drifted by the time its motion runs.
        ctx.setdefault("step_start", {})[label] = {
            g: h for g, h in planner.grasp_tracker.current_grasps.items() if h
        }
        index = ctx.setdefault("block_index", 0)
        ctx["block_index"] = index + 1
        binding = {k: v for k, v in parameters.items() if k != "block"}
        if _injected(ctx, capability, binding):
            # --inject-failure: fail as if its first phase couldn't be
            # reached, without planning (the M3 exit test).
            gripper, handle = block["seq"][0]
            ctx["failure"] = {
                "step": label,
                "capability": capability,
                "parameters": binding,
                "facts": [
                    f"refinement_failed({label.replace(' ', '_')})",
                    f"cannot_reach({gripper}, {handle})",
                ],
            }
            print(
                f"\n=== {label} === INJECTED FAILURE: cannot_reach({gripper}, {handle})"
            )
            raise RuntimeError(f"{label}: injected failure")
        r = run_block(
            task,
            planner,
            ctx,
            index,
            block,
            max_replans=max_replans,
            verbose=verbose,
        )
        if not r["success"]:
            # The transaction contract: a failed block leaves the world as it
            # was. Undo whatever its phases committed to the grasp tracker.
            reset = getattr(planner, "reset_grasp_tracker_to_call_start", None)
            if reset is not None and "seq" in block:
                reset()
            ctx["failure"] = {
                "step": label,
                "capability": capability,
                "parameters": binding,
                "facts": list(r.get("facts", [])),
            }
            raise RuntimeError(f"{label}: {r['message']}")
        executor = ctx.get("executor")
        if executor is not None:
            for command in block_commands(task, label, ctx.get("last_phases", [])):
                executor.submit(command)
        return {"replans": r["replans"], "resumes": r["resumes"]}

    def on_drift(node: dict[str, Any], observe) -> list[ExecutionCommand]:
        """The robot drifted before this step's motion: plan the step again
        from where it is (the grasps it started with, the observed config)."""
        operation = node["children"][0] if node["type"] == "transaction" else node
        capability = operation["capability"]
        parameters = dict(operation.get("parameters", {}))
        label = block_for(capability, parameters)["label"]
        MissionCheckpoint.restore_grasps(
            planner.grasp_tracker, ctx["step_start"][label]
        )
        observed = observe(ctx["q"]) if observe is not None else None
        if observed is None:
            raise RuntimeError("the backend can't report the observed configuration")
        # Within the planner's bounds: a simulated finger at -1e-9 rad is
        # "out of range" to HPP, which then rejects the start configuration.
        model = task.robot.model()
        observed = np.clip(observed, model.lowerPositionLimit, model.upperPositionLimit)
        ctx["q"] = [float(v) for v in observed]
        print(f"\n=== {label} === the robot drifted: replanning from where it is")
        executor = ctx["executor"]
        submitted = len(executor._pending)
        run_step(capability, parameters)
        commands = executor._pending[submitted:]
        del executor._pending[submitted:]
        return commands

    ctx["on_drift"] = on_drift

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
        registry.register(descriptor, check(descriptor) if is_guard else runner(name))
    document = document or build_plan_document(n_parts)
    if ctx.get("concurrent"):
        # Independent steps (the other arm's) in parallel lanes (#21).
        document = parallelize(document, registry)
    plan = TaskPlan.from_dict(document, registry)
    return TaskPlanningSession(plan, registry, world_state=world, recorded=recorded)


def _injected(ctx: dict[str, Any], capability: str, binding: dict[str, Any]) -> bool:
    """Whether ``--inject-failure`` targets this step (each spec fires once)."""
    for spec in ctx.get("inject", []):
        if spec["fired"] or spec["capability"] != capability:
            continue
        if all(str(binding.get(k)) == v for k, v in spec["match"].items()):
            spec["fired"] = True
            return True
    return False


def parse_injection(text: str) -> dict[str, Any]:
    """``capability:key=value,key=value`` -> an injection spec."""
    capability, _, rest = text.partition(":")
    match = dict(item.split("=", 1) for item in rest.split(",") if item)
    return {"capability": capability, "match": match, "fired": False}


def run_with_repair(
    task: ScrewAssemblyTask,
    planner: GraspSequencePlanner,
    n_parts: int,
    q_start: list[float] | None,
    recorded: RecordedFacts,
    rounds: int,
    mission: dict[str, Any],
    goal: list[str] | None = None,
    constraints: list[tuple[str, dict[str, str]]] | None = None,
) -> dict[str, Any]:
    """Plan, run, and replan around failures (``--replan``, issue #15).

    Every round plans the goal from the current world state with the blocked
    bindings so far, and runs it from where the last round stopped.
    """
    from long_tamp.tasks.task_planning.repair import plan_execute_repair

    state = {"q": q_start, "result": None, "seconds": 0.0, "blocks": [], "skipped": []}

    clamps = clamp_seats(task.task_config.VALID_PAIRS)

    def plan(blocked):
        return plan_from_goal(
            n_parts,
            world_atoms(planner, recorded),
            blocked=[*(constraints or []), *blocked],
            clamps=clamps,
            goal=goal,
        )

    def execute(document):
        result = run_mission(
            task, planner, n_parts, q_start=state["q"], document=document, **mission
        )
        state["q"] = result["final_config"]
        state["seconds"] += result["seconds"]
        state["blocks"] += result["blocks"]
        state["skipped"] += result["skipped"]
        state["result"] = result
        return result["failure"]

    def on_replan(failure, blocked):
        print(
            f"\n=== replanning: {failure['step']} failed "
            f"({', '.join(failure.get('facts', [])[1:]) or 'no facts'}); "
            f"blocking {blocked}",
            flush=True,
        )

    outcome = plan_execute_repair(
        plan, execute, repair_policy, max_rounds=rounds, on_replan=on_replan
    )
    # No first plan at all: nothing ran.
    result = dict(state["result"] or {"failure": None, "final_config": state["q"]})
    result.update(
        success=outcome.success,
        repair_message=outcome.message,
        seconds=round(state["seconds"], 2),
        blocks=state["blocks"],
        skipped=state["skipped"],
        task_replans=len(outcome.rounds) - 1,
        blocked=outcome.blocked,
    )
    if not outcome.success:
        print(f"\n=== repair gave up: {outcome.message}")
    return result


def run_supervised(
    task: ScrewAssemblyTask,
    planner: GraspSequencePlanner,
    n_parts: int,
    q_start: list[float] | None,
    recorded: RecordedFacts,
    rounds: int,
    mission: dict[str, Any],
    instruction: str,
    goal: list[str],
    constraints: list[tuple[str, dict[str, str]]],
    client,
    run_dir: Path,
) -> dict[str, Any]:
    """Repair loops under the execution supervisor (``--supervise``, #88).

    Each repair loop (``run_with_repair``) handles what the refiner can
    explain. When one gives up, the supervisor role decides: retry, relax
    the goal (keep a subset of it), abort, or escalate. Escalation and abort
    write ``escalation.md`` / ``escalation.json`` to the run folder.
    """
    from long_tamp.tasks.task_planning.skeleton import default_planner
    from long_tamp.tasks.task_planning.supervisor import Limits, Situation, supervise

    clamps = clamp_seats(task.task_config.VALID_PAIRS)
    where = {"q": q_start}
    #: What every repair loop ran, for the mission's summary.
    total = {"seconds": 0.0, "blocks": [], "skipped": [], "loops": 0}
    task_planner = default_planner("auto")

    def run_repair(current_goal):
        result = run_with_repair(
            task,
            planner,
            n_parts,
            where["q"],
            recorded,
            rounds,
            mission,
            current_goal,
            constraints,
        )
        where["q"] = result.get("final_config") or where["q"]
        total["seconds"] += result.get("seconds", 0.0)
        total["blocks"] += result.get("blocks", [])
        total["skipped"] += result.get("skipped", [])
        total["loops"] += 1
        return result

    def situation_of(result, current_goal):
        state = world_atoms(planner, recorded)
        return Situation(
            instruction=instruction,
            original_goal=goal,
            goal=current_goal,
            failure=result.get("failure"),
            message=result.get("repair_message", ""),
            blocked=result.get("blocked", []),
            vocabulary=goal_vocabulary(n_parts, state, clamps),
        )

    def reachable(candidate):
        try:
            task_planner.solve(
                pddl_problem(
                    n_parts,
                    world_atoms(planner, recorded),
                    constraints,
                    clamps,
                    candidate,
                )
            )
        except Exception as error:  # noqa: BLE001 - the planner's verdict
            return str(error).splitlines()[0][:300] or type(error).__name__
        return None

    outcome = supervise(
        run_repair,
        situation_of,
        instruction,
        goal,
        client,
        Limits(max_decisions=3, max_tokens=200_000),
        reachable,
        report_dir=run_dir,
    )
    for i, d in enumerate(outcome.decisions, 1):
        print(
            f"\n=== supervisor decision {i}: {d['action']}"
            + (f" (fallback: {d['fallback']})" if d["fallback"] else "")
            + f"\n    {d['reason']}"
            + (f"\n    goal now: {d['goal']}" if d["action"] == "relax_goal" else ""),
            flush=True,
        )
    if outcome.report is not None:
        print(f"\n=== escalation report: {outcome.report}", flush=True)
    result = dict(outcome.last)
    result.update(
        success=outcome.success,
        seconds=round(total["seconds"], 2),
        blocks=total["blocks"],
        skipped=total["skipped"],
    )
    result["supervisor"] = {
        "original_goal": goal,
        "final_goal": outcome.goal,
        "decisions": outcome.decisions,
        "stopped": outcome.stopped,
        "repair_loops": total["loops"],
        "report": str(outcome.report) if outcome.report else None,
    }
    return result


def run_chat(
    task, planner, n_parts, q_start, recorded, mission, client, events, web=None
):
    """An operator's chat with a model acting through gated mission tools
    (``--chat``, #89): reads operator messages from stdin until EOF or
    "quit", and writes every tool call to the event stream. With ``web`` (the
    ``--web-port`` viewer), the same chat is also in the viewer's chat panel
    (#90); after stdin closes, it goes on there until "quit" or Ctrl-C."""
    from chat_tools import INTRO, MissionChat

    from long_tamp.ai.chat import ChatSession
    from long_tamp.viewer import ChatBridge

    work = MissionChat(
        sys.modules[__name__],
        task,
        planner,
        recorded,
        n_parts,
        mission,
        q_start,
        client=client,
    )

    def on_tool(call):
        events(
            make_event(
                "chat",
                "tool",
                call.tool,
                "SUCCESS" if call.ok else "FAILURE",
                message=call.error,
                metrics={"arguments": call.arguments},
            )
        )
        if call.tool == "plan" and call.ok and work.document is not None:
            events(plan_event(work.document, "planned in the chat"))

    session = ChatSession(
        client, work.tools(), INTRO + "\n\n" + work.domain(), on_tool=on_tool
    )
    bridge = ChatBridge(session)
    if web is not None:
        bridge.attach(web)
        print(f"chat: also in the web viewer, {web.url}", flush=True)
    echo = not sys.stdin.isatty()  # piped messages: show them in the log
    print("chat: type an instruction, 'quit' or Ctrl-D to end", flush=True)
    while True:
        try:
            line = input("operator> ")
        except EOFError:
            if web is not None and not bridge.ended.is_set():
                print(
                    "chat: stdin closed; the chat goes on in the web viewer "
                    "('quit' there or Ctrl-C ends it)",
                    flush=True,
                )
                try:
                    bridge.ended.wait()
                except KeyboardInterrupt:
                    pass
                bridge.wait()
            break
        if echo:
            print(line, flush=True)
        if line.strip().lower() in ("quit", "exit"):
            break
        if not line.strip():
            continue
        turn = bridge.turn(line.strip())
        if turn is None:  # the error is in the transcript
            print(f"  [error] {bridge.transcript()[-1]['text']}", flush=True)
            continue
        for call in turn.calls:
            print(f"  [tool] {call.as_text()[:400]}", flush=True)
        if turn.error:
            print(f"  [error] {turn.error}", flush=True)
        print(f"model> {turn.say}", flush=True)
    last = work.last or {"success": True, "seconds": 0.0, "failure": None}
    result = dict(last)
    result.setdefault("final_config", work.q)
    result["chat"] = {
        "turns": [
            {"user": t.user, "say": t.say, "calls": [c.as_text() for c in t.calls]}
            for t in session.turns
        ],
        "goal": work.goal,
        "constraints": work.constraints,
    }
    return result


def world_atoms(planner: GraspSequencePlanner, recorded: RecordedFacts) -> list[str]:
    """The world state as ground atoms: held grasps plus recorded facts."""
    state = CompositeWorldState(GraspTrackerState(planner), recorded)()
    return sorted(str(atom) for atom in state)


def understand_instruction(
    instruction: str,
    n_parts: int,
    state: list[str],
    clamps: list[tuple[str, str]] | None = None,
    model: str | None = None,
    on_call=None,
    client=None,
) -> tuple[list[str], list[tuple[str, dict[str, str]]]]:
    """The goal and the constraints in an operator's ``instruction``
    (``--instruction``; #22, #87), through three model roles on one client
    (``model``: ``"<api>:<model>"``, default ``anthropic:claude-opus-5-5``):

    1. the grounder finds the objects the instruction refers to (fallback:
       matching their names);
    2. the goal writer writes the goal, checked against the vocabulary and
       reachable by the task planner from ``state``;
    3. the plan reviewer turns what the instruction rules out ("don't use
       clamp 1") into blocked bindings, kept only if the planner still
       reaches the goal (fallback: none).

    No role writes steps. ``on_call`` receives each model call's record.
    """
    from long_tamp.ai import make_client
    from long_tamp.tasks.task_planning.language import (
        ModelGoalWriter,
        goal_from_instruction,
        ground_instruction,
        with_grounding,
    )
    from long_tamp.tasks.task_planning.review import ReviewRequest, review_constraints
    from long_tamp.tasks.task_planning.skeleton import default_planner

    task_planner = default_planner("auto")

    def solve(goal, blocked=None):
        return task_planner.solve(pddl_problem(n_parts, state, blocked, clamps, goal))

    def why_not(goal, blocked=None) -> str | None:
        try:
            solve(goal, blocked)
        except Exception as error:  # noqa: BLE001 - the planner's verdict
            return str(error).splitlines()[0][:300] or type(error).__name__
        return None

    client = client or make_client(model, on_call=on_call)
    vocabulary = goal_vocabulary(n_parts, state, clamps)
    t0 = time.time()
    grounding = ground_instruction(instruction, vocabulary, client)
    vocabulary = with_grounding(vocabulary, grounding.value)
    goal = goal_from_instruction(
        instruction, ModelGoalWriter(client), vocabulary, lambda g: why_not(g)
    )
    plan = tuple(
        f"{cap}({', '.join(f'{k}={v}' for k, v in params.items() if k != 'block')})"
        for cap, params in solve(goal)
    )
    review = review_constraints(
        ReviewRequest(instruction, vocabulary, descriptors(n_parts), plan),
        client,
        replannable=lambda blocked: why_not(goal, blocked),
    )
    print(
        f"instruction {instruction!r} ({client.spec}, {time.time() - t0:.1f}s):\n"
        f"  refers to: {', '.join(grounding.value.objects) or '-'}"
        + (f" (fallback: {grounding.fallback})" if grounding.fallback else "")
        + "\n  goal:\n    "
        + "\n    ".join(goal)
        + "\n  constraints: "
        + (
            "; ".join(f"no {c}{b}" for c, b in review.value)
            if review.value
            else "none" + (f" (fallback: {review.fallback})" if review.fallback else "")
        ),
        flush=True,
    )
    for name, outcome in (("grounder", grounding), ("plan reviewer", review)):
        if outcome.fallback and outcome.attempts:
            print(f"  {name} attempts rejected:", flush=True)
            for attempt in outcome.attempts:
                print(f"    {attempt}", flush=True)
    return goal, list(review.value)


def plan_from_goal(
    n_parts: int,
    state: list[str],
    engine: str = "auto",
    blocked: list[tuple[str, dict[str, str]]] | None = None,
    clamps: list[tuple[str, str]] | None = None,
    goal: list[str] | None = None,
) -> dict[str, Any]:
    """Plan the mission from ``state`` with a task planner (``--planner up``):
    the goal (``screw_domain.mission_goal``) as PDDL, a skeleton from Unified
    Planning, then a TaskPlan document (validated when the session loads it)."""
    from long_tamp.tasks.task_planning.skeleton import default_planner, planner_name

    t0 = time.time()
    task_planner = default_planner(engine)
    name = planner_name(task_planner)
    steps = task_planner.solve(pddl_problem(n_parts, state, blocked, clamps, goal))
    print(
        f"task planner ({name}): {len(steps)} steps in {time.time() - t0:.2f}s",
        flush=True,
    )
    return planned_document(steps, n_parts, state, generator=name)


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
    backend=None,
    on_event=None,
    document: dict[str, Any] | None = None,
    inject: list[dict[str, Any]] | None = None,
    plan_ahead: bool = False,
    max_drift: float | None = None,
    concurrent: bool = False,
    control=None,
) -> dict[str, Any]:
    """Run the mission's TaskPlan from ``q_start`` (default: the scene start).

    The plan runs on a ``PlanExecutor``: each step plans its block, then its
    motion executes on ``backend`` (``None``: planning only, the default).

    Steps run in plan order through a TaskPlanningSession; a step whose
    effect already holds in the world (grasp tracker + ``recorded`` facts) is
    skipped, which is how ``--resume`` continues a run. If ``trajectory`` is a
    list, each completed block's motion is sampled into it; if ``checkpoint``
    is given, every block is logged to it. ``on_event`` receives the
    mission's event stream (``long_tamp.tasks.task_planning.events``).
    ``document`` is the TaskPlan to run (default: the hand-written
    ``build_plan_document``; see ``plan_from_goal``). ``control`` (an
    ``ExecutionControl``) pauses, resumes or stops it between steps.
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
        "backend_displays": getattr(backend, "display", None) is not None,
        "inject": inject if inject is not None else [],
        "concurrent": concurrent,
    }
    session = mission_session(
        task, planner, ctx, recorded, max_replans, verbose, document=document
    )
    executor = PlanExecutor(
        session,
        backend=backend,
        policy=ExecutionPolicy(max_start_drift=max_drift),
        control=control,
        on_skip=lambda label, why: print(f"\n=== {label} === skipped ({why})"),
        on_event=on_event,
        on_drift=ctx.get("on_drift"),
        plan_ahead=plan_ahead,
        concurrent=concurrent,
        # merged motion of two lanes: checked for collisions as a whole
        validate_config=lambda q: planner.config_gen.is_config_valid(list(q))[0],
    )
    ctx["executor"] = executor
    if on_event is not None:
        on_event(plan_event(session.plan))  # the viewer draws this plan
    t_mission = time.time()
    run = executor.run()
    if not run.success:
        print(f"\n=== {run.failed_step} stopped the mission: {run.message}")
    success, skipped = run.success, run.skipped
    return {
        "success": success,
        "seconds": round(time.time() - t_mission, 2),
        "blocks": ctx["records"],
        "skipped": skipped,
        "executed": len(run.executions),
        "execution": execution_summary(run.executions),
        "timing": run.timing or None,
        "final_config": ctx["q"],
        "failure": (
            None if run.success else ctx.get("failure") or execution_failure(run)
        ),
    }


from long_tamp.visualization.mission_viewer import MissionViewer


def make_backend(
    name: str,
    task: ScrewAssemblyTask,
    live_viewer=None,
    run_dir: Path | None = None,
    hole_error: tuple[float, float, float] = (0.0, 0.0, 0.0),
    speed: float = math.inf,
    drift: list[tuple[str, str, float]] | None = None,
    grasp: str = "weld",
    record: bool = False,
):
    """The execution backend for ``--backend``."""
    if name == "mock":
        return MockBackend(rtf=1000.0)
    if name == "playback":
        display = (lambda q: task.planner.viewer(q)) if live_viewer else None
        return PathPlaybackBackend(display=display)
    if name == "mujoco":
        from long_tamp.sim import MuJoCoBackend, QposMap, ScrewDriving, export_mjcf

        export = export_mjcf(CONFIG, (run_dir or HERE / "runs") / "mjcf")
        to_qpos = QposMap(export.load(), task.robot.model())
        # The simulation runs in its own process, like a robot controller:
        # stepping it from a thread would share the planner's interpreter
        # lock (see long_tamp.execution.process).
        contact = contact_grasps(export) if grasp == "contact" else {}
        backend = ProcessBackend(  # closed at exit (see main)
            MuJoCoBackend,
            export,
            to_qpos,
            speed=speed,
            skills={"screw": ScrewDriving(hole_error=hole_error)},
            from_qpos=to_qpos.inverse,
            grasp=grasp,
            record=(run_dir or HERE / "runs") / "sim" if record else None,
            **contact,
        )
        return DriftInjector(backend, drift) if drift else backend
    return None


def robotiq_pads(arm: str) -> dict[str, tuple[tuple, tuple]]:
    """Box pads for the MuJoCo backend on an arm's 2F-85: on the pad frames
    (the fingertips' inner faces), 1 mm thick behind the face, the size of
    ROBOTIQ_2F85's pad."""
    from long_tamp.grasping import ROBOTIQ_2F85

    half = (0.001, ROBOTIQ_2F85.pad_width / 2, ROBOTIQ_2F85.pad_length / 2)
    return {
        f"{arm}/robotiq_85_{side}_finger_pad": ((sign * 0.001, 0.0, 0.0), half)
        for side, sign in (("left", 1.0), ("right", -1.0))
    }


def contact_grasps(export) -> dict[str, Any]:
    """``--grasp contact``: the MuJoCo backend's fingers, grip table and pads.
    Each arm has one handle per object, so (object, arm) picks the closure."""
    from long_tamp.sim import GripTable

    model = export.load()
    closures = finger_closures()
    table = {}
    for gripper, handle in closures.pairs():
        arm, obj = gripper.split("/")[0], handle.split("/")[0]
        if obj not in export.free_joints:
            continue  # a part this scene doesn't have
        joint = model.joint(export.free_joints[obj]).id
        root = model.body(int(model.jnt_bodyid[joint])).name
        table[(root, arm)] = closures.closed_values(gripper, handle)
    return {
        "fingers": tuple(
            f"{arm}/robotiq_85_left_knuckle_joint" for arm in (LEFT, RIGHT)
        ),
        "grip": GripTable(table),
        "pads": {**robotiq_pads(LEFT), **robotiq_pads(RIGHT)},
    }


class DriftInjector:
    """``--inject-drift``: bump a joint just before a step's motion runs, as
    if someone knocked the robot while the next step was being planned. The
    executor's drift check (``start_error``) then sees it."""

    def __init__(self, backend, drift: list[tuple[str, str, float]]):
        self._backend = backend
        self._pending = list(drift)

    def start_error(self, command):
        for item in list(self._pending):
            label, joint, delta = item
            if command.step_id == label:
                self._backend.disturb(joint, delta)
                self._pending.remove(item)
                print(f"\n=== {label} === INJECTED DRIFT: {joint} {delta:+.3f} rad")
        return self._backend.start_error(command)

    def __getattr__(self, name):
        return getattr(self._backend, name)


def parse_drift(spec: str) -> tuple[str, str, float]:
    """``--inject-drift "LABEL:JOINT:RAD"``."""
    label, joint, delta = spec.rsplit(":", 2)
    return label, joint, float(delta)


def execution_failure(run) -> dict[str, Any] | None:
    """The failure facts of a step whose execution failed (a skill reporting
    ``screw_misaligned``, say), in the shape planning failures have."""
    failed = [e for e in run.executions if e.result.status.value != "success"]
    if not failed:
        return None
    last = failed[-1]
    return {
        "step": last.command.step_id,
        "facts": list(last.result.facts),
        "message": last.result.message,
    }


def execution_summary(executions) -> dict[str, Any] | None:
    """What the backend measured over the mission (the MuJoCo backend's
    tracking error and drift), from ``PlanRun.executions``."""
    if not executions:
        return None
    metrics = [e.result.metrics for e in executions]

    def worst(key):
        values = [m[key] for m in metrics if key in m]
        return max(values) if values else None

    return {
        "commands": len(executions),
        "failed": sum(1 for e in executions if e.result.status.value != "success"),
        "sim_seconds": round(sum(m.get("sim_seconds", 0.0) for m in metrics), 2),
        "max_tracking_error": worst("tracking_error"),
        "max_drift": worst("drift"),
        "max_object_drift": worst("object_drift"),
        "max_start_drift": worst("start_drift"),
        "screws_driven": sum(
            1 for e in executions for f in e.result.facts if f.startswith("screwed(")
        ),
        "max_screw_lateral_error": worst("lateral_error"),
    }


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
        "--web-port",
        type=int,
        help="serve the web mission viewer (plan, timeline, events; pause, resume "
        "and stop) on this port while the mission runs, e.g. 8090; it embeds the "
        "Viser scene unless --no-viewer (python -m long_tamp.viewer replays a run)",
    )
    ap.add_argument(
        "--backend",
        choices=("none", "mock", "playback", "mujoco"),
        default="none",
        help="what executes each block's motion after it is planned: none "
        "(planning only, the default), mock (instant, for testing the pipeline), "
        "playback (plays the paths in real time, in the viewer if it is on) or "
        "mujoco (simulates them under tracking control; needs the sim extra)",
    )
    ap.add_argument(
        "--grasp",
        choices=("weld", "contact"),
        default="weld",
        help="with --backend mujoco: how grasps hold objects: weld (the "
        "default) or contact (the Robotiq fingers close and hold them by "
        "friction)",
    )
    ap.add_argument(
        "--sim-record",
        action="store_true",
        help="with --backend mujoco: record the simulation to <run folder>/sim, "
        "for view_mujoco.py",
    )
    ap.add_argument(
        "--concurrent",
        action="store_true",
        help="run independent steps (the other arm's) at the same time: their "
        "motions are merged and checked for collisions (#21)",
    )
    ap.add_argument(
        "--plan-ahead",
        action="store_true",
        help="plan each step while the previous step's motion executes",
    )
    ap.add_argument(
        "--max-drift",
        type=float,
        default=0.05,
        help="with a backend that reports it: replan a step whose robot is "
        "further than this (rad) from where its plan starts (default 0.05)",
    )
    ap.add_argument(
        "--sim-speed",
        type=float,
        default=math.inf,
        help="with --backend mujoco: simulated seconds per wall-clock second "
        "(default: as fast as possible; 1 = real time)",
    )
    ap.add_argument(
        "--inject-drift",
        action="append",
        default=[],
        metavar="LABEL:JOINT:RAD",
        help="with --backend mujoco: bump JOINT by RAD just before step LABEL's "
        "motion (repeatable), e.g. 'part1 B: release:ur10_left/elbow_joint:0.2'",
    )
    ap.add_argument(
        "--hole-error",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
        help="with --backend mujoco: where the real holes are relative to the "
        "planned ones, in mm (a perception error for the screw skill)",
    )
    ap.add_argument(
        "--planner",
        choices=("none", "up"),
        default="none",
        help="how the block order is decided: none (the hand-written plan, the "
        "default) or up (a task planner plans the goal from the current world "
        "state; needs the 'planning' extra)",
    )
    ap.add_argument(
        "--instruction",
        metavar="TEXT",
        help="what to do, in words (e.g. 'assemble part 2'): Claude writes the "
        "goal, the task planner plans it (implies --planner up; needs the "
        "'language' extra and Anthropic API credentials)",
    )
    ap.add_argument(
        "--goal-model",
        metavar="API:MODEL",
        help="with --instruction: the model that writes the goal, as <api>:<model> "
        "with <api> anthropic or openai (any OpenAI-compatible endpoint); "
        "default anthropic:claude-opus-5-5. See docs/usage/ai-models.md",
    )
    ap.add_argument(
        "--chat",
        action="store_true",
        help="talk to a model that plans and runs missions through checked tools "
        "(set the goal, add constraints, plan, run, explain a failure); reads "
        "operator messages from stdin (#89), and from the web viewer's chat panel "
        "with --web-port (#90)",
    )
    ap.add_argument(
        "--supervise",
        action="store_true",
        help="with --instruction: when the repair loop gives up, a model decides "
        "(retry, relax the goal, abort, escalate) within limits; stops write an "
        "escalation report to the run folder (#88)",
    )
    ap.add_argument(
        "--ai-env",
        metavar="FILE",
        type=Path,
        help="env file with the model endpoints and keys (default: "
        "$LONG_TAMP_AI_ENV); read like a shell reads it",
    )
    ap.add_argument(
        "--replan",
        type=int,
        default=0,
        metavar="ROUNDS",
        help="with --planner up: when a step fails, block what failed "
        "(screw_domain.repair_policy) and replan from the world state, up to "
        "ROUNDS plans in all (default 0: no replanning)",
    )
    ap.add_argument(
        "--inject-failure",
        action="append",
        default=[],
        metavar="CAPABILITY:KEY=VALUE,...",
        help="fail the first matching step once, as if its first phase were "
        "unreachable, e.g. clamp_and_screw:clamp=fixtures/clamp1 (testing)",
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
        "point), facts.json (recorded facts), events.jsonl (event stream), "
        "trajectory.json, run.log, phases/. Default: runs/seed<S>_<timestamp>/",
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
    if args.instruction or args.chat:
        args.planner = "up"  # the instruction is a goal: a task planner plans it
    if args.chat and (args.instruction or args.supervise or args.replan):
        ap.error("--chat runs on its own (no --instruction, --supervise or --replan)")
    if args.supervise and not args.instruction:
        ap.error("--supervise needs --instruction")
    if args.resume and args.run_dir is None:
        ap.error("--resume needs --run-dir")
    if args.replan and args.planner != "up":
        ap.error("--replan needs --planner up")
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

    # The event stream, appended to on --resume, so one file covers the run.
    events = JsonlEventWriter(run_dir / "events.jsonl")
    atexit.register(events.close)

    closures = finger_closures()
    for row in closures.report():
        if row["handle"].startswith(("driver", "part1")):
            print(
                f"grasp {row['gripper']} > {row['handle']}: close to "
                f"{row['width'] * 1000:.1f} mm (robotiq_85_left_knuckle_joint {row['q']:.3f})"
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
    control, web = None, None
    if args.web_port is not None:
        from long_tamp.execution import ExecutionControl
        from long_tamp.viewer import ViewerConfig, ViewerServer

        control = ExecutionControl()
        web = ViewerServer(
            run_dir / "events.jsonl",
            ViewerConfig(
                title=f"Screw assembly, seed {args.seed}",
                scene_url=(
                    None
                    if live_viewer is None
                    else f"http://localhost:{args.viewer_port}"
                ),
            ),
            port=args.web_port,
            control=control,
        )
        print(f"web viewer: {web.start()}", flush=True)
        atexit.register(web.close)
    try:
        inject = [parse_injection(spec) for spec in args.inject_failure]
        mission = dict(
            max_replans=args.max_replans,
            trajectory=trajectory,
            checkpoint=checkpoint,
            live_viewer=live_viewer,
            recorded=recorded,
            backend=make_backend(
                args.backend,
                task,
                live_viewer,
                run_dir,
                hole_error=tuple(v / 1000.0 for v in args.hole_error),
                speed=args.sim_speed,
                grasp=args.grasp,
                record=args.sim_record,
                drift=[parse_drift(spec) for spec in args.inject_drift],
            ),
            plan_ahead=args.plan_ahead,
            concurrent=args.concurrent,
            max_drift=args.max_drift,
            on_event=events,
            inject=inject,
            control=control,
        )
        goal, constraints = None, []
        client = None
        if args.instruction or args.chat:
            from long_tamp.ai import configure

            configure(args.ai_env)  # endpoints, keys; localhost in a container

            def on_call(record):
                events(
                    make_event(
                        "ai",
                        "model",
                        record.role,
                        "FAILURE" if record.error else "SUCCESS",
                        message=record.error or "",
                        metrics=record.metrics(),
                    )
                )

            from long_tamp.ai import make_client

            client = make_client(args.goal_model, on_call=on_call)
        if args.chat:
            result = run_chat(
                task, planner, n_parts, q_start, recorded, mission, client, events, web
            )
        elif args.instruction:
            goal, constraints = understand_instruction(
                args.instruction,
                n_parts,
                world_atoms(planner, recorded),
                clamps=clamp_seats(task.task_config.VALID_PAIRS),
                client=client,
            )
        if args.chat:
            pass  # the chat ran the missions it was asked to
        elif args.supervise:
            result = run_supervised(
                task,
                planner,
                n_parts,
                q_start,
                recorded,
                args.replan or 3,
                mission,
                args.instruction,
                goal,
                constraints,
                client,
                run_dir,
            )
        elif args.replan:
            result = run_with_repair(
                task,
                planner,
                n_parts,
                q_start,
                recorded,
                args.replan,
                mission,
                goal,
                constraints,
            )
        else:
            result = run_mission(
                task,
                planner,
                n_parts,
                q_start=q_start,
                document=(
                    plan_from_goal(
                        n_parts,
                        world_atoms(planner, recorded),
                        clamps=clamp_seats(task.task_config.VALID_PAIRS),
                        goal=goal,
                        blocked=constraints,
                    )
                    if args.planner == "up"
                    else None
                ),
                **mission,
            )
    except BaseException:
        if live_viewer is not None:
            live_viewer.close()
        raise
    backend = mission.get("backend")
    if hasattr(backend, "close"):
        backend.close()  # a simulation process: stop and reap it
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
