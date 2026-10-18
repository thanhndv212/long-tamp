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
from long_tamp.mission import MissionRunner, MissionScene  # noqa: E402
from long_tamp.mission.runner import parse_injection, write_trajectory  # noqa: E402
from long_tamp.mission.ui import RESTART, parse_watchdog  # noqa: E402, F401
from long_tamp.tasks.task_planning import (  # noqa: E402
    CompositeWorldState,
    GraspTrackerState,
    RecordedFacts,
)
from long_tamp.tasks.task_planning.events import (  # noqa: E402
    JsonlEventWriter,
    make_event,
    plan_event,
)
from long_tamp.execution import (  # noqa: E402
    MockBackend,
    PathPlaybackBackend,
    ProcessBackend,
    RemoteBackend,
)
# Also re-exported: chat_tools, scenarios and screw_bt_session reach the
# domain's functions through this module.
from screw_domain import (  # noqa: E402, F401
    LEFT,
    RECORDED_PREDICATES,
    RIGHT,
    blocks_by_label,
    build_plan_document,
    clamp_seats,
    descriptors,
    goal_vocabulary,
    pddl_problem,
    planned_document,
)
from screw_mission import ScrewAssemblyDomain  # noqa: E402

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


#: ``run_mission`` options that configure the runner, not the run.
RUNNER_OPTIONS = ("max_replans", "recorded", "verbose")


def mission_runner(
    task: ScrewAssemblyTask,
    planner: GraspSequencePlanner,
    n_parts: int,
    recorded: RecordedFacts | None = None,
    max_replans: int = 10,
    verbose: bool = True,
) -> MissionRunner:
    """The mission on the generic runner (ADR-0012): the screw domain in
    this scene. ``recorded`` is the world's recorded facts (default: none
    yet); pass the same one to every runner of a mission."""
    domain = ScrewAssemblyDomain(n_parts, clamp_seats(task.task_config.VALID_PAIRS))
    scene = MissionScene(task, planner, list(task.q_init))
    return MissionRunner(
        domain, scene, recorded, max_replans=max_replans, verbose=verbose
    )


def split_options(mission: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    """``mission`` as (``mission_runner`` options, ``run_mission`` options)."""
    config = {k: v for k, v in mission.items() if k in RUNNER_OPTIONS}
    return config, {k: v for k, v in mission.items() if k not in RUNNER_OPTIONS}


def run_mission(
    task: ScrewAssemblyTask,
    planner: GraspSequencePlanner,
    n_parts: int,
    **mission: Any,
) -> dict[str, Any]:
    """Run one TaskPlan of the mission (``MissionRunner.run_mission``: the
    options ``q_start``, ``document``, ``backend``, ``trajectory``,
    ``checkpoint`` ...), on a runner configured by ``max_replans``,
    ``recorded`` and ``verbose`` (``mission_runner``)."""
    config, options = split_options(mission)
    return mission_runner(task, planner, n_parts, **config).run_mission(**options)


def run_chat(runner, q_start, mission, client, events, web=None, watchdog=None):
    """The operator chat (``--chat``, #89) on the library's generic chat
    tools (``long_tamp.mission.chat``, #133)."""
    from long_tamp.mission.chat import MissionChat
    from long_tamp.mission.chat import run_chat as chat

    work = MissionChat(runner, split_options(mission)[1], q_start, client)
    return chat(work, client, events, web, watchdog)


def world_atoms(planner: GraspSequencePlanner, recorded: RecordedFacts) -> list[str]:
    """The world state as ground atoms: held grasps plus recorded facts."""
    state = CompositeWorldState(GraspTrackerState(planner), recorded)()
    return sorted(str(atom) for atom in state)


#: How far an object must be from where it was planned to be taken where it
#: is observed [m] (a part knocked out of place, #128).
MOVED = 0.005


def observed_start(task, planner, planned, observed) -> list[float]:
    """The configuration to replan from: the arms and the objects where the
    backend observes them, but a hand that carries something as planned, and
    what it carries where it was planned in that hand. An observed carried object sits a
    little off its grasp (contact grasps slip) and an observed closed hand
    squeezes into it; the planner would keep both, and every later
    configuration would have a finger in the driver (#128). An object resting
    in a fixture (a part in the rack) is taken where it is: it may have been
    knocked out of place, which is what a retry is for."""
    import pinocchio as pin

    model = task.robot.model()
    # Within the planner's bounds: a simulated finger at -1e-9 rad is
    # "out of range" to HPP, which then rejects the start configuration.
    observed = np.clip(observed, model.lowerPositionLimit, model.upperPositionLimit)
    holder = {  # held object -> what holds it (an arm, an object, a fixture)
        h.split("/")[0]: g.split("/")[0]
        for g, h in planner.grasp_tracker.current_grasps.items()
        if h
    }
    arms = {
        name.split("/")[0]: model.parents[jid]  # arm -> its wrist joint
        for jid, name in enumerate(model.names)
        if name.endswith("robotiq_85_left_knuckle_joint")
    }

    def arm_of(body: str) -> str | None:
        """The arm carrying ``body``, through what holds it, if any."""
        for _ in range(len(holder) + 1):
            if body in arms:
                return body
            if body not in holder:
                return None
            body = holder[body]
        return None

    carried = {obj: arm for obj in holder if (arm := arm_of(obj)) is not None}
    q = [float(v) for v in planned]
    objects = {f"{o}/root_joint" for o in task.task_config.OBJECTS}
    for jid in range(1, model.njoints):
        name, r, nq = model.names[jid], model.idx_qs[jid], model.nqs[jid]
        body = name.split("/")[0]
        moved = np.linalg.norm(np.subtract(observed[r : r + 3], planned[r : r + 3]))
        if name in objects and moved < MOVED:
            # Where the plan's constraints put it, not 10 µm off in the
            # simulation: only an object that really moved is re-placed.
            continue
        # A hand that carries something stays as planned; an empty one (the
        # hand a retry just opened) is taken as it is, or the replanned
        # approach would arrive closed.
        if body in carried or ("robotiq" in name and body in carried.values()):
            continue
        q[r : r + nq] = [float(v) for v in observed[r : r + nq]]
    # A carried object moves with its arm's wrist, from where it was planned.
    before, after = model.createData(), model.createData()
    pin.forwardKinematics(model, before, np.asarray(planned, dtype=float))
    pin.forwardKinematics(model, after, np.asarray(q, dtype=float))
    for obj, arm in carried.items():
        jid = model.getJointId(f"{obj}/root_joint")
        if jid >= model.njoints:
            continue
        wrist = arms[arm]
        m = after.oMi[wrist] * before.oMi[wrist].inverse()
        r = model.idx_qs[jid]
        pose = pin.XYZQUATToSE3(np.asarray(planned[r : r + 7], dtype=float))
        q[r : r + 7] = [float(v) for v in pin.SE3ToXYZQUAT(m * pose)]
    return q


from long_tamp.visualization.mission_viewer import MissionViewer


def make_backend(
    name: str,
    task: ScrewAssemblyTask,
    live_viewer=None,
    run_dir: Path | None = None,
    hole_error: tuple[float, float, float] = (0.0, 0.0, 0.0),
    speed: float = math.inf,
    drift: list[tuple[str, str, float]] | None = None,
    miss: list[tuple[str, str, tuple[float, float, float]]] | None = None,
    grasp: str = "weld",
    record: bool = False,
    remote: bool = False,
):
    """The execution backend for ``--backend``; with ``remote``, the
    simulation is served over TCP (``--remote``, ADR-0007) instead of a pipe."""
    if name == "mock":
        return MockBackend(rtf=1000.0)
    if name == "playback":
        display = live_viewer.display if live_viewer else None
        return PathPlaybackBackend(display=display)
    if name == "mujoco":
        from long_tamp.sim import MuJoCoBackend, QposMap, ScrewDriving, export_mjcf

        export = export_mjcf(CONFIG, (run_dir or HERE / "runs") / "mjcf")
        to_qpos = QposMap(export.load(), task.robot.model())
        # The simulation runs in its own process, like a robot controller:
        # stepping it from a thread would share the planner's interpreter
        # lock (see long_tamp.execution.process).
        grasps = contact_grasps(export) if grasp == "contact" else weld_grasps(export)
        # A scene in its own process takes the simulated states straight
        # from the simulation's process (#106): what the robot does, not
        # what was planned.
        scene = getattr(live_viewer, "scene", None)
        display = scene.client if scene is not None else None
        serve = RemoteBackend.spawn if remote else ProcessBackend
        backend = serve(  # closed at exit (see main)
            MuJoCoBackend,
            export,
            to_qpos,
            speed=speed,
            skills={"screw": ScrewDriving(hole_error=hole_error)},
            from_qpos=to_qpos.inverse,
            grasp=grasp,
            record=(run_dir or HERE / "runs") / "sim" if record else None,
            display=display,
            **grasps,
        )
        backend.display = display  # the planned paths need not be played
        backend = DriftInjector(backend, drift) if drift else backend
        return MissInjector(backend, miss) if miss else backend
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


def grip_table(export, closures):
    """(object root body, arm) -> the finger closure and the expected grasp.
    Each arm has one handle per object, so (object, arm) picks the closure."""
    from long_tamp.sim import GripTable

    model = export.load()
    table, expected = {}, {}
    for gripper, handle in closures.pairs():
        arm, obj = gripper.split("/")[0], handle.split("/")[0]
        if obj not in export.free_joints:
            continue  # a part this scene doesn't have
        joint = model.joint(export.free_joints[obj]).id
        root = model.body(int(model.jnt_bodyid[joint])).name
        table[(root, arm)] = closures.closed_values(gripper, handle)
        expected[(root, arm)] = closures.expected_grasp(gripper, handle)
    return GripTable(table, expected)


def weld_grasps(export) -> dict[str, Any]:
    """Weld grasps: no fingers, but a missed one names its gripper and object
    (``grasp_missed``, ADR-0009) so the repair can retry it (#128)."""
    return {"expect": grip_table(export, finger_closures()).expect}


def contact_grasps(export) -> dict[str, Any]:
    """``--grasp contact``: the MuJoCo backend's fingers, grip table and pads."""
    closures = finger_closures()
    grip = grip_table(export, closures)
    return {
        "fingers": tuple(
            f"{arm}/robotiq_85_left_knuckle_joint" for arm in (LEFT, RIGHT)
        ),
        "grip": grip,
        "expect": grip.expect,
        "hands": tuple(closures.bindings.values()),
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


class MissInjector:
    """``--inject-miss``: move an object just before a step's motion starts,
    as if someone took it after it was seen. The step's grasp then closes on
    nothing and reports ``grasp_missed`` (#127); the repair opens the hand,
    observes the world and picks the object where it is now (#128)."""

    def __init__(self, backend, miss: list[tuple[str, str, tuple]]):
        self._backend = backend
        self._pending = list(miss)

    def start(self, command):
        for item in list(self._pending):
            label, obj, delta = item
            if command.step_id == label:
                self._backend.displace(obj, delta)
                self._pending.remove(item)
                print(f"\n=== {label} === INJECTED MISS: {obj} moved by {delta} m")
        return self._backend.start(command)

    def __getattr__(self, name):
        return getattr(self._backend, name)


def parse_miss(spec: str) -> tuple[str, str, tuple[float, float, float]]:
    """``--inject-miss "LABEL:OBJECT:DX,DY,DZ"``."""
    label, obj, delta = spec.rsplit(":", 2)
    values = tuple(float(v) for v in delta.split(","))
    if len(values) != 3:
        raise ValueError(f"--inject-miss {spec!r}: DX,DY,DZ in metres")
    return label, obj, values


def parse_drift(spec: str) -> tuple[str, str, float]:
    """``--inject-drift "LABEL:JOINT:RAD"``."""
    label, joint, delta = spec.rsplit(":", 2)
    return label, joint, float(delta)


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
        "--viewer-in-process",
        action="store_true",
        help="serve the Viser scene from the mission's process (before #106): it "
        "then freezes while HPP plans; by default it has its own process",
    )
    ap.add_argument(
        "--watchdog",
        metavar="SOFT[,HARD]",
        help="seconds a step may plan before the watchdog decides: wait, skip its "
        "search or abort it (the chat's model decides when there is one, else a rule); "
        "at HARD (default 3 x SOFT) it aborts. E.g. 300,900 (#109)",
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
        "--remote",
        action="store_true",
        help="with --backend mujoco: drive the simulation through the remote "
        "backend (TCP, as a robot on another host would be; ADR-0007) instead "
        "of a process pipe",
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
        "--inject-miss",
        action="append",
        default=[],
        metavar="LABEL:OBJECT:DX,DY,DZ",
        help="with --backend mujoco: move OBJECT by (DX, DY, DZ) m just before "
        "step LABEL's motion, so its grasp misses (repeatable), e.g. "
        "'part1 A0: grasp:part1:0,-0.10,0' (clear of the open fingers, which "
        "close across y); with --replan the step is retried where the object "
        "is (#128)",
    )
    ap.add_argument(
        "--grasp-retries",
        type=int,
        default=2,
        metavar="N",
        help="with --replan: retry a step whose grasp missed up to N times "
        "(open, observe, replan the pick), then give up (default 2; #128)",
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
        "default $LONG_TAMP_GOAL_MODEL (it may be set in the AI env file), else "
        "anthropic:claude-opus-5-5. See docs/usage/ai-models.md",
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
        "$LONG_TAMP_AI_ENV, else ~/.config/long-tamp/ai.env if it exists); read "
        "like a shell reads it",
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
    runner = mission_runner(task, planner, n_parts, recorded, args.max_replans)
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
            # its own process, so it stays live while HPP plans (#106)
            separate_process=not args.viewer_in_process,
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
            separate_process=True,  # planning holds the GIL for seconds
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
                remote=args.remote,
                drift=[parse_drift(spec) for spec in args.inject_drift],
                miss=[parse_miss(spec) for spec in args.inject_miss],
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
        watchdog = None
        if args.watchdog:
            from long_tamp.execution import ExecutionControl
            from long_tamp.execution.watchdog import StepWatchdog, model_decider

            if control is None:
                control = mission["control"] = ExecutionControl()
            soft, hard = parse_watchdog(args.watchdog)
            watchdog = StepWatchdog(
                control,
                soft,
                hard,
                decide=model_decider(client) if client is not None else None,
                sink=events,
                on_decision=lambda watch, decision: print(
                    f"watchdog: {watch.label}: {decision.action} ({decision.by}): "
                    f"{decision.reason}",
                    flush=True,
                ),
            ).start()
            atexit.register(watchdog.close)

            def watched(event, _write=events):
                _write(event)
                watchdog.observe(event)

            mission["on_event"] = watched
        if args.chat:
            result = run_chat(
                runner, q_start, mission, client, events, web, watchdog
            )
        elif args.instruction:
            goal, constraints = runner.understand_instruction(
                args.instruction, client=client
            )
        if args.chat:
            pass  # the chat ran the missions it was asked to
        elif args.supervise:
            result = runner.run_supervised(
                q_start,
                args.replan or 3,
                split_options(mission)[1],
                args.instruction,
                goal,
                constraints,
                client,
                run_dir,
                args.grasp_retries,
            )
        elif args.replan:
            result = runner.run_with_repair(
                q_start,
                args.replan,
                split_options(mission)[1],
                goal,
                constraints,
                args.grasp_retries,
            )
        else:
            result = runner.run_mission(
                q_start=q_start,
                document=(
                    runner.plan_from_goal(goal=goal, blocked=constraints)
                    if args.planner == "up"
                    else None
                ),
                **split_options(mission)[1],
            )
    except BaseException:
        if live_viewer is not None:
            live_viewer.close()
        raise
    backend = mission.get("backend")
    if hasattr(backend, "close"):
        backend.close()  # a simulation process: stop and reap it
    checkpoint.finish(result["success"], result["seconds"])
    write_trajectory(traj_path, trajectory, runner.domain.scene_info())
    result["seed"] = args.seed
    result["parts"] = n_parts
    print(
        f"\n{'MISSION COMPLETE' if result['success'] else 'MISSION FAILED'} "
        f"in {result['seconds']}s  (run log: {run_dir / 'mission.json'})"
    )
    if args.summary:
        summary = {k: v for k, v in result.items() if k != "final_config"}
        args.summary.write_text(json.dumps(summary, indent=2))
    if result.get("reset"):
        if live_viewer is not None:
            live_viewer.close()
        return RESTART  # the mission UI launcher starts a fresh mission
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
