"""Execute planned paths in MuJoCo: an execution backend with a controller in the loop.

``MuJoCoBackend(export, to_qpos)`` implements the execution contract
(``long_tamp.execution``, ADR-0004) on the scene ``export_mjcf`` wrote:

- **Tracking control.** Every robot joint that isn't a mimic follower gets a
  torque motor driven by a PD controller on the path's position and velocity,
  with gravity and Coriolis compensation. Gains are scheduled on the joint's
  apparent inertia (plus its mimic followers') for a critically damped
  response at ``bandwidth`` Hz; torques are limited to the URDF's effort, and
  the damping runs through MuJoCo's implicit integrator. The Robotiq fingers are joints like
  the others, so the grippers open and close to the widths the planner chose,
  and the mimic joints follow through their equality constraints.
- **Grasps are welds.** Contact-rich grasping isn't simulated. At the start of
  each command, every object is welded to what carries it in the plan: a robot
  link it moves rigidly with, or the world when it doesn't move. A new grasp
  snaps the object into the planner's grasp, as closing fingers would, when it
  is within ``grasp_tolerance`` (else the command fails: the grasp missed,
  ``grasp_error`` says by how much); a release leaves it where the simulation
  put it, so placement error shows as ``object_drift``. An object that moves with nothing is driven along
  its planned pose (reported as ``kinematic_objects``).
- **Retiming.** Planned paths keep their geometry, which the planner checked
  for collisions, but not always a timing a robot can follow: they may start
  at full speed or turn corners instantly. Each path is retimed along its own
  parameter, from rest to rest, never faster than planned, within the joints'
  URDF velocity limits and ``max_acceleration`` (slowing through corners),
  then stretched until inverse dynamics needs no more than ``torque_margin``
  of any joint's effort. ``time_scale`` reports the retimed duration over the
  planned one.
- **Contacts are off** by default (``contacts=True`` turns them on): paths are
  collision-free in the planner, and with welded grasps contacts only add
  jitter.
- **Metrics.** Every heartbeat carries ``tracking_error`` (the largest joint
  error along the command so far, rad) and, at the end, ``drift`` (joint error
  after the command, rad), ``object_drift`` (the largest object position error
  against the plan, m) and ``start_drift`` (how far the robot was from the
  path's start when the command began, rad). The executor puts them in the
  command's ``motion`` event.

The simulation starts at the first command's start configuration and then
runs continuously: each command starts from wherever the previous one left the
robot and the objects. ``speed`` is simulated seconds per wall-clock second
(``math.inf``: as fast as possible, one poll per command).
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

import numpy as np

from long_tamp.execution.contract import ExecutionCommand, ExecutionStatus, Feedback
from long_tamp.execution.playback import _t0
from long_tamp.tasks.task_planning.skills import SkillCommand

from .mjcf import MjcfExport

#: Samples along a path used to decide what carries each object.
CARRIER_SAMPLES = 7
#: Position [m] / orientation [rad] change below which a pose is "constant".
RIGID_TOLERANCE = 1e-3


def _quat_distance(q1: np.ndarray, q2: np.ndarray) -> float:
    return 2.0 * math.acos(min(1.0, abs(float(np.dot(q1, q2)))))


class MuJoCoBackend:
    """Runs time-parameterized paths in MuJoCo under tracking control.

    ``scene`` is an :class:`MjcfExport` or an MJCF path; ``to_qpos`` maps a
    planner configuration to MuJoCo ``qpos`` (a :class:`QposMap`).
    ``command.payload`` is a path (``length()``, ``eval(t) -> (q, ok)``) or a
    key ``get_path`` resolves. ``max_tracking_error`` (rad) fails a command
    whose tracking error exceeds it; ``None`` only reports it.
    """

    def __init__(
        self,
        scene: MjcfExport | str | Path,
        to_qpos: Callable[[Any], np.ndarray],
        *,
        get_path: Callable[[Any], Any] | None = None,
        speed: float = 1.0,
        timestep: float = 0.002,
        bandwidth: float = 10.0,
        settle: float = 0.2,
        contacts: bool = False,
        max_tracking_error: float | None = None,
        grasp_tolerance: float = 0.02,
        max_acceleration: float = 3.0,
        torque_margin: float = 0.8,
        skills: dict[str, Callable[[MuJoCoBackend, SkillCommand], Any]] | None = None,
        from_qpos: Callable[[np.ndarray, Any], np.ndarray] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        import mujoco

        self.mujoco = mujoco
        path = scene.path if isinstance(scene, MjcfExport) else Path(scene)
        self.to_qpos = to_qpos
        self.get_path = get_path
        self.speed = speed
        self.settle = settle
        self.max_tracking_error = max_tracking_error
        self.grasp_tolerance = grasp_tolerance
        self.torque_margin = torque_margin
        #: Skill controllers by skill name: ``factory(backend, command)``
        #: returns an object whose ``poll()`` runs the skill (see
        #: ``long_tamp.sim.skills``). Other skills play their approach path.
        self.skills = dict(skills or {})
        #: Maps MuJoCo ``qpos`` back to a planner configuration
        #: (``QposMap.inverse``), for replanning from the observed state.
        self.from_qpos = from_qpos
        self._skill = None
        self.clock = clock
        self.model, self._objects, self._actuated = self._build(
            path, timestep, contacts
        )
        self._omega = 2.0 * math.pi * bandwidth
        self._qadr = self.model.jnt_qposadr[self._actuated]
        self._vadr = self.model.jnt_dofadr[self._actuated]
        self._followers = self._mimic_followers()
        #: Extra joint torques added to the controller's command (a skill's
        #: force feed-forward, see ``wrench_torques``); reset every command.
        self.feedforward = np.zeros(len(self._actuated))
        self._vlim = self._velocity_limits()
        self._vlim = np.where(np.isfinite(self._vlim), self._vlim, 1e6)
        self._alim = np.full(len(self._actuated), float(max_acceleration))
        # The URDF effort bounds the net torque (motor minus damping), which
        # _set_ctrl enforces; MuJoCo's own clamp on the motor alone would
        # clip the feed-forward that cancels the damping at speed.
        frc = self.model.jnt_actfrcrange[self._actuated]
        limited = self.model.jnt_actfrclimited[self._actuated].astype(bool)
        self._effort = np.where(
            limited, np.minimum(np.abs(frc[:, 0]), np.abs(frc[:, 1])), np.inf
        )
        self.model.jnt_actfrclimited[self._actuated] = 0
        # Dofs whose apparent inertia sets the gains: driven joints and their
        # mimic followers.
        self._dofs = np.array(
            sorted(
                set(self._vadr.tolist())
                | {f for group in self._followers for f, _ in group}
            ),
            dtype=int,
        )
        self._unit_rows = np.zeros((len(self._dofs), self.model.nv))
        self._unit_rows[np.arange(len(self._dofs)), self._dofs] = 1.0
        self.data = mujoco.MjData(self.model)
        self._scratch = mujoco.MjData(self.model)
        self._robot_bodies = self._bodies_outside(self._objects.values())
        self._initialized = False
        self._path: Any = None
        self.cancelled = False

    # -- model -----------------------------------------------------------

    def _build(self, path: Path, timestep: float, contacts: bool):
        mujoco = self.mujoco
        spec = mujoco.MjSpec.from_file(str(path))
        spec.option.timestep = timestep
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        if not contacts:
            spec.option.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
        model = spec.compile()

        followers = {
            model.joint(model.eq_obj1id[i]).name
            for i in range(model.neq)
            if model.eq_type[i] == int(mujoco.mjtEq.mjEQ_JOINT)
        }
        objects: dict[str, int] = {}  # object root body name -> body id
        for j in range(model.njnt):
            if model.jnt_type[j] == int(mujoco.mjtJoint.mjJNT_FREE):
                body = model.jnt_bodyid[j]
                objects[model.body(body).name] = int(body)
        object_bodies = self._subtrees(model, objects.values())
        actuated = [
            j
            for j in range(model.njnt)
            if model.jnt_type[j]
            # Compare ints: in MuJoCo 3.14 `x in (enum, ...)` is always False.
            in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE))
            and model.jnt_bodyid[j] not in object_bodies
            and model.joint(j).name not in followers
        ]

        for j in actuated:
            name = model.joint(j).name
            act = spec.add_actuator()
            act.name = f"{name}/motor"
            act.trntype = mujoco.mjtTrn.mjTRN_JOINT
            act.target = name
            # Effort limits apply to the net torque, in _set_ctrl.

        # One weld per object: world (retargeted at run time) -> object.
        for name in objects:
            eq = spec.add_equality()
            eq.name = f"{name}/carried"
            eq.type = mujoco.mjtEq.mjEQ_WELD
            eq.objtype = mujoco.mjtObj.mjOBJ_BODY
            eq.name1 = "world"
            eq.name2 = name
            eq.active = False
            # As stiff as the time step allows: less sag under the object's load.
            eq.solref = [2.0 * timestep, 1.0]
        model = spec.compile()
        actuated_ids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, model.joint(j).name)
            for j in actuated
        ]
        objects = {
            name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            for name in objects
        }
        return model, objects, actuated_ids

    @staticmethod
    def _subtrees(model: Any, roots) -> set[int]:
        roots = {int(r) for r in roots}
        inside = set()
        for b in range(model.nbody):
            a = b
            while a > 0:
                if a in roots:
                    inside.add(b)
                    break
                a = int(model.body_parentid[a])
        return inside

    def _bodies_outside(self, roots) -> list[int]:
        inside = self._subtrees(self.model, roots)
        return [b for b in range(1, self.model.nbody) if b not in inside]

    def _weld(self, obj: str) -> int:
        return self.mujoco.mj_name2id(
            self.model, self.mujoco.mjtObj.mjOBJ_EQUALITY, f"{obj}/carried"
        )

    @property
    def joint_names(self) -> list[str]:
        """The joints the controller drives."""
        return [self.model.joint(j).name for j in self._actuated]

    # -- state -----------------------------------------------------------

    def reset(self, planner_q: Any) -> None:
        """Put the simulation at a planner configuration, at rest."""
        mujoco = self.mujoco
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self.to_qpos(planner_q)
        mujoco.mj_forward(self.model, self.data)
        self._set_ctrl(self.data.qpos, np.zeros(self.model.nv))
        self._initialized = True

    def _pose(self, data: Any, body: int) -> tuple[np.ndarray, np.ndarray]:
        return data.xpos[body].copy(), data.xquat[body].copy()

    def _relative(self, data: Any, b1: int, b2: int) -> tuple[np.ndarray, np.ndarray]:
        """Pose of ``b2`` in ``b1``'s frame."""
        mujoco = self.mujoco
        rot = data.xmat[b1].reshape(3, 3)
        pos = rot.T @ (data.xpos[b2] - data.xpos[b1])
        inv, quat = np.zeros(4), np.zeros(4)
        mujoco.mju_negQuat(inv, data.xquat[b1])
        mujoco.mju_mulQuat(quat, inv, data.xquat[b2])
        return pos, quat

    def _carriers(self, samples: list[np.ndarray]) -> dict[str, int | None]:
        """What carries each object along the path: a body id, 0 (the world),
        or ``None`` (nothing rigidly: drive it kinematically)."""
        mujoco, scratch = self.mujoco, self._scratch
        frames = []
        for qpos in samples:
            scratch.qpos[:] = qpos
            mujoco.mj_kinematics(self.model, scratch)
            frames.append(
                (scratch.xpos.copy(), scratch.xquat.copy(), scratch.xmat.copy())
            )
        carriers: dict[str, int | None] = {}
        for name, obj in self._objects.items():
            x0, q0 = frames[0][0][obj], frames[0][1][obj]
            if all(
                np.linalg.norm(f[0][obj] - x0) < RIGID_TOLERANCE
                and _quat_distance(f[1][obj], q0) < RIGID_TOLERANCE
                for f in frames
            ):
                carriers[name] = 0
                continue
            # Of the links the object moves rigidly with, the one highest in
            # the kinematic tree: the gripper's palm rather than a closed
            # finger, which hangs on a weakly driven mimic joint.
            best, best_key = None, None
            for body in self._robot_bodies:
                rel = []
                for xpos, xquat, xmat in frames:
                    rot = xmat[body].reshape(3, 3)
                    inv, quat = np.zeros(4), np.zeros(4)
                    mujoco.mju_negQuat(inv, xquat[body])
                    mujoco.mju_mulQuat(quat, inv, xquat[obj])
                    rel.append((rot.T @ (xpos[obj] - xpos[body]), quat))
                dev = max(
                    max(np.linalg.norm(p - rel[0][0]), _quat_distance(q, rel[0][1]))
                    for p, q in rel
                )
                if dev >= RIGID_TOLERANCE:
                    continue
                key = (len(self._ancestors(body)), dev)
                if best_key is None or key < best_key:
                    best, best_key = body, key
            carriers[name] = best
        return carriers

    def _attach(self, carriers: dict[str, int | None], planned: np.ndarray) -> str:
        """Weld each object to its carrier; returns a failure message, or "".

        A weld whose carrier doesn't change is kept as it was (a grasp persists
        across commands). A new grasp (a robot link starts carrying the
        object) snaps the object into the planner's grasp, as closing fingers
        would, if it is within ``grasp_tolerance`` of it: otherwise the grasp
        missed. A new world weld (a release) keeps the object where the
        simulation put it, so placement error shows as ``object_drift``.
        """
        mujoco, scratch = self.mujoco, self._scratch
        scratch.qpos[:] = planned
        mujoco.mj_kinematics(self.model, scratch)
        for name, carrier in carriers.items():
            eq = self._weld(name)
            obj = self._objects[name]
            if carrier is None:
                self.data.eq_active[eq] = 0
                continue
            if self.data.eq_active[eq] and self.model.eq_obj1id[eq] == carrier:
                continue
            pos, quat = self._relative(self.data, carrier, obj)
            if carrier != 0:
                pos_plan, quat_plan = self._relative(scratch, carrier, obj)
                error = float(np.linalg.norm(pos - pos_plan))
                self._grasp_error = max(self._grasp_error, error)
                if error > self.grasp_tolerance:
                    return (
                        f"grasp missed: {name} is {error * 1000:.0f} mm from "
                        f"its planned grasp (tolerance "
                        f"{self.grasp_tolerance * 1000:.0f} mm)"
                    )
                pos, quat = pos_plan, quat_plan
            self.model.eq_obj1id[eq] = carrier
            self.model.eq_data[eq][:] = 0.0
            self.model.eq_data[eq][3:6] = pos
            self.model.eq_data[eq][6:10] = quat
            self.model.eq_data[eq][10] = 1.0  # torquescale
            self.data.eq_active[eq] = 1
        return ""

    def _retime(self) -> tuple[np.ndarray, np.ndarray]:
        """A time law along the planned path that the robot can follow.

        Planned paths keep their geometry (what the planner checked for
        collisions) but not necessarily a feasible timing: they may start at
        full speed or turn corners instantly. On a grid of the path's own
        parameter ``s``, the path speed ``ds/dt`` is capped so that every
        driven joint stays within its velocity limit and, through corners
        (curvature ``q''``), its acceleration limit; a forward and a backward
        pass then bound ``d2s/dt2`` by the acceleration limits, from rest to
        rest. Returns (times, s) samples of the law.
        """
        n = int(min(4000, max(50, self._length / 0.005)))
        grid = np.linspace(0.0, self._length, n + 1)
        if self._length <= 0:
            return np.array([0.0]), np.array([0.0])
        h = grid[1] - grid[0]
        q = np.array([self._reference(g)[self._qadr] for g in grid])
        dq = np.gradient(q, h, axis=0)
        ddq = np.gradient(dq, h, axis=0)
        vlim, alim = self._vlim, self._alim
        with np.errstate(divide="ignore", invalid="ignore"):
            cap = np.min(np.where(np.abs(dq) > 1e-9, vlim / np.abs(dq), np.inf), axis=1)
            cap = np.minimum(
                cap,
                np.min(
                    np.where(np.abs(ddq) > 1e-9, np.sqrt(alim / np.abs(ddq)), np.inf),
                    axis=1,
                ),
            )
            accel = np.min(
                np.where(np.abs(dq) > 1e-9, alim / np.abs(dq), np.inf), axis=1
            )
        cap = np.minimum(cap, 1.0)  # never faster than planned
        accel = np.minimum(accel, 1e6)
        sd = cap.copy()
        sd[0] = sd[-1] = 0.0
        for i in range(n):  # forward: accelerate within limits
            sd[i + 1] = min(sd[i + 1], math.sqrt(sd[i] ** 2 + 2.0 * h * accel[i]))
        for i in range(n, 0, -1):  # backward: decelerate within limits
            sd[i - 1] = min(sd[i - 1], math.sqrt(sd[i] ** 2 + 2.0 * h * accel[i]))
        mean = 0.5 * (sd[1:] + sd[:-1])
        dt = np.where(mean > 0, h / np.maximum(mean, 1e-12), 0.0)
        times = np.concatenate([[0.0], np.cumsum(dt)])
        return times * self._torque_scale(times, grid), grid

    def _torque_scale(self, times: np.ndarray, grid: np.ndarray) -> float:
        """How much to stretch a time law so that, by inverse dynamics, no
        driven joint needs more than ``torque_margin`` of its effort.

        Stretching time by ``k`` divides the velocity- and acceleration-
        dependent torques (inertia, Coriolis, centrifugal) by ``k**2``;
        gravity stays.
        """
        mujoco, model, scratch = self.mujoco, self.model, self._scratch
        duration = float(times[-1])
        if duration <= 0 or not np.isfinite(self._effort).any():
            return 1.0
        n = int(min(400, max(20, duration / 0.01)))
        h = duration / n
        qpos = [
            self._reference(float(np.interp(k * h, times, grid))) for k in range(n + 1)
        ]
        qvel = np.zeros((n + 2, model.nv))  # at rest before and after
        for k in range(n):
            mujoco.mj_differentiatePos(model, qvel[k + 1], h, qpos[k], qpos[k + 1])
        qacc = np.diff(qvel, axis=0) / h
        effort = self._effort * self.torque_margin
        scale = 1.0
        for k in range(n + 1):
            scratch.qpos[:] = qpos[k]
            scratch.qvel[:] = 0.0
            scratch.qacc[:] = 0.0
            mujoco.mj_inverse(model, scratch)
            gravity = scratch.qfrc_inverse[self._vadr].copy()
            scratch.qvel[:] = 0.5 * (qvel[k] + qvel[k + 1])
            scratch.qacc[:] = qacc[k]
            mujoco.mj_inverse(model, scratch)
            dynamic = np.abs(scratch.qfrc_inverse[self._vadr] - gravity)
            room = np.maximum(effort - np.abs(gravity), 1e-3 * effort)
            finite = np.isfinite(room) & (dynamic > 0)
            if finite.any():
                scale = max(
                    scale, float(np.sqrt(np.max(dynamic[finite] / room[finite])))
                )
        return scale

    def _velocity_limits(self) -> np.ndarray:
        """Each driven joint's velocity limit from the export (inf if none)."""
        from .mjcf import VELOCITY_LIMIT_PREFIX

        model = self.model
        limits = {}
        for i in range(model.nnumeric):
            name = model.numeric(i).name
            if name.startswith(VELOCITY_LIMIT_PREFIX):
                limits[name[len(VELOCITY_LIMIT_PREFIX) :]] = float(
                    model.numeric_data[model.numeric_adr[i]]
                )
        return np.array(
            [limits.get(model.joint(j).name, np.inf) for j in self._actuated]
        )

    def _mimic_followers(self) -> list[list[tuple[int, float]]]:
        """For each driven joint, the (dof, multiplier) of the joints mimicking it."""
        model, mujoco = self.model, self.mujoco
        followers: list[list[tuple[int, float]]] = [[] for _ in self._actuated]
        index = {j: i for i, j in enumerate(self._actuated)}
        for e in range(model.neq):
            if model.eq_type[e] != int(mujoco.mjtEq.mjEQ_JOINT):
                continue
            leader = int(model.eq_obj2id[e])
            if leader in index:
                follower = int(model.eq_obj1id[e])
                followers[index[leader]].append(
                    (int(model.jnt_dofadr[follower]), float(model.eq_data[e][1]))
                )
        return followers

    def _apparent_inertia(self) -> np.ndarray:
        """Each driven joint's apparent inertia, ``1 / (M^-1)_ii``: what it
        feels when the other joints move freely, the conservative value for
        stable gains (the diagonal ``M_ii`` can be many times larger on a
        coupled arm). A joint with mimic followers adds theirs, scaled by the
        multiplier squared."""
        solved = np.zeros_like(self._unit_rows)
        self.mujoco.mj_solveM(self.model, self.data, solved, self._unit_rows)
        apparent = 1.0 / np.maximum(
            solved[np.arange(len(self._dofs)), self._dofs], 1e-9
        )
        by_dof = dict(zip(self._dofs.tolist(), apparent))
        return np.array(
            [
                by_dof[dof] + sum(m * m * by_dof[f] for f, m in self._followers[i])
                for i, dof in enumerate(self._vadr)
            ]
        )

    def _payload(self) -> tuple[np.ndarray, np.ndarray]:
        """What carried objects add to each driven joint: inertia about its
        axis, and the torque their weight exerts on it.

        A weld is a constraint, not part of the joint-space inertia or of
        ``qfrc_bias``, so a gripper holding a 1 kg drill would otherwise be
        controlled as if empty (its wrist joints' own inertia is grams-scale).
        """
        model, data = self.model, self.data
        inertia = np.zeros(len(self._actuated))
        torque = np.zeros(len(self._actuated))
        for name, obj in self._objects.items():
            eq = self._weld(name)
            carrier = int(model.eq_obj1id[eq])
            if not data.eq_active[eq] or carrier == 0:
                continue
            ancestors = self._ancestors(carrier)
            mass = float(model.body_subtreemass[obj])
            com = data.subtree_com[obj]
            rot = data.ximat[obj].reshape(3, 3)
            body_inertia = rot @ np.diag(model.body_inertia[obj]) @ rot.T
            weight = mass * model.opt.gravity
            for i, j in enumerate(self._actuated):
                if model.jnt_bodyid[j] not in ancestors:
                    continue
                axis, anchor = data.xaxis[j], data.xanchor[j]
                lever = com - anchor
                inertia[i] += mass * float(np.sum(np.cross(axis, lever) ** 2))
                inertia[i] += float(axis @ body_inertia @ axis)
                torque[i] += float(axis @ np.cross(lever, weight))
        return inertia, torque

    def _ancestors(self, body: int) -> set[int]:
        chain = set()
        while body > 0:
            chain.add(body)
            body = int(self.model.body_parentid[body])
        return chain

    def _set_ctrl(self, qpos_ref: np.ndarray, qvel_ref: np.ndarray) -> None:
        """Joint torques for tracking ``qpos_ref`` / ``qvel_ref``.

        A PD law with gains scheduled on each joint's apparent inertia, for a
        critically damped response at ``bandwidth``. The damping is the
        joint's own (``dof_damping``), which the implicit integrator keeps
        stable at any gain; the motor adds the stiffness, the reference
        velocity times the damping (so the damping acts on the velocity
        error), and gravity and Coriolis compensation. Carried objects (welds)
        count in both the inertia and the gravity. The net torque (motor minus
        damping) is limited to the joint's effort.
        """
        model, data, omega = self.model, self.data, self._omega
        load_inertia, load_torque = self._payload()
        inertia = self._apparent_inertia() + load_inertia
        damping = 2.0 * inertia * omega
        model.dof_damping[self._vadr] = damping
        error = qpos_ref[self._qadr] - data.qpos[self._qadr]
        bias = data.qfrc_bias[self._vadr] - load_torque  # hold the payload up too
        for i, group in enumerate(self._followers):
            for dof, multiplier in group:  # followers' loads, seen by the leader
                bias[i] += multiplier * data.qfrc_bias[dof]
        command = (
            inertia * omega**2 * error
            + damping * qvel_ref[self._vadr]
            + bias
            + self.feedforward
        )
        passive = damping * data.qvel[self._vadr]  # what the damping takes away
        data.ctrl[:] = np.clip(command, passive - self._effort, passive + self._effort)

    # -- contract --------------------------------------------------------

    def start(self, command: ExecutionCommand) -> ExecutionStatus:
        payload = command.payload
        path = self.get_path(payload) if self.get_path is not None else payload
        if path is None or not hasattr(path, "eval"):
            return ExecutionStatus.FAILURE
        self._path, self.cancelled = path, False
        self._t0, self._length = _t0(path), float(path.length())
        start = self._reference(0.0)
        if start is None:
            return ExecutionStatus.FAILURE
        if not self._initialized:
            self.reset(self._planner_q)
        self._qpos_start = start
        self._start_drift = self._joint_error(start)
        samples = [
            self._reference(self._length * k / (CARRIER_SAMPLES - 1))
            for k in range(CARRIER_SAMPLES)
        ]
        if any(s is None for s in samples):
            return ExecutionStatus.FAILURE
        self._grasp_error = 0.0
        self._carried = self._carriers(samples)
        #: What carries each object in the current command (body names).
        self.carriers = {
            name: (
                "world" if c == 0 else None if c is None else self.model.body(c).name
            )
            for name, c in self._carried.items()
        }
        self._failure = self._attach(self._carried, start)
        self._skill = None
        self.feedforward[:] = 0.0
        if isinstance(payload, SkillCommand):
            factory = self.skills.get(payload.name)
            if factory is not None:
                self._duration = self._length
                self._scale = 1.0
                self._t = 0.0
                self._tracking = 0.0
                self._started = self.clock()
                self._skill = factory(self, payload)
                return ExecutionStatus.RUNNING
        self._times, self._grid = self._retime()
        self._duration = float(self._times[-1])
        # A (near) zero-length path still takes a moment rest to rest; its
        # ratio says nothing.
        self._scale = self._duration / self._length if self._length > 0.05 else 1.0
        self._t = 0.0
        self._tracking = 0.0
        self._started = self.clock()
        return ExecutionStatus.RUNNING

    def _path_time(self, t: float) -> float:
        """The path parameter reached at simulated time ``t`` (retimed)."""
        return float(np.interp(t, self._times, self._grid))

    def _reference(self, t: float) -> np.ndarray | None:
        q, ok = self._path.eval(self._t0 + min(max(t, 0.0), self._length))
        if not ok:
            return None
        self._planner_q = q
        return self.to_qpos(q)

    def _joint_error(self, qpos_ref: np.ndarray) -> float:
        adr = self._qadr
        return float(np.max(np.abs(self.data.qpos[adr] - qpos_ref[adr]), initial=0.0))

    def poll(self) -> tuple[ExecutionStatus, Feedback | None]:
        if self._path is None:
            return ExecutionStatus.FAILURE, Feedback(message="nothing started")
        if self.cancelled:
            return ExecutionStatus.FAILURE, Feedback(message="cancelled")
        if self._failure:
            return ExecutionStatus.FAILURE, Feedback(
                message=self._failure, metrics=self._metrics()
            )
        if self._skill is not None:
            return self._skill.poll()
        model = self.model
        moving = self._duration
        end = moving + self.settle
        target = self.sim_target(end)
        dt = model.opt.timestep
        while self._t < target:
            ref = self._reference(self._path_time(self._t))
            ahead = self._reference(self._path_time(self._t + dt))
            if ref is None or ahead is None:
                return ExecutionStatus.FAILURE, Feedback(
                    message=f"path evaluation failed at t={self._t:.3f}"
                )
            qvel_ref = self.velocity(ref, ahead)
            if self._t >= moving:
                qvel_ref[:] = 0.0
            error = self.control_step(ref, qvel_ref, ahead)
            if error:
                return ExecutionStatus.FAILURE, Feedback(
                    message=error, metrics=self._metrics()
                )
        metrics = self._metrics()
        if self._t >= end:
            final = self._reference(self._length)
            metrics.update(self._final_metrics(final))
            carried = ", ".join(
                f"{name.split('/')[0]} by {carrier}"
                for name, carrier in self.carriers.items()
                if carrier not in ("world", None)
            )
            return ExecutionStatus.SUCCESS, Feedback(
                progress=1.0,
                message=f"carried: {carried}" if carried else "",
                metrics=metrics,
            )
        progress = min(1.0, self._t / end) if end > 0 else 1.0
        return ExecutionStatus.RUNNING, Feedback(progress=progress, metrics=metrics)

    # -- the control loop, shared with skill controllers ------------------

    def sim_target(self, end: float) -> float:
        """How far (simulated seconds into the command) this poll may run."""
        if math.isinf(self.speed):
            return end
        return min(end, (self.clock() - self._started) * self.speed)

    def velocity(self, qpos: np.ndarray, ahead: np.ndarray) -> np.ndarray:
        """The joint velocity going from ``qpos`` to ``ahead`` in one step."""
        qvel = np.zeros(self.model.nv)
        self.mujoco.mj_differentiatePos(
            self.model, qvel, self.model.opt.timestep, qpos, ahead
        )
        return qvel

    def control_step(
        self, qpos_ref: np.ndarray, qvel_ref: np.ndarray, ahead: np.ndarray
    ) -> str:
        """Track the reference for one physics step; "" or a failure message."""
        data = self.data
        self._set_ctrl(qpos_ref, qvel_ref)
        for name, carrier in self._carried.items():
            if carrier is None:
                self._drive(name, ahead, qvel_ref)
        self.mujoco.mj_step(self.model, data)
        self._t += self.model.opt.timestep
        if not np.isfinite(data.qpos).all():
            return f"simulation diverged at t={self._t:.3f}"
        self._tracking = max(self._tracking, self._joint_error(qpos_ref))
        if (
            self.max_tracking_error is not None
            and self._tracking > self.max_tracking_error
        ):
            return (
                f"tracking error {self._tracking:.3f} rad exceeds "
                f"{self.max_tracking_error:g} at t={self._t:.2f} s"
            )
        return ""

    def planned_qpos(self, s: float) -> np.ndarray | None:
        """The planned configuration (MuJoCo ``qpos``) at path parameter ``s``."""
        return self._reference(s)

    def body_position(self, qpos: np.ndarray, body: int) -> np.ndarray:
        """Where ``body`` is at configuration ``qpos`` (kinematics only)."""
        self._scratch.qpos[:] = qpos
        self.mujoco.mj_kinematics(self.model, self._scratch)
        return self._scratch.xpos[body].copy()

    def wrench_torques(self, body: int, wrench: np.ndarray) -> np.ndarray:
        """Driven-joint torques that exert ``wrench`` (force, torque; world
        frame) at ``body``'s centre of mass, through what carries it:
        ``J^T w`` (the counterpart of ``data.xfrc_applied[body]``)."""
        mujoco, model, data = self.mujoco, self.model, self.data
        link = body
        for name, obj in self._objects.items():
            eq = self._weld(name)
            if obj == body and data.eq_active[eq]:
                link = int(model.eq_obj1id[eq])  # a welded object: its carrier
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        # At the centre of mass: where MuJoCo applies xfrc_applied.
        mujoco.mj_jac(model, data, jacp, jacr, data.xipos[body], link)
        torques = jacp.T @ wrench[:3] + jacr.T @ wrench[3:]
        return torques[self._vadr]

    # -- drift -------------------------------------------------------------

    def start_error(self, command: ExecutionCommand) -> float | None:
        """How far the simulation is from where ``command``'s path starts:
        the largest driven-joint error [rad], or ``None`` before the first
        command (the simulation starts wherever that command starts)."""
        if not self._initialized:
            return None
        path = command.payload
        if self.get_path is not None and not hasattr(path, "eval"):
            path = self.get_path(path)
        if path is None or not hasattr(path, "eval"):
            return None
        q, ok = path.eval(_t0(path))
        if not ok:
            return None
        return self._joint_error(self.to_qpos(q))

    def observed_config(self, like: Any) -> np.ndarray | None:
        """The simulation's state as a planner configuration (``like`` fills
        what MuJoCo doesn't map), or ``None`` without ``from_qpos``."""
        if self.from_qpos is None:
            return None
        return self.from_qpos(self.data.qpos.copy(), like)

    def disturb(self, joint: str, delta: float) -> None:
        """Move one joint by ``delta`` right now, as if the robot were bumped
        (the drift the executor checks for before running a cached plan)."""
        j = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, joint)
        if j < 0:
            raise KeyError(joint)
        self.data.qpos[self.model.jnt_qposadr[j]] += delta
        self.data.qvel[:] = 0.0
        self.mujoco.mj_forward(self.model, self.data)

    def carried_by_robot(self) -> dict[str, int]:
        """Objects a robot link carries in this command: name -> body id."""
        return {
            name: self._objects[name]
            for name, carrier in self._carried.items()
            if carrier not in (0, None)
        }

    def _drive(self, name: str, qpos_ref: np.ndarray, qvel_ref: np.ndarray) -> None:
        model = self.model
        body = self._objects[name]
        j = model.body_jntadr[body]
        adr, dof = model.jnt_qposadr[j], model.jnt_dofadr[j]
        self.data.qpos[adr : adr + 7] = qpos_ref[adr : adr + 7]
        self.data.qvel[dof : dof + 6] = qvel_ref[dof : dof + 6]

    def _metrics(self) -> dict[str, float]:
        metrics = {
            "tracking_error": round(self._tracking, 5),
            "start_drift": round(self._start_drift, 5),
            "sim_seconds": round(self._t, 3),
            "time_scale": round(self._scale, 3),
        }
        if self._grasp_error:
            metrics["grasp_error"] = round(self._grasp_error, 5)
        kinematic = sum(1 for c in self._carried.values() if c is None)
        if kinematic:
            metrics["kinematic_objects"] = kinematic
        return metrics

    def _final_metrics(self, final: np.ndarray) -> dict[str, float]:
        scratch = self._scratch
        scratch.qpos[:] = final
        self.mujoco.mj_kinematics(self.model, scratch)
        object_drift = max(
            (
                float(np.linalg.norm(self.data.xpos[b] - scratch.xpos[b]))
                for b in self._objects.values()
            ),
            default=0.0,
        )
        return {
            "drift": round(self._joint_error(final), 5),
            "object_drift": round(object_drift, 5),
        }

    def cancel(self) -> None:
        self.cancelled = True
