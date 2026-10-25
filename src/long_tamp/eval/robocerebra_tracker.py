"""Track a planned TCP path with robosuite ``OSC_POSE`` delta actions (#159).

RoboCerebra's Panda runs robosuite's operational-space controller in delta
mode at 20 Hz: an action ``[dx, dy, dz, rx, ry, rz, g]`` in ``[-1, 1]`` moves
the controller's goal to ``ee_pos + 0.05 * d`` and ``R(0.5 * r) @ ee_ori``
(axis-angle, world frame), ``g`` is the gripper (-1 open, +1 closed). The
goal is set from the *current* pose every step, so the tracker is closed
loop: each step it reads the end-effector pose from the observation and
aims at a carrot a little ahead of it on the path.

OSC keeps its orientation goal through an action whose rotation is exactly
zero (``math.isclose``), so a goal left by an earlier segment would hold.
Every action here carries a nonzero rotation, which re-anchors the goal.

Pure numpy: the planner's TCP frame (HPP's ``panda_hand_tcp``) differs from
the observed end-effector frame by a fixed offset, ``tcp_to_eef``, that the
caller measures once at the segment start.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

POS_SCALE = 0.05  # m per unit action (robosuite OSC_POSE output_max)
ROT_SCALE = 0.5  # rad per unit action
_TINY_ROTATION = 1e-6  # nonzero, so OSC re-anchors its orientation goal


@dataclass
class Stage:
    """Part of a segment's motion: TCP poses to follow with one gripper
    command, then ``dwell`` steps holding the last pose (a gripper closing
    takes about 8 steps to go from open to closed). ``step`` overrides the
    tracker's carrot distance: short where the path must be followed closely
    (an insertion), long where only its end matters; ``tol`` overrides how
    close to the last pose the stage must end."""

    name: str
    tcp: np.ndarray  # (N, 4, 4) world poses of the planner's TCP
    gripper: float  # -1 open, +1 closed
    dwell: int = 0
    step: float | None = None
    tol: float | None = None

    def __post_init__(self) -> None:
        self.tcp = np.asarray(self.tcp, dtype=float).reshape(-1, 4, 4)
        if len(self.tcp) == 0:
            raise ValueError(f"stage {self.name!r} has no poses")


def obs_pose(obs: dict[str, Any]) -> np.ndarray:
    """The end-effector pose in a step's observation (``eef_quat`` is xyzw)."""
    x, y, z, w = (float(v) for v in obs["robot0_eef_quat"])
    T = np.eye(4)
    T[:3, :3] = _quat_matrix(x, y, z, w)
    T[:3, 3] = np.asarray(obs["robot0_eef_pos"], dtype=float)
    return T


def rotation_vector(R: np.ndarray) -> np.ndarray:
    """Axis-angle vector of rotation matrix ``R``."""
    cos = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = float(np.arccos(cos))
    if angle < 1e-9:
        return np.zeros(3)
    if np.pi - angle < 1e-6:
        # Near pi: the axis is the dominant column of R + I.
        M = (R + np.eye(3)) / 2.0
        axis = M[:, int(np.argmax(np.diag(M)))]
        return angle * axis / np.linalg.norm(axis)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return angle * axis / (2.0 * np.sin(angle))


def pose_error(target: np.ndarray, current: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """World-frame translation and rotation vector from ``current`` to ``target``."""
    return target[:3, 3] - current[:3, 3], rotation_vector(target[:3, :3] @ current[:3, :3].T)


def resample(poses: np.ndarray, pos_step: float, rot_step: float) -> np.ndarray:
    """Keep a pose whenever it is ``pos_step``/``rot_step`` away from the
    last kept one; the first and last are always kept."""
    kept = [poses[0]]
    for T in poses[1:-1]:
        dp, dr = pose_error(T, kept[-1])
        if np.linalg.norm(dp) >= pos_step or np.linalg.norm(dr) >= rot_step:
            kept.append(T)
    if len(poses) > 1:
        kept.append(poses[-1])
    return np.array(kept)


@dataclass
class OscTracker:
    """Closed-loop OSC_POSE actions along ``stages``.

    Call it with each observation; it returns the next ``(7,)`` action. Past
    the last stage it holds the final pose (``done`` is then true), so a
    segment whose predicate needs the scene to settle keeps its budget.

    A stage's waypoints are its TCP poses mapped to the end-effector frame
    and thinned to a quarter of the carrot distance (the stage's ``step``,
    else ``pos_step``; ``rot_step`` in rotation). The carrot is the first
    waypoint at least that far from the current pose; a stage ends once the
    last waypoint is within ``pos_tol``/``rot_tol`` (or after ``patience``
    steps aiming at it), plus the stage's dwell.

    OSC's pull is proportional to the goal offset, so a small remaining
    error can stall against friction or a grazing contact: while aiming at
    the last waypoint the gain grows by one ``gain`` every ``boost`` steps,
    up to ``max_boost`` times, like an integral term.
    """

    stages: list[Stage]
    tcp_to_eef: np.ndarray = field(default_factory=lambda: np.eye(4))
    pos_step: float = 0.05
    rot_step: float = 0.3
    pos_tol: float = 0.005
    rot_tol: float = 0.05
    patience: int = 15
    gain: float = 1.0
    boost: int = 4
    max_boost: float = 4.0

    def __post_init__(self) -> None:
        if not self.stages:
            raise ValueError("no stages to track")
        self._steps = [s.step or self.pos_step for s in self.stages]
        self._waypoints = [
            resample(s.tcp @ self.tcp_to_eef, step / 4, self.rot_step / 4)
            for s, step in zip(self.stages, self._steps)
        ]
        self.stage = 0
        self._index = 0
        self._at_end = 0  # steps aiming at the stage's last waypoint
        self._dwelt = 0
        self.log: list[dict[str, Any]] = []

    @property
    def done(self) -> bool:
        return self.stage >= len(self.stages)

    @property
    def stage_name(self) -> str:
        return self.stages[min(self.stage, len(self.stages) - 1)].name

    def __call__(self, obs: dict[str, Any]) -> np.ndarray:
        current = obs_pose(obs)
        if self.done:
            return self._action(self._waypoints[-1][-1], current, self.stages[-1].gripper)
        stage = self.stages[self.stage]
        waypoints = self._waypoints[self.stage]
        last = len(waypoints) - 1
        while self._index < last and self._within(waypoints[self._index], current, self._steps[self.stage]):
            self._index += 1
        target = waypoints[self._index]
        if self._index == last:
            self._at_end += 1
            if self._converged(target, current, stage.tol) or self._at_end > self.patience:
                self._dwelt += 1
                if self._dwelt > stage.dwell:
                    self.log.append({"stage": stage.name, "end_error": self._errors(target, current)})
                    self._next_stage()
                    return self._action(target, current, stage.gripper)
            return self._action(target, current, stage.gripper, self._boost())
        return self._action(target, current, stage.gripper)

    def _boost(self) -> float:
        return min(1.0 + self._at_end / self.boost, self.max_boost)

    def _next_stage(self) -> None:
        self.stage += 1
        self._index = 0
        self._at_end = 0
        self._dwelt = 0

    def _within(self, target: np.ndarray, current: np.ndarray, step: float) -> bool:
        dp, dr = pose_error(target, current)
        return np.linalg.norm(dp) < step and np.linalg.norm(dr) < self.rot_step

    def _converged(self, target: np.ndarray, current: np.ndarray, tol: float | None) -> bool:
        dp, dr = pose_error(target, current)
        return np.linalg.norm(dp) < (tol or self.pos_tol) and np.linalg.norm(dr) < self.rot_tol

    @staticmethod
    def _errors(target: np.ndarray, current: np.ndarray) -> list[float]:
        dp, dr = pose_error(target, current)
        return [round(float(np.linalg.norm(dp)), 4), round(float(np.linalg.norm(dr)), 4)]

    def _action(
        self, target: np.ndarray, current: np.ndarray, gripper: float, boost: float = 1.0
    ) -> np.ndarray:
        dp, dr = pose_error(target, current)
        gain = self.gain * boost
        action = np.empty(7)
        action[:3] = np.clip(gain * dp / POS_SCALE, -1.0, 1.0)
        action[3:6] = np.clip(gain * dr / ROT_SCALE, -1.0, 1.0)
        if not np.any(action[3:6]):
            action[3] = _TINY_ROTATION
        action[6] = gripper
        return action


def _quat_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


__all__ = ["OscTracker", "Stage", "obs_pose", "pose_error", "resample", "rotation_vector"]
