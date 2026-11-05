"""Open, close and turn RoboCerebra's articulated fixtures (#161).

LIBERO judges ``open``/``close`` and ``turnon``/``turnoff`` by a fixture's
joint value against its class's ranges (``load``'s ``articulations``): a
cabinet drawer is open once its slide is past the open range's bound, the
stove is on once its knob has turned into its on range.
:func:`articulated_motion` reads the joint from the exported scene (its
value, its axis and its body's pose; ``manifest["articulated"]``) and the
goal's target value (:func:`joint_target`), and puts the hand on the
fixture's handle: a pose fixed in the moving body's frame, per fixture class
(:data:`HANDLES`). :class:`ArticulationController` approaches the handle
along the hand's axis, grips it, moves the hand by the joint's own rigid
motion (a slide translates it along the axis, a hinge turns it about the
axis through the anchor), releases and backs out.

The motion is scripted on the observations, not planned: nothing checks it
against collisions or the arm's reach. The handles were measured on the
demos' grasps (the hand's pose in the moving body's frame as the gripper
closes) and on the fixtures' handle geometry, then checked in the Ideal
cases: Ideal/case6 (short cabinet), case7 (stove), case9 (microwave) and
Memory_Exploration/case1 (wooden cabinet, open and close).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from long_tamp.grasping import xyzquat_to_pose

from .robocerebra_plan import PlanningError
from .robocerebra_tracker import OscTracker, Stage, obs_pose, rotation_vector

#: The goal predicates moving a joint, and which of the fixture's ranges is
#: their goal and which their opposite.
ARTICULATED = {
    "open": ("open", "close"),
    "close": ("close", "open"),
    "turnon": ("turnon", "turnoff"),
    "turnoff": ("turnoff", "turnon"),
}
#: How far an ``open``/``turnon`` goes into its range: half its width, at
#: most this [m for a slide, rad for a hinge]. The stove's knob slips in
#: the fingers, turning about half as far as the hand: aimed 0.6 rad into
#: its range, it passes the bound.
INTO_RANGE = {"slide": 0.04, "hinge": 0.6}
#: How far a ``close``/``turnoff`` pushes past its joint's limit: the
#: fixture stops the joint, the hand keeps pressing it there (a push aimed
#: at the limit fades as OSC's pull shrinks, short of it).
PAST_LIMIT = {"slide": 0.02, "hinge": 0.1}
#: The carrot while the joint moves [m]: a drawer's slide is damped
#: (50 N s/m), a faster pull slips out of the fingers.
MOVE_STEP = {"slide": 0.01, "hinge": 0.03}
#: Poses along the joint's motion.
MOVE_POSES = 30
#: Back from the handle along the hand's axis: the approach's start and the
#: retreat's end [m]; how near the approach's start the hand must come
#: before the reach (only its end is precise) [m].
BACK_OFF = 0.08
APPROACH_TOL = 0.02
#: Steps the gripper closes on the handle, and opens after the move.
GRIP, RELEASE = 10, 8


def _handle(
    x: tuple[float, ...], y: tuple[float, ...], position: tuple[float, ...]
) -> np.ndarray:
    """The hand's pose from its x and y axes and its position."""
    T = np.eye(4)
    T[:3, 0], T[:3, 1] = x, y
    T[:3, 2] = np.cross(T[:3, 0], T[:3, 1])
    T[:3, 3] = position
    return T


#: The hand's pose (the observed end effector, robosuite's grip site) on a
#: fixture's handle, in the moving body's frame: per fixture class (``load``'s
#: ``articulations[...]["class"]``), per joint by its name's suffix
#: (``"top_level"``), the ``""`` entry for any joint. The parallel fingers
#: grip it as well turned half a turn about the hand's axis
#: (:meth:`ArticulatedMotion.grip_stages` takes the nearer).
HANDLES = {
    # The pull tab, its centre 7 mm off the drawer's front.
    "ShortCabinet": {"": _handle((0, 0, 1), (0, 1, 0), (0.111, 0.002, 0.0))},
    # The knob, pinched from above.
    "FlatStove": {"": _handle((0, -1, 0), (-1, 0, 0), (-0.009, -0.0129, 0.0523))},
    # The door's handle, pinched from above.
    "Microwave": {"": _handle((0, 1, 0), (1, 0, 0), (0.2426, -0.0571, 0.0451))},
}
# The wooden and white cabinets are one model: the three drawers' bodies
# share their frame, each handle bar at its own height (x -0.0028,
# y -0.1016), gripped 1.2 cm behind its centre. The bottom bar is 4 cm off
# the table, too low for the hand upright: its entry is the demo's flat-hand
# pose, which does not hold the bar yet (the bottom drawer does not open).
HANDLES["WoodenCabinet"] = HANDLES["WhiteCabinet"] = {
    "top_level": _handle((-1, 0, 0), (0, 0, 1), (-0.0028, -0.090, 0.1839)),
    "middle_level": _handle((-1, 0, 0), (0, 0, 1), (-0.0028, -0.090, 0.1104)),
    "bottom_level": _handle((0, 0, -1), (-1, 0, 0), (-0.0228, -0.1126, 0.0397)),
}
#: The hand turned half a turn about its axis.
HALF_TURN = np.diag([-1.0, -1.0, 1.0, 1.0])

#: The Panda's last wrist joint limit (2.8973 rad), less a margin.
WRIST_LIMIT = 2.75


def handle_pose(fixture_class: str, joint: str) -> np.ndarray | None:
    """The hand's pose on ``joint``'s handle in its body's frame
    (:data:`HANDLES`), or ``None`` without one."""
    poses = HANDLES.get(fixture_class, {})
    pose = next(
        (T for suffix, T in poses.items() if suffix and joint.endswith(suffix)),
        poses.get(""),
    )
    return None if pose is None else pose.copy()


def joint_target(predicate: str, ranges: dict[str, list[float]], kind: str) -> float:
    """The joint value a ``predicate`` goal moves to, from the fixture's
    ``ranges``: an ``open``/``turnon`` goes into its range by half its
    width, at most :data:`INTO_RANGE`, from the bound nearest the opposite
    range; a ``close``/``turnoff`` goes to its range's far bound (the joint's
    limit)."""
    goal, other = ARTICULATED[predicate]
    if goal not in ranges or other not in ranges:
        raise ValueError(f"{predicate} needs {goal} and {other} ranges: {ranges}")
    low, high = sorted(ranges[goal])
    above = low >= max(ranges[other])
    if predicate in ("open", "turnon"):
        into = min(0.5 * (high - low), INTO_RANGE[kind])
        return low + into if above else high - into
    return high if above else low


def reached(predicate: str, ranges: dict[str, list[float]], value: float) -> bool:
    """Whether ``value`` passes the checker for ``predicate`` (LIBERO's
    ``is_open``/``is_close`` and their stove counterparts): past the goal
    range's bound nearest the opposite range."""
    goal, other = ARTICULATED[predicate]
    low, high = sorted(ranges[goal])
    return value > low if low >= max(ranges[other]) else value < high


@dataclass
class ArticulatedMotion:
    """One joint to move: ``joint`` of kind ``kind`` (``"slide"`` or
    ``"hinge"``) from ``value`` to ``target``, the hand on ``handle`` (its
    pose in the moving body's frame). ``body_pose`` is the body's world
    pose, ``axis`` the joint's world axis and ``anchor`` a world point on
    it. ``info`` goes into the segment's record."""

    predicate: str
    joint: str
    kind: str
    value: float
    target: float
    body_pose: np.ndarray
    axis: np.ndarray
    anchor: np.ndarray
    handle: np.ndarray
    info: dict[str, Any] = field(default_factory=dict)

    @property
    def travel(self) -> float:
        """The joint's motion commanded: to the target, and past it by
        :data:`PAST_LIMIT` for a ``close``/``turnoff``."""
        delta = self.target - self.value
        if self.predicate in ("close", "turnoff"):
            delta += np.sign(delta) * PAST_LIMIT[self.kind]
        return float(delta)

    def motion(self, delta: float) -> np.ndarray:
        """The world transform moving the body by ``delta`` along the joint."""
        M = np.eye(4)
        if self.kind == "slide":
            M[:3, 3] = delta * self.axis
            return M
        k = self.axis
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        R = np.eye(3) + np.sin(delta) * K + (1 - np.cos(delta)) * K @ K
        M[:3, :3] = R
        M[:3, 3] = self.anchor - R @ self.anchor
        return M

    def grasp(self) -> np.ndarray:
        """The hand's world pose on the handle."""
        return self.body_pose @ self.handle

    def grip_stages(self, obs: dict[str, Any]) -> list[Stage]:
        """From the hand where it is: to the approach start, along the hand's
        axis onto the handle, close. Of the handle's pose and its half turn
        (:data:`HALF_TURN`), the one nearer the hand's orientation is kept,
        unless it would turn the last wrist joint past its limit: how far
        the wrist turns depends on how the fixture faces the robot."""
        hand = obs_pose(obs)
        turned = self.handle @ HALF_TURN
        if _wrist_cost(self.body_pose @ turned, hand, obs) < _wrist_cost(
            self.grasp(), hand, obs
        ):
            self.handle = turned
        self.info["handle"] = np.round(self.handle[:3], 4).tolist()
        grasp = self.grasp()
        start = _back(grasp, BACK_OFF)
        return [
            Stage("approach", _line(hand, start), -1.0, tol=APPROACH_TOL),
            Stage("reach", _line(start, grasp), -1.0, tol=0.004),
            Stage("grip", grasp[None], 1.0, dwell=GRIP, settle=True),
        ]

    def move_stage(self) -> Stage:
        """The hand carried by the joint's motion, from the grasp."""
        grasp = self.grasp()
        poses = [
            self.motion(s * self.travel) @ grasp
            for s in np.linspace(0.0, 1.0, MOVE_POSES)
        ]
        return Stage("move", np.array(poses), 1.0, step=MOVE_STEP[self.kind])

    def let_go(self, hand: np.ndarray) -> list[Stage]:
        """Open where the hand is, then back out along its axis."""
        return [
            Stage("release", hand[None], -1.0, dwell=RELEASE),
            Stage("retreat", _line(hand, _back(hand, BACK_OFF)), -1.0),
        ]


@dataclass
class ArticulationController:
    """Closed-loop actions for an :class:`ArticulatedMotion`:
    ``controller(obs) -> (7,)``. Grips the handle and moves it, then lets go
    and backs out. A move that stalls (a pushed joint at its limit) lets go
    where the hand stopped. ``done`` once it has backed out, or when the
    grip stalled (``stalled``); then it holds still, the gripper as it was."""

    way: ArticulatedMotion
    info: dict[str, Any] = field(default_factory=dict)
    phase: str = field(default="grip", init=False)
    stalled: bool = field(default=False, init=False)
    tracker: OscTracker | None = field(default=None, init=False)
    _gripper: float = field(default=-1.0, init=False)

    @property
    def done(self) -> bool:
        return self.phase == "done"

    def __call__(self, obs: dict[str, Any]) -> np.ndarray:
        if self.tracker is None:
            self.tracker = OscTracker(
                [*self.way.grip_stages(obs), self.way.move_stage()]
            )
        elif self.tracker.done and self.phase != "done":
            self._next(obs)
        if self.done:
            action = np.zeros(7)
            action[3] = 1e-6  # nonzero: re-anchors OSC's orientation goal
            action[6] = self._gripper
            return action
        action = self.tracker(obs)
        self._gripper = float(action[6])
        self.info["stage"] = self.tracker.stage_name
        return action

    def _next(self, obs: dict[str, Any]) -> None:
        """After the grip and move, let go; after letting go, end."""
        assert self.tracker is not None
        self.info.setdefault("stages", []).extend(self.tracker.log)
        stalled = self.tracker.stalled
        if self.phase == "grip" and stalled in (None, "move"):
            self.info["move_stalled"] = stalled == "move"
            self.phase = "let go"
            self.tracker = OscTracker(self.way.let_go(obs_pose(obs)))
            return
        self.stalled = stalled is not None
        if self.stalled:
            self.info["error"] = f"tracking stalled in {stalled}"
        self.phase = "done"


def articulated_motion(
    manifest: dict[str, Any],
    articulations: dict[str, dict[str, Any]],
    predicate: str,
    name: str,
) -> ArticulatedMotion:
    """The way to make ``[predicate, name]`` hold in the exported scene
    (``manifest``; ``articulations`` is ``load``'s). An object's ``open``
    moves its first joint (the checker wants any one open), its ``close``
    the one joint not yet closed; :class:`PlanningError` when the fixture
    class has no handle, or more than one joint would have to move."""
    predicate = predicate.lower()
    if predicate not in ARTICULATED:
        raise ValueError(f"{predicate!r} moves no joint")
    info: dict[str, Any] = {"target": name, "predicate": predicate}
    entry = articulations.get(name)
    if entry is None:
        raise PlanningError(f"{name} has no articulation", info)
    info["fixture"], info["class"] = entry["parent"], entry["class"]
    if entry["class"] not in HANDLES:
        raise PlanningError(f"no handle for a {entry['class']}", info)
    joints = {row["joint"]: row for row in manifest.get("articulated", [])}
    missing = [j for j in entry["joints"] if j not in joints]
    if missing:
        raise PlanningError(f"{missing} are not in the exported scene", info)
    ranges = entry["ranges"]
    todo = [
        j for j in entry["joints"] if not reached(predicate, ranges, joints[j]["value"])
    ]
    if not todo:
        todo = entry["joints"][:1]  # holds already: move it all the same
    if predicate in ("open", "turnon"):
        todo = todo[:1]
    if len(todo) > 1:
        raise PlanningError(f"{predicate} of {name} moves {len(todo)} joints", info)
    row = joints[todo[0]]
    if row["type"] not in MOVE_STEP:
        raise PlanningError(f"{row['joint']} is a {row['type']} joint", info)
    grasp = handle_pose(entry["class"], row["joint"])
    if grasp is None:
        raise PlanningError(f"no handle for {row['joint']}", info)
    body = xyzquat_to_pose(row["body_pose"])
    axis = np.asarray(row["axis_world"], dtype=float)
    target = joint_target(predicate, ranges, row["type"])
    info.update(joint=row["joint"], value=round(row["value"], 4), goal=round(target, 4))
    return ArticulatedMotion(
        predicate,
        row["joint"],
        row["type"],
        float(row["value"]),
        target,
        body,
        axis / np.linalg.norm(axis),
        body[:3, :3] @ np.asarray(row["anchor"], dtype=float) + body[:3, 3],
        grasp,
        info,
    )


def _wrist_cost(target: np.ndarray, hand: np.ndarray, obs: dict[str, Any]) -> float:
    """How far the hand turns to ``target``; infinite when the turn about its
    axis would carry the last wrist joint past :data:`WRIST_LIMIT` (the hand
    axis is that joint's axis)."""
    turn = rotation_vector(hand[:3, :3].T @ target[:3, :3])
    q = obs.get("robot0_joint_pos")
    if q is not None and abs(float(q[-1]) + float(turn[2])) > WRIST_LIMIT:
        return float("inf")
    return float(np.linalg.norm(turn))


def _back(T: np.ndarray, distance: float) -> np.ndarray:
    """``T`` moved back along its own z axis (the hand's approach)."""
    out = T.copy()
    out[:3, 3] -= distance * T[:3, 2]
    return out


def _line(a: np.ndarray, b: np.ndarray, n: int = 10) -> np.ndarray:
    """Poses from ``a``'s position to ``b``'s, in ``b``'s orientation."""
    out = np.repeat(b[None], n, axis=0)
    out[:, :3, 3] = np.linspace(a[:3, 3], b[:3, 3], n)
    return out


__all__ = [
    "ARTICULATED",
    "HANDLES",
    "ArticulatedMotion",
    "ArticulationController",
    "articulated_motion",
    "handle_pose",
    "joint_target",
    "reached",
]
