"""Pick a flat object by its overhang: push it to the table's edge, then
pinch it from the side (#188).

A plate's rim stands a few millimetres above the table: no top-down pinch
reaches it without the hand in the table, and HPP plans none of its handles
(R3). :func:`edge_pinch` reads a way from the exported scene: the support
region the object rests on, the region's edge nearest the robot, and a
point by that edge, clear of the other objects, where the object overhangs
the edge. :class:`EdgePinchController` pushes it there in short strokes,
each re-seated outside the rim where the object now is (on one long push
the fingertips ride over the rim), then pinches the overhang from the side,
the fingers closing vertically, and lifts.

The strokes and the pinch are scripted on the observations, not planned:
they are checked only against the other objects' collision geometry, in
plan view at the heights the object and the hand pass at. The constants
were measured on Ideal/case3's plate (seeds 0-9 all pick, about 540 env
steps), with the robot at the table's -x side; the pinch's orientation keeps
joint 7 off its limit there.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .robocerebra_plan import PlanningError, _hull
from .robocerebra_tracker import OscTracker, Stage, obs_pose

#: Height of the grip site above the closed fingertips of a hand pointing
#: down: a push this high above the table drags the tips just over it [m].
TIP = 0.022
#: Thickest object pushed (bottom to top of its collision geometry) [m].
FLAT = 0.03
#: Gap from the rim where a stroke comes down, and the object's travel a
#: stroke [m].
PUSH_OUT = 0.03
STROKE = 0.06
#: Height above the push that a stroke re-seats from [m].
RESEAT = 0.054
#: The object's centre from the edge after the push: it overhangs by
#: ``radius - INSET`` [m].
INSET = 0.025
#: The pinch: how far inside the rim's outer edge, and below its top [m].
PINCH_IN = 0.009
PINCH_BELOW_TOP = 0.0037
#: Closing axis tilt toward the object [rad].
PINCH_TILT = np.radians(20.0)
#: The hand's approach along the edge: start, pre-grasp [m]; the start's
#: height above the table [m].
APPROACH = (0.15, 0.11)
APPROACH_HEIGHT = 0.096
#: Steps the closed grip settles before the lift, and holds after it (the
#: pick checker wants the object still over its last 40 steps); lift [m].
SETTLE = 55
HOLD = 45
LIFT = 0.12
#: Lateral offsets along the edge tried, nearest first [m].
SIDES = (0.0, 0.07, -0.07, 0.14, -0.14, 0.21, -0.21)
#: Clearance of the pushed object, and of the hand's approach, from the
#: other objects' geometry below them [m]; the hand's height above the
#: table [m].
CLEARANCE = 0.005
HAND = 0.05
HAND_HEIGHT = 0.15
#: Strokes before the push gives up; distance to the goal it stops at [m].
MAX_STROKES = 12
REACHED = 0.008


@dataclass
class EdgePinch:
    """Where to push ``target`` and how to pinch it (see :func:`edge_pinch`).

    ``goal`` is the object's centre after the push (xy), ``normal`` the
    edge's outward normal (xy, towards the robot), ``radius`` and ``top``
    the object's outer radius and its top above its origin, ``table_z`` the
    support's height. ``info`` goes into the segment's record."""

    target: str
    goal: np.ndarray
    normal: np.ndarray
    radius: float
    top: float
    table_z: float
    info: dict[str, Any] = field(default_factory=dict)

    @property
    def tangent(self) -> np.ndarray:
        """Along the edge, the side the hand approaches the pinch from."""
        return np.array([self.normal[1], -self.normal[0]])

    def frame(self) -> np.ndarray:
        """World rotation of the edge frame: x inwards, y along
        :attr:`tangent`, z up (the identity at the table's -x edge)."""
        return np.column_stack(
            [np.r_[-self.normal, 0.0], np.r_[self.tangent, 0.0], [0.0, 0.0, 1.0]]
        )

    def stroke(self, obs: dict[str, Any]) -> list[Stage] | None:
        """The next push stroke from where the object is, or None once it
        is at :attr:`goal`."""
        here = np.asarray(obs[f"{self.target}_pos"][:2], dtype=float)
        gap = self.goal - here
        distance = float(np.linalg.norm(gap))
        if distance <= REACHED:
            return None
        u = gap / distance
        z = self.table_z + TIP
        back = here - u * (self.radius + PUSH_OUT)
        end = here + u * (min(STROKE, distance) - self.radius - 0.004)
        hand_pose = obs_pose(obs)
        down = _pointing_down(hand_pose[:3, :3])
        hand = hand_pose[:3, 3]
        above, at, to = np.r_[back, z + RESEAT], np.r_[back, z], np.r_[end, z]
        return [
            Stage("close", obs_pose(obs)[None], 1.0),
            Stage("over", _line(down, hand, above, 8), 1.0),
            Stage("down", _line(down, above, at, 8), 1.0, step=0.01),
            Stage("push", _line(down, at, to, 15), 1.0, step=0.01),
            Stage("up", _line(down, to, to + [0, 0, RESEAT], 5), 1.0),
        ]

    def pinch(self, obs: dict[str, Any]) -> list[Stage]:
        """Approach along the edge, the fingers closing vertically on the
        overhanging rim, grip and lift."""
        a = PINCH_TILT
        rotation = np.column_stack(
            [[-np.cos(a), 0, np.sin(a)], [np.sin(a), 0, np.cos(a)], [0, 1.0, 0]]
        ) @ np.diag([-1.0, -1.0, 1.0])
        mirror = np.diag([1.0, -1.0, 1.0])  # approach along -y, from +y
        R = self.frame() @ mirror @ rotation @ mirror
        position = np.asarray(obs[f"{self.target}_pos"], dtype=float)
        grasp = np.r_[
            position[:2] + self.normal * (self.radius - PINCH_IN),
            position[2] + self.top - PINCH_BELOW_TOP,
        ]
        side = np.r_[self.tangent, 0.0]
        start = grasp + side * APPROACH[0]
        start[2] = self.table_z + APPROACH_HEIGHT
        pre = grasp + side * APPROACH[1]
        lift = grasp + [0, 0, LIFT]
        here = obs_pose(obs)
        return [
            Stage("open", here[None], -1.0, dwell=10),
            Stage("turn", _line(R, here[:3, 3], start, 10), -1.0),
            Stage("pre", _line(R, start, pre, 10), -1.0, step=0.01),
            Stage("in", _line(R, pre, grasp, 20), -1.0, step=0.01),
            Stage("grip", _pose(R, grasp)[None], 1.0, dwell=SETTLE),
            Stage("lift", _line(R, grasp, lift, 20), 1.0, step=0.01, dwell=HOLD),
        ]


@dataclass
class EdgePinchController:
    """Closed-loop actions for an :class:`EdgePinch`: ``controller(obs) ->
    (7,)``. Strokes until the object is at the goal (a stroke planned
    afresh from each observation), then the pinch. ``done`` once the pinch
    has run, or when the push gave up or the pinch stalled (``stalled``);
    then it holds still, the gripper as it was."""

    way: EdgePinch
    info: dict[str, Any] = field(default_factory=dict)
    max_strokes: int = MAX_STROKES
    phase: str = field(default="push", init=False)
    strokes: int = field(default=0, init=False)
    stalled: bool = field(default=False, init=False)
    tracker: OscTracker | None = field(default=None, init=False)
    _gripper: float = field(default=1.0, init=False)

    @property
    def done(self) -> bool:
        return self.phase == "done"

    def __call__(self, obs: dict[str, Any]) -> np.ndarray:
        position = [round(float(v), 4) for v in obs[f"{self.way.target}_pos"]]
        self.info.setdefault("target_start", position)
        self.info["target_end"] = position
        if self.tracker is None or self.tracker.done:
            self._next(obs)
        if self.done:
            action = np.zeros(7)
            action[3] = 1e-6  # nonzero: re-anchors OSC's orientation goal
            action[6] = self._gripper
            return action
        assert self.tracker is not None
        action = self.tracker(obs)
        self._gripper = float(action[6])
        self.info["stage"] = f"{self.phase} {self.tracker.stage_name}"
        return action

    def _next(self, obs: dict[str, Any]) -> None:
        """Start the next stroke or the pinch, or end."""
        if self.phase == "pinch":
            self.stalled = self.tracker is not None and self.tracker.stalled is not None
            self.info["pinch"] = self.tracker.log if self.tracker else []
            self.phase = "done"
            return
        stages = self.way.stroke(obs)
        if stages is not None and self.strokes >= self.max_strokes:
            self.info["error"] = (
                f"the push did not reach the edge in {self.strokes} strokes"
            )
            self.stalled, self.phase = True, "done"
            return
        if stages is None:
            self.phase = "pinch"
            stages = self.way.pinch(obs)
        else:
            self.strokes += 1
            self.info["strokes"] = self.strokes
        self.tracker = OscTracker(stages)


def edge_pinch(scene: Any, target: str) -> EdgePinch:
    """The way to pick ``target`` by pushing it over the edge of the region
    it rests on that is nearest the robot (``scene`` an exported scene).

    The goal sits :data:`INSET` in from the edge, offset along it by the
    first of :data:`SIDES` whose push (the object's disc swept from where it
    is) and whose pinch approach clear the other objects' collision
    primitives that reach down to the heights they pass at.
    :class:`PlanningError` when ``target`` is not a flat object resting
    upright on a region, or no offset is clear."""
    from long_tamp.grasping import load_urdf_primitives, xyzquat_to_pose

    manifest = scene.manifest
    paths = scene.file_paths()["objects"]
    pose = xyzquat_to_pose(scene.object_config(target))
    info = {"target": target, "grasp": "edge pinch"}
    if pose[2, 2] < 0.95:
        raise PlanningError(f"{target} is not upright", info)
    corners = np.vstack(
        [_box_corners(p) for p in load_urdf_primitives(paths[target]["urdf"])]
    )
    radius = float(np.linalg.norm(corners[:, :2], axis=1).max())
    top, bottom = float(corners[:, 2].max()), float(corners[:, 2].min())
    if top - bottom > FLAT:
        raise PlanningError(f"{target} is {top - bottom:.3f} m thick, not flat", info)
    where = pose[:2, 3]
    support = _support(manifest.get("regions", []), where, pose[2, 3] + bottom)
    if support is None:
        raise PlanningError(f"{target} rests on no region", info)
    polygon = np.asarray(support["points"], dtype=float)
    base = np.asarray(manifest["robot"]["pose"][:2], dtype=float)
    a, b = min(
        zip(polygon[:, :2], np.roll(polygon[:, :2], -1, axis=0)),
        key=lambda edge: _point_segment(base, *edge),
    )
    t = (b - a) / np.linalg.norm(b - a)
    normal = np.array([t[1], -t[0]])
    if normal @ (a - polygon[:, :2].mean(axis=0)) < 0:
        normal = -normal
    table_z = float(polygon[:, 2].mean())
    pushed = _obstacles(scene, target, table_z + top - bottom + 0.01)
    passed = _obstacles(scene, target, table_z + HAND_HEIGHT)
    along = float((where - a) @ t)
    length = float(np.linalg.norm(b - a))
    for side in SIDES:
        s = along + side
        if not radius <= s <= length - radius:
            continue
        goal = a + t * s - normal * INSET
        way = EdgePinch(target, goal, normal, radius, top, table_z, info)
        hand = goal + normal * (radius - PINCH_IN)
        approach = hand + way.tangent * APPROACH[0]
        clear = all(
            _segment_polygon(where, goal, f) > radius + CLEARANCE for f in pushed
        ) and all(_segment_polygon(hand, approach, f) > HAND for f in passed)
        if clear:
            info.update(region=support["region"], goal=goal.round(4).tolist())
            return way
    raise PlanningError(
        f"no clear way to push {target} to the edge of {support['region']}", info
    )


def _support(regions: list[dict[str, Any]], xy: np.ndarray, z: float) -> Any:
    """The region whose polygon holds ``xy``, its height nearest ``z``."""
    under = [
        r
        for r in regions
        if r.get("points") and _inside(xy, np.asarray(r["points"])[:, :2])
    ]
    return min(under, key=lambda r: abs(r["points"][0][2] - z), default=None)


def _box_corners(primitive: Any) -> np.ndarray:
    half = primitive.half_extents()
    signs = np.array(np.meshgrid(*[[-1, 1]] * 3)).T.reshape(-1, 3)
    return (primitive.pose[:3, :3] @ (signs * half).T).T + primitive.pose[:3, 3]


def _obstacles(scene: Any, target: str, below: float) -> list[np.ndarray]:
    """Plan-view hulls of the other objects' collision primitives (their
    bounding boxes) that reach lower than ``below``."""
    from long_tamp.grasping import load_urdf_primitives, xyzquat_to_pose

    hulls = []
    for name, paths in scene.file_paths()["objects"].items():
        if name == target:
            continue
        pose = xyzquat_to_pose(scene.object_config(name))
        for primitive in load_urdf_primitives(paths["urdf"]):
            corners = _box_corners(primitive) @ pose[:3, :3].T + pose[:3, 3]
            hull = _hull(corners[:, :2])
            if corners[:, 2].min() < below and len(hull) >= 3:
                hulls.append(hull)
    return hulls


def _inside(p: np.ndarray, polygon: np.ndarray) -> bool:
    """Whether ``p`` is in the convex ``polygon`` (either winding)."""
    turns = [
        (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        for a, b in zip(polygon, np.roll(polygon, -1, axis=0))
    ]
    return all(t >= 0 for t in turns) or all(t <= 0 for t in turns)


def _point_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    d = b - a
    s = 0.0 if not d @ d else float(np.clip((p - a) @ d / (d @ d), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + s * d)))


def _segment_polygon(a: np.ndarray, b: np.ndarray, polygon: np.ndarray) -> float:
    """Distance from the segment ``a b`` to a convex ``polygon``, sampled
    every centimetre along the segment (0 inside it)."""
    count = max(2, int(np.ceil(np.linalg.norm(b - a) / 0.01)) + 1)
    edges = list(zip(polygon, np.roll(polygon, -1, axis=0)))
    return min(
        0.0 if _inside(p, polygon) else min(_point_segment(p, *e) for e in edges)
        for p in np.linspace(a, b, count)
    )


def _pointing_down(R: np.ndarray) -> np.ndarray:
    """The hand pointing straight down nearest ``R`` in yaw."""
    x = np.r_[R[:2, 0], 0.0]
    x = x / np.linalg.norm(x) if np.linalg.norm(x) > 1e-6 else np.array([1.0, 0, 0])
    z = np.array([0.0, 0.0, -1.0])
    return np.column_stack([x, np.cross(z, x), z])


def _pose(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def _line(R: np.ndarray, a: np.ndarray, b: np.ndarray, n: int) -> np.ndarray:
    return np.array([_pose(R, a + (b - a) * t) for t in np.linspace(0, 1, n)])


__all__ = ["EdgePinch", "EdgePinchController", "edge_pinch"]
