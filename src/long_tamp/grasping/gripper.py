"""Gripper models: finger kinematics and a coarse collision hull.

The planners in ``long_tamp.planning`` treat a grasp as a rigid TCP
constraint (gripper frame == handle frame): they never move the fingers, so
the finger joints stay frozen at their open value in every planned path. The
missing piece -- *how far the fingers close* on a given object -- lives here.

A :class:`ParallelGripperModel` knows:

- the finger joints and how each follows the driving value (the 2F-85's
  six-joint linkage is one driving value with mimic multipliers; a Panda's
  two prismatic fingers are each half the opening);
- the stroke table: pad-to-pad gap and pad depth along the approach axis
  for a range of driving values, so :meth:`~ParallelGripperModel.q_for_width`
  turns an object width into joint values;
- a few boxes (palm, two fingers) used to reject grasps that would collide.
  They are a *pre-filter*: HPP's mesh collision check on the planned path
  remains the ground truth.

**Canonical grasp frame.** Every model is described in one frame, whatever
its HPP ``<gripper>`` frame looks like: +X is the approach direction, +Y
the closing axis, +Z = X x Y, origin at the TCP. ``frame_rotation`` maps
that canonical frame to the model's ``<gripper>`` frame, so a planned grasp
can be written as an SRDF ``<handle>`` the gripper frame lands on.

**Other hands.** The grasp planner and the closure table only use the
:class:`GripperModel` protocol (ADR-0008). :class:`RotaryJawGripperModel`
covers a hand with one moving jaw against a fixed one (the SO-101): the
midpoint between its pads moves along the closing axis as it closes. Presets
are in :data:`PRESETS`; :func:`register_gripper` adds one.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol, runtime_checkable

import numpy as np


@runtime_checkable
class GripperModel(Protocol):
    """What the grasp planner and the closure table need from a hand.

    All geometry is in the canonical grasp frame (see the module doc). The
    "driving value" ``q`` is the one number a gripper controller commands;
    ``joint_values`` spreads it over the finger joints.
    """

    name: str
    frame_rotation: np.ndarray
    pad_length: float
    pad_width: float
    default_squeeze: float

    @property
    def q_open(self) -> float: ...

    @property
    def max_width(self) -> float: ...

    @property
    def min_width(self) -> float: ...

    def width_at(self, q: float) -> float:
        """Pad-to-pad gap at driving value ``q``."""
        ...

    def q_for_width(self, width: float) -> float:
        """Driving value whose gap is ``width`` (clamped to the stroke)."""
        ...

    def pad_x_at(self, q: float) -> float:
        """Pads' centre along the approach axis at ``q``."""
        ...

    def pad_x_for_width(self, width: float) -> float: ...

    def center_offset_for_width(self, width: float) -> float:
        """Midpoint between the pads along the closing axis at gap ``width``
        (0 for a symmetric hand)."""
        ...

    def contact_width(self, y_lo: float, y_hi: float) -> float:
        """Gap at which the closing pads first touch an object spanning
        ``[y_lo, y_hi]`` along the closing axis (TCP at 0); wider than
        :attr:`max_width` when it doesn't fit."""
        ...

    def collision_boxes(
        self, width: float, sweep: float = 0.0
    ) -> list[tuple[str, np.ndarray, np.ndarray]]: ...

    def joint_values(self, q: float, prefix: str = "") -> dict[str, float]: ...

    def open_joint_values(self, prefix: str = "") -> dict[str, float]: ...


@dataclass(frozen=True)
class ParallelGripperModel:
    """A parallel-jaw gripper, in the canonical grasp frame (see module doc).

    Attributes:
        name: Model name (e.g. ``"robotiq_2f85"``).
        joints: Finger joint name (without robot prefix) -> multiplier of the
            driving value.
        stroke: ``(q, gap, pad_x)`` rows, sorted by ``q``: driving value,
            pad-to-pad gap (inner faces) and the pads' centre along the
            approach axis (canonical X; negative = behind the TCP).
        pad_length: Pad size along the approach axis.
        pad_width: Pad size along canonical Z.
        finger_thickness: Finger size along the closing axis, outward from
            the pad's inner face.
        finger_width: Finger size along canonical Z.
        palm_front: Canonical X of the palm's front face (negative).
        palm_size: Palm box ``(depth along X, size along Y, size along Z)``.
        frame_rotation: Canonical frame -> HPP ``<gripper>`` frame rotation
            (columns: the gripper frame's axes expressed in canonical axes).
        default_squeeze: Width closed past first contact, so a real gripper
            builds up force (the planned closure never exceeds the stroke).
        inner_front: Per stroke row, the approach coordinate of the
            foremost linkage point *between* the pads (the 2F-85's knuckles
            fold inward there): the object can't reach deeper into the hand.
            Empty: the palm front is the limit.
        inner_width: Size of that linkage along canonical Z.
        linkage_front: Approach coordinate where the fingers' linkage (wider
            than the pads, beside and behind them) ends; the fingers are
            modelled as ``finger_width`` wide in front of it and
            ``inner_width`` wide, ``linkage_thickness`` thick behind it.
            ``None``: one box per finger.
        linkage_thickness: Linkage size along the closing axis, outward
            from the pad's inner face.
        palm_offset: Canonical Y of the palm box's centre.
        finger_tip: Canonical X of the fingertips when they reach past the
            pads' front (the finger boxes then extend to it). ``None``: the
            fingers end at the pads' front.
    """

    name: str
    joints: Mapping[str, float]
    stroke: tuple[tuple[float, float, float], ...]
    pad_length: float
    pad_width: float
    finger_thickness: float
    finger_width: float
    palm_front: float
    palm_size: tuple[float, float, float]
    frame_rotation: np.ndarray = field(default_factory=lambda: np.eye(3))
    default_squeeze: float = 0.0
    inner_front: tuple[float, ...] = ()
    inner_width: float = 0.0
    linkage_front: float | None = None
    linkage_thickness: float = 0.0
    palm_offset: float = 0.0
    finger_tip: float | None = None

    def __post_init__(self) -> None:
        table = np.asarray(self.stroke, dtype=float)
        if table.ndim != 2 or table.shape[1] != 3 or len(table) < 2:
            raise ValueError("stroke needs at least two (q, gap, pad_x) rows")
        if np.any(np.diff(table[:, 0]) <= 0):
            raise ValueError("stroke rows must be sorted by strictly increasing q")
        gaps = table[:, 1]
        if not (np.all(np.diff(gaps) < 0) or np.all(np.diff(gaps) > 0)):
            raise ValueError("stroke gap must be monotonic in q")
        if self.inner_front and len(self.inner_front) != len(table):
            raise ValueError("inner_front needs one value per stroke row")

    # -- stroke ------------------------------------------------------------

    @property
    def _table(self) -> np.ndarray:
        return np.asarray(self.stroke, dtype=float)

    @property
    def q_open(self) -> float:
        """Driving value with the widest gap."""
        t = self._table
        return float(t[np.argmax(t[:, 1]), 0])

    @property
    def max_width(self) -> float:
        return float(self._table[:, 1].max())

    @property
    def min_width(self) -> float:
        return float(self._table[:, 1].min())

    def width_at(self, q: float) -> float:
        t = self._table
        return float(np.interp(q, t[:, 0], t[:, 1]))

    def pad_x_at(self, q: float) -> float:
        t = self._table
        return float(np.interp(q, t[:, 0], t[:, 2]))

    def q_for_width(self, width: float) -> float:
        """Driving value whose pad gap equals ``width`` (clamped to the stroke)."""
        t = self._table
        order = np.argsort(t[:, 1])
        return float(np.interp(width, t[order, 1], t[order, 0]))

    def pad_x_for_width(self, width: float) -> float:
        return self.pad_x_at(self.q_for_width(width))

    def center_offset_for_width(self, width: float) -> float:
        """Midpoint between the pads along the closing axis: 0, the pads
        close symmetrically about the TCP."""
        return 0.0

    def contact_width(self, y_lo: float, y_hi: float) -> float:
        """Both pads close together and stop when the first one touches, so
        the wider side of ``[y_lo, y_hi]`` sets the gap."""
        return 2 * max(y_hi, -y_lo)

    def inner_front_for_width(self, width: float) -> float | None:
        if not self.inner_front:
            return None
        return float(
            np.interp(self.q_for_width(width), self._table[:, 0], self.inner_front)
        )

    def joint_values(self, q: float, prefix: str = "") -> dict[str, float]:
        """Every finger joint's value for driving value ``q``.

        ``prefix`` is the robot name HPP prepends (``"ur10_left"`` gives
        ``"ur10_left/finger_joint"``).
        """
        pre = f"{prefix}/" if prefix else ""
        return {f"{pre}{j}": float(m * q) for j, m in self.joints.items()}

    def open_joint_values(self, prefix: str = "") -> dict[str, float]:
        return self.joint_values(self.q_open, prefix)

    # -- collision hull ----------------------------------------------------

    def collision_boxes(
        self, width: float, sweep: float = 0.0
    ) -> list[tuple[str, np.ndarray, np.ndarray]]:
        """``(name, center, half_extents)`` boxes in the canonical frame.

        ``width`` is the pad gap. ``sweep`` stretches every box backward
        (-X) by that distance: the volume the hand sweeps while approaching
        from ``sweep`` metres away along the approach axis.
        """
        pad_x = self.pad_x_for_width(width)
        front = pad_x + self.pad_length / 2
        if self.finger_tip is not None:
            front = max(front, self.finger_tip)
        back = self.palm_front - sweep
        mid = self.center_offset_for_width(width)
        boxes = []

        def slab(name, sign, x0, x1, thickness, size_z):
            inner = mid + sign * width / 2
            center = np.array([(x0 + x1) / 2, inner + sign * thickness / 2, 0.0])
            half = np.array([(x1 - x0) / 2, thickness / 2, size_z / 2])
            boxes.append((name, center, half))

        split = self.linkage_front
        for side, sign in (("finger_left", -1.0), ("finger_right", 1.0)):
            if split is not None and back < split < front:
                slab(side, sign, split, front, self.finger_thickness, self.finger_width)
                slab(
                    side + "_linkage",
                    sign,
                    back,
                    split,
                    max(self.linkage_thickness, self.finger_thickness),
                    max(self.inner_width, self.finger_width),
                )
            else:
                slab(side, sign, back, front, self.finger_thickness, self.finger_width)
        depth, sy, sz = self.palm_size
        pf = self.palm_front
        center = np.array([pf - depth / 2 - sweep / 2, self.palm_offset, 0.0])
        half = np.array([(depth + sweep) / 2, sy / 2, sz / 2])
        boxes.append(("palm", center, half))
        inner = self.inner_front_for_width(width)
        if inner is not None and inner > back:
            center = np.array([(inner + back) / 2, mid, 0.0])
            half = np.array([(inner - back) / 2, width / 2, self.inner_width / 2])
            boxes.append(("knuckles", center, half))
        return boxes


@dataclass(frozen=True)
class RotaryJawGripperModel(ParallelGripperModel):
    """One jaw swinging against a fixed one (the SO-101's gripper).

    Same description as :class:`ParallelGripperModel`, plus where the pads'
    midpoint is: only one jaw moves, so the midpoint between the pads slides
    along the closing axis as the hand closes, and the pads don't close
    symmetrically about the TCP. The moving jaw's pad is approximated as
    parallel to the fixed one; the stroke's gap is the narrowest one over
    the pads' length, so an object that fits the table fits the hand.

    Attributes:
        center: Per stroke row, canonical Y of the midpoint between the pads'
            inner faces (the fixed pad's face sits at ``center + gap / 2``
            when the fixed jaw is on the +Y side, ``center - gap / 2``
            otherwise).
    """

    center: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        super().__post_init__()
        if len(self.center) != len(self.stroke):
            raise ValueError("center needs one value per stroke row")

    def center_offset_at(self, q: float) -> float:
        return float(np.interp(q, self._table[:, 0], self.center))

    def center_offset_for_width(self, width: float) -> float:
        return self.center_offset_at(self.q_for_width(width))

    def contact_width(self, y_lo: float, y_hi: float) -> float:
        """The pads close until one touches: the narrowest gap whose pads
        still enclose ``[y_lo, y_hi]``. An object reaching past the fixed
        pad never fits: the result is then wider than the stroke."""
        t = self._table
        widths = np.linspace(self.min_width, self.max_width, 2001)
        order = np.argsort(t[:, 1])
        mid = np.interp(widths, t[order, 1], np.asarray(self.center)[order])
        lo, hi = mid - widths / 2, mid + widths / 2
        ok = (lo <= y_lo + 1e-9) & (hi >= y_hi - 1e-9)
        if ok.any():
            return float(widths[np.argmax(ok)])
        short = max(y_hi - hi[-1], lo[-1] - y_lo, 0.0)
        return self.max_width + 2 * short + 1e-6


# ---------------------------------------------------------------------------
# Presets
# ---------------------------------------------------------------------------

#: Robotiq 2F-85 from PickNikRobotics/ros2_robotiq_gripper, as merged into
#: this repo's UR10 (ikea_table_prototype's ``ur10_robotiq*.urdf``; gripper
#: frame on ``gripper_tcp`` 0.15 m in front of ``robotiq_85_base_link``,
#: turned so it approaches along Z and closes along Y like before). Driven by
#: ``robotiq_85_left_knuckle_joint`` (0 open .. 0.8 closed). Stroke measured
#: with :func:`calibrate_from_urdf` on the merged URDF, pad frames
#: (``robotiq_85_{left,right}_finger_pad``) on the fingertips' inner faces,
#: so ``pad_thickness`` is 0. Its HPP gripper frame uses +X approach / Y
#: closing, so ``frame_rotation`` is I.
ROBOTIQ_2F85 = ParallelGripperModel(
    name="robotiq_2f85",
    joints={
        "robotiq_85_left_knuckle_joint": 1.0,
        "robotiq_85_right_knuckle_joint": -1.0,
        "robotiq_85_left_inner_knuckle_joint": 1.0,
        "robotiq_85_right_inner_knuckle_joint": -1.0,
        "robotiq_85_left_finger_tip_joint": -1.0,
        "robotiq_85_right_finger_tip_joint": 1.0,
    },
    stroke=(
        (0.00, 0.08492, -0.01967),
        (0.05, 0.08048, -0.01787),
        (0.10, 0.07588, -0.01618),
        (0.15, 0.07111, -0.01461),
        (0.20, 0.06618, -0.01316),
        (0.25, 0.06112, -0.01183),
        (0.30, 0.05593, -0.01063),
        (0.35, 0.05063, -0.00957),
        (0.40, 0.04523, -0.00863),
        (0.45, 0.03975, -0.00783),
        (0.50, 0.03418, -0.00718),
        (0.55, 0.02857, -0.00666),
        (0.60, 0.02290, -0.00628),
        (0.65, 0.01721, -0.00604),
        (0.70, 0.01150, -0.00595),
        (0.75, 0.00578, -0.00600),
        (0.80, 0.00008, -0.00619),
    ),
    # The fingertips' flat inner faces (collision mesh) span 38 mm along the
    # approach (13 .. 51 mm up the fingertip), 22 mm across.
    pad_length=0.038,
    pad_width=0.022,
    finger_thickness=0.02,
    finger_width=0.027,
    # robotiq_85_base_link's collision mesh spans 0..0.09 m along the
    # approach axis, 0.15 m behind the TCP; 85 x 75 mm across.
    palm_front=-0.06,
    palm_size=(0.09, 0.085, 0.075),
    default_squeeze=0.002,
    # Foremost knuckle/finger collision-mesh point between the pads, per
    # stroke row (calibrate_from_urdf inner_links: the knuckles, inner
    # knuckles and fingers). Past q = 0.65 nothing lies between the pads;
    # the last value is held.
    inner_front=(
        -0.04458,
        -0.04312,
        -0.04179,
        -0.04062,
        -0.03960,
        -0.03876,
        -0.03811,
        -0.03771,
        -0.03759,
        -0.03785,
        -0.03867,
        -0.04046,
        -0.04436,
        -0.05552,
        -0.05552,
        -0.05552,
        -0.05552,
    ),
    inner_width=0.039,
    linkage_front=-0.025,
    linkage_thickness=0.03,
)

#: Robotiq 2F-85 from ros-industrial/robotiq (joint ``finger_joint`` and its
#: mimics), as this repo's UR10 carried it until #71 (ikea_table_prototype's
#: ``ur10_robotiq*.urdf``, gripper frame on ``gripper_tcp`` 0.15 m in front
#: of ``robotiq_arg2f_base_link``). Stroke measured with pinocchio forward
#: kinematics of that URDF, pad inner faces (``*_inner_finger_pad``, 6.35 mm
#: thick) -- see :func:`calibrate_from_urdf` to redo it. Its HPP gripper
#: frame already uses +X approach / Y closing, so ``frame_rotation`` is I.
ROBOTIQ_2F85_ROS_INDUSTRIAL = ParallelGripperModel(
    name="robotiq_2f85_ros_industrial",
    joints={
        "finger_joint": 1.0,
        "left_inner_knuckle_joint": 1.0,
        "left_inner_finger_joint": -1.0,
        "right_outer_knuckle_joint": 1.0,
        "right_inner_knuckle_joint": 1.0,
        "right_inner_finger_joint": -1.0,
    },
    stroke=(
        (0.00, 0.08601, -0.01968),
        (0.05, 0.08162, -0.01785),
        (0.10, 0.07705, -0.01614),
        (0.15, 0.07232, -0.01454),
        (0.20, 0.06743, -0.01306),
        (0.25, 0.06240, -0.01171),
        (0.30, 0.05724, -0.01048),
        (0.35, 0.05196, -0.00939),
        (0.40, 0.04659, -0.00843),
        (0.45, 0.04112, -0.00760),
        (0.50, 0.03558, -0.00691),
        (0.55, 0.02997, -0.00636),
        (0.60, 0.02432, -0.00596),
        (0.65, 0.01863, -0.00569),
        (0.70, 0.01292, -0.00557),
        (0.75, 0.00721, -0.00558),
        (0.80, 0.00151, -0.00575),
    ),
    pad_length=0.0375,
    pad_width=0.022,
    finger_thickness=0.02,
    finger_width=0.027,
    # robotiq_arg2f_base_link's collision mesh spans 0..0.09 m along the
    # approach axis, 0.15 m behind the TCP; 75 x 85 mm across.
    palm_front=-0.06,
    palm_size=(0.09, 0.085, 0.075),
    default_squeeze=0.002,
    # Foremost knuckle/outer-finger collision-mesh point between the pads,
    # per stroke row: calibrate_from_urdf(..., inner_links=(the inner/outer
    # knuckles and outer fingers)). Past q = 0.65 the gap is narrower than
    # the knuckles' root, so the last value is held.
    inner_front=(
        -0.04386,
        -0.04230,
        -0.04086,
        -0.03955,
        -0.03837,
        -0.03734,
        -0.03647,
        -0.03578,
        -0.03529,
        -0.03504,
        -0.03512,
        -0.03572,
        -0.03737,
        -0.04246,
        -0.04246,
        -0.04246,
        -0.04246,
    ),
    inner_width=0.039,
    # Knuckles and outer fingers reach ~25 mm behind the TCP beside the pads
    # (FK of their collision meshes over the stroke), 39 mm wide.
    linkage_front=-0.025,
    linkage_thickness=0.03,
)

#: Franka Panda hand (``panda_hand_tcp``, identity ``<gripper>`` pose, so the
#: gripper frame's +Z is the approach and +Y the closing axis). Each
#: prismatic finger joint is half the opening -- exact; the pad and palm
#: sizes are nominal (Franka datasheet), not measured on this repo's URDF.
PANDA_HAND = ParallelGripperModel(
    name="panda_hand",
    joints={"panda_finger_joint1": 0.5, "panda_finger_joint2": 0.5},
    stroke=((0.0, 0.0, -0.01), (0.08, 0.08, -0.01)),
    pad_length=0.02,
    pad_width=0.018,
    finger_thickness=0.01,
    finger_width=0.02,
    palm_front=-0.045,
    palm_size=(0.06, 0.2, 0.06),
    # gripper x = -canonical z, gripper y = canonical y, gripper z = canonical x
    frame_rotation=np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]]),
    default_squeeze=0.001,
)

#: SO-101 jaw (``SO-ARM100/Simulation/SO101/so101_new_calib.urdf``): the
#: ``gripper`` joint (radians) swings the moving jaw against the fixed
#: finger of ``gripper_link``. ``<gripper>`` frame on ``gripper_frame_link``,
#: whose +Z is the approach and +X the opening direction (moving jaw at -X,
#: fixed finger's inner face at x = +2.0 mm). Gap and pad midpoint measured
#: from the STL meshes the way soarm_tamp's ``studies/reachability.py`` does
#: (inner faces in the z in [-35, -10] mm band, so that band is the pad);
#: they match its ``geometry.JAW_TABLE`` to 0.05 mm. The joint opens to
#: 100 deg, but past 30 deg (the open command soarm_tamp uses) the jaw's tip
#: swings back out of the pad band, so the stroke stops there. Hull boxes
#: from the meshes' extents: the fingertips reach 8 mm past the TCP, the
#: servo and wrist body start 45 mm behind it.
_SO101_JAW = (  # angle (deg), gap (mm), pad midpoint along gripper x (mm)
    (-10.0, 8.23, -2.12),
    (-5.0, 14.21, -5.10),
    (0.0, 19.80, -7.90),
    (5.0, 25.24, -10.62),
    (10.0, 29.14, -12.57),
    (15.0, 32.84, -14.42),
    (20.0, 36.51, -16.26),
    (25.0, 40.14, -18.07),
    (30.0, 43.69, -19.84),
)
SO101_JAW = RotaryJawGripperModel(
    name="so101_jaw",
    joints={"gripper": 1.0},
    stroke=tuple(
        (float(np.radians(deg)), gap / 1000, -0.0225) for deg, gap, _ in _SO101_JAW
    ),
    center=tuple(mid / 1000 for _, _, mid in _SO101_JAW),
    pad_length=0.025,
    pad_width=0.014,
    finger_thickness=0.015,
    finger_width=0.016,
    finger_tip=0.008,
    palm_front=-0.045,
    palm_size=(0.055, 0.066, 0.052),
    palm_offset=-0.0055,
    # gripper x = canonical y, gripper y = canonical z, gripper z = canonical x
    frame_rotation=np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
    # Position-controlled serial servo: grip force comes from commanding
    # past contact and stalling (soarm_tamp closes ~5 mm past a 30 mm cube).
    default_squeeze=0.005,
)

#: Hand models by name. :func:`register_gripper` adds one.
PRESETS: dict[str, GripperModel] = {
    ROBOTIQ_2F85.name: ROBOTIQ_2F85,
    PANDA_HAND.name: PANDA_HAND,
    SO101_JAW.name: SO101_JAW,
}


def register_gripper(model: GripperModel, *, replace: bool = False) -> GripperModel:
    """Add ``model`` to :data:`PRESETS` under its name; refuses to replace a
    different model of that name unless ``replace``."""
    if not isinstance(model, GripperModel):
        raise TypeError(f"{model!r} does not implement GripperModel")
    if not replace and PRESETS.get(model.name, model) is not model:
        raise ValueError(f"a gripper model named {model.name!r} is registered")
    PRESETS[model.name] = model
    return model


def _slab_front(tris: np.ndarray, half: float, closing: int, approach: int) -> float:
    """Foremost approach coordinate of triangles clipped to ``|closing| <= half``."""
    best = -np.inf
    for tri in tris:
        poly = list(tri)
        for sign in (1.0, -1.0):  # clip to sign * p[closing] <= half
            out = []
            for k in range(len(poly)):
                a, b = poly[k], poly[(k + 1) % len(poly)]
                da, db = sign * a[closing] - half, sign * b[closing] - half
                if da <= 0:
                    out.append(a)
                if da * db < 0:
                    out.append(a + (b - a) * (da / (da - db)))
            poly = out
            if not poly:
                break
        if poly:
            best = max(best, max(v[approach] for v in poly))
    return float(best)


def calibrate_from_urdf(
    urdf_path: str | Path,
    *,
    tcp_frame: str,
    pad_frames: tuple[str, str],
    joints: Mapping[str, float],
    q_values: Sequence[float],
    pad_thickness: float = 0.0,
    approach_axis: int = 2,
    closing_axis: int = 1,
    inner_links: Sequence[str] = (),
) -> tuple[tuple[tuple[float, float, float], ...], tuple[float, ...]]:
    """Measure a stroke table by forward kinematics (needs pinocchio; coal
    meshes for ``inner_links``).

    For each driving value the finger joints are set through ``joints``'
    multipliers and the two pad frames are expressed in ``tcp_frame``.
    ``approach_axis`` / ``closing_axis`` index the TCP frame's axes (the
    2F-85's ``gripper_tcp`` approaches along its Z and closes along Y).
    ``pad_thickness`` is subtracted from the frame-to-frame distance when
    the pad frames sit at the pads' centres rather than their inner faces.

    Returns ``(stroke, inner_front)``: the stroke rows, and -- when
    ``inner_links`` is given -- the foremost point of those links' collision
    meshes between the pads (triangles clipped to the gap, not just their
    vertices: a sparse mesh reaches well past its vertices), else ``()``.
    """
    import pinocchio as pin

    model = pin.buildModelFromUrdf(str(urdf_path))
    data = model.createData()
    geom = gdata = None
    if inner_links:
        from long_tamp.backends._urdf_paths import resolve_mesh_paths

        geom = pin.buildGeomFromUrdf(
            model, resolve_mesh_paths(str(urdf_path)), pin.COLLISION
        )
        gdata = geom.createData()
    tcp = model.getFrameId(tcp_frame)
    fa, fb = (model.getFrameId(f) for f in pad_frames)
    rows, fronts = [], []
    for q_drive in q_values:
        q = pin.neutral(model)
        for joint, mult in joints.items():
            jid = model.getJointId(joint)
            q[model.idx_qs[jid]] = mult * q_drive
        pin.framesForwardKinematics(model, data, q)
        inv = data.oMf[tcp].inverse()
        pa = (inv * data.oMf[fa]).translation
        pb = (inv * data.oMf[fb]).translation
        gap = abs(pa[closing_axis] - pb[closing_axis]) - pad_thickness
        rows.append(
            (
                float(q_drive),
                float(gap),
                float((pa[approach_axis] + pb[approach_axis]) / 2),
            )
        )
        if geom is not None:
            pin.updateGeometryPlacements(model, data, geom, gdata, q)
            front = -np.inf
            for k, obj in enumerate(geom.geometryObjects):
                if model.frames[obj.parentFrame].name not in inner_links:
                    continue
                if not hasattr(obj.geometry, "tri_indices"):
                    continue
                M = (inv * gdata.oMg[k]).homogeneous
                v = np.asarray(obj.geometry.vertices()) @ M[:3, :3].T + M[:3, 3]
                idx = [
                    obj.geometry.tri_indices(i) for i in range(obj.geometry.num_tris)
                ]
                tris = v[np.array([[t[0], t[1], t[2]] for t in idx])]
                front = max(
                    front, _slab_front(tris, gap / 2, closing_axis, approach_axis)
                )
            fronts.append(front)
    return tuple(rows), tuple(fronts)
