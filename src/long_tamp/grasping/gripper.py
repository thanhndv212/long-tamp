"""Parallel-jaw gripper models: finger kinematics and a coarse collision hull.

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
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


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
        back = self.palm_front - sweep
        boxes = []

        def slab(name, sign, x0, x1, thickness, size_z):
            y0, y1 = width / 2, width / 2 + thickness
            center = np.array([(x0 + x1) / 2, sign * (y0 + y1) / 2, 0.0])
            half = np.array([(x1 - x0) / 2, (y1 - y0) / 2, size_z / 2])
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
        center = np.array([pf - depth / 2 - sweep / 2, 0.0, 0.0])
        half = np.array([(depth + sweep) / 2, sy / 2, sz / 2])
        boxes.append(("palm", center, half))
        inner = self.inner_front_for_width(width)
        if inner is not None and inner > back:
            center = np.array([(inner + back) / 2, 0.0, 0.0])
            half = np.array([(inner - back) / 2, width / 2, self.inner_width / 2])
            boxes.append(("knuckles", center, half))
        return boxes


# ---------------------------------------------------------------------------
# Presets
# ---------------------------------------------------------------------------

#: Robotiq 2F-85 as merged into this repo's UR10 (ikea_table_prototype's
#: ``ur10_robotiq*.urdf``, gripper frame on ``gripper_tcp`` 0.15 m in front
#: of ``robotiq_arg2f_base_link``). Stroke measured with pinocchio forward
#: kinematics of that URDF, pad inner faces (``*_inner_finger_pad``, 6.35 mm
#: thick) -- see :func:`calibrate_from_urdf` to redo it. Its HPP gripper
#: frame already uses +X approach / Y closing, so ``frame_rotation`` is I.
ROBOTIQ_2F85 = ParallelGripperModel(
    name="robotiq_2f85",
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

PRESETS: dict[str, ParallelGripperModel] = {
    ROBOTIQ_2F85.name: ROBOTIQ_2F85,
    PANDA_HAND.name: PANDA_HAND,
}


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
