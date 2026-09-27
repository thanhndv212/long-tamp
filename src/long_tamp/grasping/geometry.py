"""Object geometry for grasp planning: primitives, URDF loading, queries.

A grasp planner needs to know an object's *shape*, not its mesh. This module
describes an object as a list of convex primitives (:class:`Box`,
:class:`Cylinder`, :class:`Sphere`) posed in the object's root-link frame,
and answers the two questions the planner asks of them:

- where does a line cross the object (:func:`line_intervals`) -- used to find
  where closing fingers first touch it;
- does a box overlap it (:func:`box_overlaps`) -- used to reject grasps whose
  palm or fingers would collide.

Everything here is plain numpy: no HPP, no pinocchio. Poses are 4x4
homogeneous matrices. URDF collision elements load via
:func:`load_urdf_primitives`; a ``<mesh>`` collision is replaced by its
oriented bounding box when ``trimesh`` can read it (a coarse but safe
stand-in), and skipped with a warning otherwise.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from long_tamp.logging import get_logger

logger = get_logger("grasping.geometry")

_EPS = 1e-9


# ---------------------------------------------------------------------------
# Pose helpers
# ---------------------------------------------------------------------------


def pose(rotation: np.ndarray | None = None, translation=(0.0, 0.0, 0.0)) -> np.ndarray:
    """4x4 homogeneous transform from a 3x3 rotation and a translation."""
    T = np.eye(4)
    if rotation is not None:
        T[:3, :3] = rotation
    T[:3, 3] = translation
    return T


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF ``rpy`` (fixed-axis X, then Y, then Z) as a rotation matrix."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def quat_xyzw_to_matrix(q: Sequence[float]) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < _EPS:
        raise ValueError("zero quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> unit quaternion ``[x, y, z, w]`` with ``w >= 0``."""
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    return -q if q[3] < 0 else q


def xyzquat_to_pose(xyzquat: Sequence[float]) -> np.ndarray:
    return pose(quat_xyzw_to_matrix(xyzquat[3:7]), xyzquat[:3])


def pose_to_xyzquat(T: np.ndarray) -> list[float]:
    return [float(v) for v in T[:3, 3]] + [
        float(v) for v in matrix_to_quat_xyzw(T[:3, :3])
    ]


def inverse(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    return pose(R.T, -R.T @ t)


# ---------------------------------------------------------------------------
# Primitives
# ---------------------------------------------------------------------------


@dataclass
class Primitive:
    """A convex shape posed in the object frame (``pose``: object <- shape)."""

    pose: np.ndarray = field(default_factory=lambda: np.eye(4))
    name: str = ""

    def half_extents(self) -> np.ndarray:  # pragma: no cover - abstract
        """Half sizes of the shape's bounding box in its own frame."""
        raise NotImplementedError

    def volume(self) -> float:  # pragma: no cover - abstract
        raise NotImplementedError

    def interval_local(
        self, p: np.ndarray, d: np.ndarray
    ) -> tuple[float, float] | None:
        """Parameter interval ``[t0, t1]`` where ``p + t d`` (shape frame) is inside."""
        raise NotImplementedError  # pragma: no cover - abstract

    @property
    def center(self) -> np.ndarray:
        return self.pose[:3, 3].copy()


@dataclass
class Box(Primitive):
    size: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def half_extents(self) -> np.ndarray:
        return np.asarray(self.size, dtype=float) / 2

    def volume(self) -> float:
        return float(np.prod(self.size))

    def interval_local(self, p, d):
        return _slab_interval(p, d, self.half_extents())

    def intervals_local(self, P, d):
        h = self.half_extents()
        n = len(P)
        t0, t1 = np.full(n, -np.inf), np.full(n, np.inf)
        miss = np.zeros(n, dtype=bool)
        for i in range(3):
            if abs(d[i]) < _EPS:
                miss |= np.abs(P[:, i]) > h[i]
                continue
            a, b = (-h[i] - P[:, i]) / d[i], (h[i] - P[:, i]) / d[i]
            t0, t1 = np.maximum(t0, np.minimum(a, b)), np.minimum(t1, np.maximum(a, b))
        miss |= t0 > t1
        return np.where(miss, np.nan, t0), np.where(miss, np.nan, t1)


@dataclass
class Cylinder(Primitive):
    """Cylinder along its local Z (URDF convention)."""

    radius: float = 0.0
    length: float = 0.0

    def half_extents(self) -> np.ndarray:
        return np.array([self.radius, self.radius, self.length / 2])

    def volume(self) -> float:
        return math.pi * self.radius**2 * self.length

    def interval_local(self, p, d):
        hz = self.length / 2
        # Axial slab.
        if abs(d[2]) < _EPS:
            if abs(p[2]) > hz:
                return None
            tz0, tz1 = -math.inf, math.inf
        else:
            a, b = (-hz - p[2]) / d[2], (hz - p[2]) / d[2]
            tz0, tz1 = min(a, b), max(a, b)
        # Radial: |p_xy + t d_xy|^2 <= r^2.
        A = d[0] ** 2 + d[1] ** 2
        B = 2 * (p[0] * d[0] + p[1] * d[1])
        C = p[0] ** 2 + p[1] ** 2 - self.radius**2
        if A < _EPS:
            if C > 0:
                return None
            tr0, tr1 = -math.inf, math.inf
        else:
            disc = B * B - 4 * A * C
            if disc < 0:
                return None
            s = math.sqrt(disc)
            tr0, tr1 = (-B - s) / (2 * A), (-B + s) / (2 * A)
        t0, t1 = max(tz0, tr0), min(tz1, tr1)
        return (t0, t1) if t0 <= t1 else None

    def intervals_local(self, P, d):
        hz, n = self.length / 2, len(P)
        miss = np.zeros(n, dtype=bool)
        if abs(d[2]) < _EPS:
            miss |= np.abs(P[:, 2]) > hz
            tz0, tz1 = np.full(n, -np.inf), np.full(n, np.inf)
        else:
            a, b = (-hz - P[:, 2]) / d[2], (hz - P[:, 2]) / d[2]
            tz0, tz1 = np.minimum(a, b), np.maximum(a, b)
        A = d[0] ** 2 + d[1] ** 2
        C = P[:, 0] ** 2 + P[:, 1] ** 2 - self.radius**2
        if A < _EPS:
            miss |= C > 0
            tr0, tr1 = np.full(n, -np.inf), np.full(n, np.inf)
        else:
            B = 2 * (P[:, 0] * d[0] + P[:, 1] * d[1])
            disc = B * B - 4 * A * C
            miss |= disc < 0
            s = np.sqrt(np.maximum(disc, 0.0))
            tr0, tr1 = (-B - s) / (2 * A), (-B + s) / (2 * A)
        t0, t1 = np.maximum(tz0, tr0), np.minimum(tz1, tr1)
        miss |= t0 > t1
        return np.where(miss, np.nan, t0), np.where(miss, np.nan, t1)


@dataclass
class Sphere(Primitive):
    radius: float = 0.0

    def half_extents(self) -> np.ndarray:
        return np.full(3, self.radius)

    def volume(self) -> float:
        return 4.0 / 3.0 * math.pi * self.radius**3

    def interval_local(self, p, d):
        A = float(d @ d)
        B = 2 * float(p @ d)
        C = float(p @ p) - self.radius**2
        disc = B * B - 4 * A * C
        if disc < 0:
            return None
        s = math.sqrt(disc)
        return ((-B - s) / (2 * A), (-B + s) / (2 * A))

    def intervals_local(self, P, d):
        A = float(d @ d)
        B = 2 * (P @ d)
        C = np.einsum("ij,ij->i", P, P) - self.radius**2
        disc = B * B - 4 * A * C
        s = np.sqrt(np.maximum(disc, 0.0))
        t0, t1 = (-B - s) / (2 * A), (-B + s) / (2 * A)
        miss = disc < 0
        return np.where(miss, np.nan, t0), np.where(miss, np.nan, t1)


def _slab_interval(p, d, h) -> tuple[float, float] | None:
    t0, t1 = -math.inf, math.inf
    for i in range(3):
        if abs(d[i]) < _EPS:
            if abs(p[i]) > h[i]:
                return None
            continue
        a, b = (-h[i] - p[i]) / d[i], (h[i] - p[i]) / d[i]
        t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
        if t0 > t1:
            return None
    return (t0, t1)


# ---------------------------------------------------------------------------
# Queries
# ---------------------------------------------------------------------------


def line_intervals(
    primitives: Iterable[Primitive], point: np.ndarray, direction: np.ndarray
) -> list[tuple[float, float]]:
    """Intervals of ``t`` where ``point + t * direction`` is inside any primitive.

    ``point``/``direction`` are in the object frame; ``direction`` need not be
    unit length (``t`` is in its units). Overlapping intervals are not merged.
    """
    out = []
    point = np.asarray(point, dtype=float)
    direction = np.asarray(direction, dtype=float)
    for prim in primitives:
        R, t = prim.pose[:3, :3], prim.pose[:3, 3]
        iv = prim.interval_local(R.T @ (point - t), R.T @ direction)
        if iv is not None:
            out.append(iv)
    return out


def line_intervals_batch(
    primitives: Sequence[Primitive], points: np.ndarray, direction: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """:func:`line_intervals` for many parallel lines at once.

    Returns ``(t0, t1)`` arrays of shape ``(len(primitives), len(points))``,
    NaN where a line misses a primitive.
    """
    points = np.asarray(points, dtype=float)
    direction = np.asarray(direction, dtype=float)
    T0 = np.full((len(primitives), len(points)), np.nan)
    T1 = np.full_like(T0, np.nan)
    for k, prim in enumerate(primitives):
        R, t = prim.pose[:3, :3], prim.pose[:3, 3]
        T0[k], T1[k] = prim.intervals_local((points - t) @ R, R.T @ direction)
    return T0, T1


def box_overlaps(
    primitive: Primitive,
    box_pose: np.ndarray,
    box_half: np.ndarray,
    margin: float = 0.0,
) -> bool:
    """Whether an oriented box (object frame) overlaps ``primitive``.

    Boxes use an exact separating-axis test. Spheres test the distance from
    the centre to the box. Cylinders are tested as their bounding box, which
    is conservative (may report a collision that a round side would miss).
    ``margin`` grows the query box on every side (clearance).
    """
    box_half = np.asarray(box_half, dtype=float) + margin
    if isinstance(primitive, Sphere):
        local = inverse(box_pose) @ np.append(primitive.center, 1.0)
        closest = np.clip(local[:3], -box_half, box_half)
        return float(np.linalg.norm(local[:3] - closest)) < primitive.radius
    return obb_overlap(primitive.pose, primitive.half_extents(), box_pose, box_half)


def obb_overlap(Ta: np.ndarray, ha: np.ndarray, Tb: np.ndarray, hb: np.ndarray) -> bool:
    """Separating-axis test for two oriented boxes (strict overlap)."""
    A, B = Ta[:3, :3], Tb[:3, :3]
    R = A.T @ B
    t = A.T @ (Tb[:3, 3] - Ta[:3, 3])
    absR = np.abs(R) + 1e-12
    for i in range(3):
        if abs(t[i]) >= ha[i] + hb @ absR[i]:
            return False
    for j in range(3):
        if abs(t @ R[:, j]) >= ha @ absR[:, j] + hb[j]:
            return False
    for i in range(3):
        for j in range(3):
            i1, i2 = (i + 1) % 3, (i + 2) % 3
            j1, j2 = (j + 1) % 3, (j + 2) % 3
            ra = ha[i1] * absR[i2, j] + ha[i2] * absR[i1, j]
            rb = hb[j1] * absR[i, j2] + hb[j2] * absR[i, j1]
            if abs(t[i2] * R[i1, j] - t[i1] * R[i2, j]) >= ra + rb:
                return False
    return True


def bounding_box(primitives: Sequence[Primitive]) -> tuple[np.ndarray, np.ndarray]:
    """Axis-aligned ``(min, max)`` corners of the primitives in the object frame."""
    lo, hi = np.full(3, math.inf), np.full(3, -math.inf)
    for prim in primitives:
        R, t = prim.pose[:3, :3], prim.pose[:3, 3]
        ext = np.abs(R) @ prim.half_extents()
        lo, hi = np.minimum(lo, t - ext), np.maximum(hi, t + ext)
    return lo, hi


def centroid(primitives: Sequence[Primitive]) -> np.ndarray:
    """Volume-weighted centre of the primitives (a centre-of-mass proxy)."""
    vols = np.array([max(p.volume(), _EPS) for p in primitives])
    centers = np.array([p.center for p in primitives])
    return (vols[:, None] * centers).sum(axis=0) / vols.sum()


# ---------------------------------------------------------------------------
# URDF loading
# ---------------------------------------------------------------------------


def _origin(el: ET.Element | None) -> np.ndarray:
    if el is None:
        return np.eye(4)
    xyz = [float(v) for v in el.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in el.get("rpy", "0 0 0").split()]
    return pose(rpy_matrix(*rpy), xyz)


def link_poses(root: ET.Element) -> dict[str, np.ndarray]:
    """Each link's pose in the root link's frame, through fixed joints only.

    Links hanging off a movable joint are placed at the joint's zero value
    (grasp planning works on rigid objects; an articulated one is planned at
    its reference configuration).
    """
    links = [link.get("name") for link in root.findall("link")]
    children = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        children.setdefault(parent, []).append((child, _origin(joint.find("origin"))))
    child_names = {c for kids in children.values() for c, _ in kids}
    roots = [name for name in links if name not in child_names]
    poses: dict[str, np.ndarray] = {}
    stack = [(r, np.eye(4)) for r in roots[:1]]
    while stack:
        name, T = stack.pop()
        poses[name] = T
        for child, T_joint in children.get(name, []):
            stack.append((child, T @ T_joint))
    return poses


def _mesh_obb(
    filename: str, urdf_dir: Path, scale: np.ndarray, T: np.ndarray, name: str
):
    try:
        import trimesh
    except ImportError:
        logger.warning("skipping mesh collision %s: trimesh not installed", filename)
        return None
    from long_tamp.backends._urdf_paths import resolve_mesh_path

    path = resolve_mesh_path(filename, urdf_dir)
    try:
        mesh = trimesh.load(path, force="mesh")
    except Exception as exc:
        logger.warning("skipping mesh collision %s: %s", filename, exc)
        return None
    mesh.apply_scale(scale)
    obb = mesh.bounding_box_oriented
    return Box(
        pose=T @ np.asarray(obb.transform), size=tuple(obb.primitive.extents), name=name
    )


def load_urdf_primitives(
    urdf_path: str | Path, links: Sequence[str] | None = None
) -> list[Primitive]:
    """Collision primitives of a URDF, expressed in its root link's frame.

    Args:
        urdf_path: The object's URDF.
        links: Only these links (default: every link).
    """
    urdf_path = Path(urdf_path)
    root = ET.parse(urdf_path).getroot()
    poses_by_link = link_poses(root)
    prims: list[Primitive] = []
    for link in root.findall("link"):
        lname = link.get("name")
        if links is not None and lname not in links:
            continue
        T_link = poses_by_link.get(lname, np.eye(4))
        for k, col in enumerate(link.findall("collision")):
            geom = col.find("geometry")
            if geom is None or len(geom) == 0:
                continue
            T = T_link @ _origin(col.find("origin"))
            name = col.get("name") or f"{lname}_{k}"
            shape = geom[0]
            if shape.tag == "box":
                size = tuple(float(v) for v in shape.get("size").split())
                prims.append(Box(pose=T, size=size, name=name))
            elif shape.tag == "cylinder":
                prims.append(
                    Cylinder(
                        pose=T,
                        radius=float(shape.get("radius")),
                        length=float(shape.get("length")),
                        name=name,
                    )
                )
            elif shape.tag == "sphere":
                prims.append(
                    Sphere(pose=T, radius=float(shape.get("radius")), name=name)
                )
            elif shape.tag == "mesh":
                scale = np.array(
                    [float(v) for v in shape.get("scale", "1 1 1").split()]
                )
                box = _mesh_obb(shape.get("filename"), urdf_path.parent, scale, T, name)
                if box is not None:
                    prims.append(box)
            else:
                logger.warning(
                    "skipping unsupported collision geometry <%s>", shape.tag
                )
    if not prims:
        raise ValueError(f"{urdf_path}: no usable collision geometry")
    return prims


@dataclass(frozen=True)
class SrdfFrame:
    """A ``<handle>`` or ``<gripper>`` read from an SRDF (pose in its link)."""

    name: str
    kind: str  # "handle" | "gripper"
    link: str
    pose: np.ndarray
    clearance: float
    approaching_direction: tuple[float, float, float] = (1.0, 0.0, 0.0)


def load_srdf_frames(srdf_path: str | Path) -> dict[str, SrdfFrame]:
    """Every ``<handle>``/``<gripper>`` of an SRDF, keyed by name.

    Accepts both position forms used in this repo: ``xyz``/``xyzw``
    attributes and the legacy ``x y z  w x y z`` text (HPP's wxyz order).
    """
    root = ET.parse(srdf_path).getroot()
    out = {}
    for kind in ("handle", "gripper"):
        for el in root.findall(kind):
            pos = el.find("position")
            if pos is not None and pos.get("xyz") is not None:
                xyz = [float(v) for v in pos.get("xyz").split()]
                R = quat_xyzw_to_matrix([float(v) for v in pos.get("xyzw").split()])
            elif pos is not None and (pos.text or "").strip():
                vals = [float(v) for v in pos.text.split()]
                xyz, (w, x, y, z) = vals[:3], vals[3:7]
                R = quat_xyzw_to_matrix([x, y, z, w])
            else:
                xyz, R = [0.0, 0.0, 0.0], np.eye(3)
            link = el.find("link")
            approach = tuple(
                float(v) for v in el.get("approaching_direction", "1 0 0").split()
            )
            out[el.get("name")] = SrdfFrame(
                name=el.get("name"),
                kind=kind,
                link=link.get("name") if link is not None else "base_link",
                pose=pose(R, xyz),
                clearance=float(el.get("clearance", "0")),
                approaching_direction=approach,
            )
    return out
