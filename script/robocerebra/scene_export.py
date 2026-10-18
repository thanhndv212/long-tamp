"""Export a compiled MuJoCo scene as URDF/SRDF for HPP (issue #157, R2).

RoboCerebra's scenes are built by LIBERO from a BDDL file into one MuJoCo
model; long_tamp plans on HPP, which loads URDF/SRDF. This module turns the
model *as it stands* (after a reset: fixtures placed, objects at their initial
poses) into:

``environment.urdf``
    Every static body's collision geometry (fixtures, fixed objects, the
    floor), one link per MuJoCo body, fixed at its current world pose.
    Articulated parts (cabinet drawers, the microwave door, the stove knob)
    are baked at their current joint value; the manifest lists those joints.
``environment.srdf``
    One ``<contact>`` per placement region (``env.regions``): the region
    site's footprint clipped to the top face of the geometry that supports
    it, normal up.
``objects/<name>.urdf|srdf``
    Each free-floating object's collision geometry in its body frame, and a
    ``bottom`` contact (the face that points down at the current pose,
    normal down). Grasp handles are #158's.
``manifest.json``
    Object poses, the robot's base pose and joint values, every region with
    its support (or why it has none), the baked joints, and what was skipped.

Only MuJoCo and numpy: it runs in the benchmark env (``env_server.py``'s
``export_scene`` op) and in tests over a hand-written MJCF. Collision
geometry is what has ``contype`` or ``conaffinity`` set; in RoboCerebra that
is boxes and a few capsules (a capsule is a cylinder plus two spheres in
URDF, which has no capsule). Meshes and ellipsoids are skipped and reported.
"""

from __future__ import annotations

import math
import re
from typing import Any

import mujoco
import numpy as np

# A region's support may sit a little above its site (the fork's table
# regions are 4 mm under the top) or below it (a compartment floor).
SUPPORT_ABOVE = 0.03
SUPPORT_BELOW = 0.25
FLOOR_EXTENT = 2.5  # half-size of the floor slab when the plane is infinite
FLOOR_THICKNESS = 0.01
UP_TOL = 0.999
# A face supports a region if it covers this much of the region's footprint
# (or the footprint this much of it: the table region is 4 m across) and is
# at least this wide.
MIN_OVERLAP = 0.2
MIN_SUPPORT_WIDTH = 0.02  # |cos| between a face normal and world z for "horizontal"


def export_scene(
    model: Any,
    data: Any,
    *,
    objects: dict[str, str] | None = None,
    regions: list[str] | None = None,
    robot_root: str = "robot0_base",
    robot_joints: list[str] | None = None,
    env_name: str = "environment",
) -> dict[str, Any]:
    """The scene in ``data`` (call ``mj_forward`` first) as file texts and a
    manifest: ``{"files": {relpath: text}, "manifest": {...}}``.

    ``objects``: movable object name -> its root body (default: every body
    with a free joint, named after the body). ``regions``: the placement
    region site names (default: every site ending in ``_region``).
    ``robot_root``: the robot's root body; it and everything under it are
    left out (long_tamp loads its own robot model).
    """
    names = _Names(model)
    body_id = {names.body(i): i for i in range(model.nbody)}
    if objects is None:
        objects = {
            names.body(model.jnt_bodyid[j]): names.body(model.jnt_bodyid[j])
            for j in range(model.njnt)
            if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE
        }
    children: dict[int, list[int]] = {}
    for i in range(1, model.nbody):
        children.setdefault(int(model.body_parentid[i]), []).append(i)

    def subtree(root: int) -> list[int]:
        out, stack = [], [root]
        while stack:
            b = stack.pop()
            out.append(b)
            stack.extend(children.get(b, []))
        return out

    robot = set(subtree(body_id[robot_root])) if robot_root in body_id else set()
    object_bodies = {name: subtree(body_id[root]) for name, root in objects.items()}
    movable = {b for bodies in object_bodies.values() for b in bodies}
    static = [b for b in range(model.nbody) if b not in robot and b not in movable]

    skipped: list[dict[str, Any]] = []
    files: dict[str, str] = {}
    shapes: dict[str, dict[str, Any]] = {}

    # -- environment --
    env_links: dict[int, str] = {}
    env_geoms: list[int] = []
    links_xml = [f'  <link name="{env_name}_root"/>']
    for b in static:
        geoms = [g for g in _collision_geoms(model, b) if _supported(model, names, g, skipped)]
        if not geoms:
            continue
        link = _sanitize(names.body(b) or f"body{b}")
        env_links[b] = link
        env_geoms += geoms
        pos, rot = data.xpos[b].copy(), data.xmat[b].reshape(3, 3).copy()
        links_xml.append(_link_xml(model, names, link, geoms))
        _record_shapes(model, data, names, f"{env_name}/{link}", b, geoms, shapes)
        links_xml.append(
            f'  <joint name="{link}_fixed" type="fixed">\n'
            f'    <parent link="{env_name}_root"/>\n    <child link="{link}"/>\n'
            f"    {_origin(pos, rot)}\n  </joint>"
        )
    files["environment.urdf"] = _robot_xml(env_name, links_xml)

    region_sites = regions
    if region_sites is None:
        region_sites = [
            names.site(i) for i in range(model.nsite) if names.site(i).endswith("_region")
        ]
    region_info, contacts_xml = [], []
    for region in region_sites:
        info = _region_support(model, data, names, region, env_geoms, env_links)
        region_info.append(info)
        if info.get("points") is not None:
            contacts_xml.append(
                _contact_xml(region, info["link"], np.asarray(info.pop("points_link")))
            )
        info.pop("points_link", None)
    files["environment.srdf"] = _robot_xml(env_name, contacts_xml, srdf=True)

    # -- objects --
    object_info: dict[str, Any] = {}
    for name, bodies in object_bodies.items():
        root = bodies[0]
        root_pos, root_rot = data.xpos[root].copy(), data.xmat[root].reshape(3, 3).copy()
        links_xml, geoms_all = [], []
        link_of = {}
        for b in bodies:
            geoms = [
                g for g in _collision_geoms(model, b) if _supported(model, names, g, skipped)
            ]
            if b != root and not geoms:
                continue
            link = "base_link" if b == root else _sanitize(names.body(b))
            link_of[b] = link
            geoms_all += geoms
            links_xml.append(_link_xml(model, names, link, geoms))
            _record_shapes(model, data, names, f"{name}/{link}", b, geoms, shapes)
            if b != root:
                # Bodies under the object's root are rigidly attached in
                # RoboCerebra's objects; bake them at their current pose.
                rel = root_rot.T @ (data.xpos[b] - root_pos)
                rel_rot = root_rot.T @ data.xmat[b].reshape(3, 3)
                links_xml.append(
                    f'  <joint name="{link}_fixed" type="fixed">\n'
                    f'    <parent link="base_link"/>\n    <child link="{link}"/>\n'
                    f"    {_origin(rel, rel_rot)}\n  </joint>"
                )
        files[f"objects/{name}.urdf"] = _robot_xml(name, links_xml)
        bottom = _bottom_face(model, data, geoms_all, root_pos, root_rot)
        contacts_xml = []
        if bottom is not None:
            contacts_xml.append(_contact_xml("bottom", "base_link", bottom[0]))
        else:
            skipped.append({"what": f"{name}/bottom", "reason": "no collision geometry"})
        files[f"objects/{name}.srdf"] = _robot_xml(name, contacts_xml, srdf=True)
        object_info[name] = {
            "body": names.body(root),
            "pose": _xyzquat(root_pos, root_rot),
            "geoms": [names.geom(g) for g in geoms_all],
            "bottom": None if bottom is None else bottom[0].tolist(),
            # > 0: resting tilted (or not resting on a face at all).
            "bottom_tilt": None if bottom is None else bottom[1],
        }

    # -- robot and articulations --
    robot_info: dict[str, Any] = {"root": robot_root}
    if robot_root in body_id:
        b = body_id[robot_root]
        robot_info["pose"] = _xyzquat(data.xpos[b], data.xmat[b].reshape(3, 3))
        joints = robot_joints or [
            names.joint(j) for j in range(model.njnt) if model.jnt_bodyid[j] in robot
        ]
        robot_info["joints"] = {j: _joint_value(model, data, j) for j in joints}
        robot_info["links"] = {
            names.body(i): _xyzquat(data.xpos[i], data.xmat[i].reshape(3, 3))
            for i in sorted(robot)
        }
    articulated = []
    static_set = set(static)
    for j in range(model.njnt):
        if model.jnt_bodyid[j] not in static_set:
            continue
        kind = mujoco.mjtJoint(model.jnt_type[j]).name[len("mjJNT_") :].lower()
        b = int(model.jnt_bodyid[j])
        articulated.append(
            {
                "joint": names.joint(j),
                "type": kind,
                "value": _joint_value(model, data, names.joint(j)),
                "range": model.jnt_range[j].tolist() if model.jnt_limited[j] else None,
                "axis_world": (data.xmat[b].reshape(3, 3) @ model.jnt_axis[j]).tolist(),
                "bodies": [names.body(x) for x in subtree(b)],
            }
        )

    exported = env_geoms + [
        names.geom_id(g) for info in object_info.values() for g in info["geoms"]
    ]
    manifest = {
        "environment": {
            "name": env_name,
            "links": {names.body(b): link for b, link in env_links.items()},
            "geoms": [names.geom(g) for g in env_geoms],
        },
        "objects": object_info,
        "regions": region_info,
        "robot": robot_info,
        "articulated": articulated,
        # Every exported collision shape's world pose, under the name HPP
        # gives it ("<model>/<link>_<i>"), and MuJoCo's contacts among the
        # exported geoms: for checking the export against the HPP model.
        "shapes": shapes,
        "contacts": _contacts(data, names, set(exported)),
        "skipped": skipped,
    }
    return {"files": files, "manifest": manifest}


# -- geometry ---------------------------------------------------------------


class _Names:
    def __init__(self, model: Any) -> None:
        self.model = model

    def _name(self, kind: Any, i: int) -> str:
        return mujoco.mj_id2name(self.model, kind, int(i)) or ""

    def body(self, i: int) -> str:
        return self._name(mujoco.mjtObj.mjOBJ_BODY, i)

    def geom(self, i: int) -> str:
        return self._name(mujoco.mjtObj.mjOBJ_GEOM, i) or f"geom{i}"

    def geom_id(self, name: str) -> int:
        if name.startswith("geom") and name[4:].isdigit():
            return int(name[4:])
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)

    def site(self, i: int) -> str:
        return self._name(mujoco.mjtObj.mjOBJ_SITE, i)

    def joint(self, i: int) -> str:
        return self._name(mujoco.mjtObj.mjOBJ_JOINT, i)


def _collision_geoms(model: Any, body: int) -> list[int]:
    return [
        g
        for g in range(model.ngeom)
        if model.geom_bodyid[g] == body and (model.geom_contype[g] or model.geom_conaffinity[g])
    ]


# Keyed by int: newer mujoco enums don't hash like the model's numpy ints.
_KINDS = {
    int(mujoco.mjtGeom.mjGEOM_BOX): "box",
    int(mujoco.mjtGeom.mjGEOM_SPHERE): "sphere",
    int(mujoco.mjtGeom.mjGEOM_CYLINDER): "cylinder",
    int(mujoco.mjtGeom.mjGEOM_CAPSULE): "capsule",
    int(mujoco.mjtGeom.mjGEOM_PLANE): "plane",
}


def _supported(model: Any, names: _Names, g: int, skipped: list) -> bool:
    if int(model.geom_type[g]) in _KINDS:
        return True
    kind = mujoco.mjtGeom(model.geom_type[g]).name
    skipped.append({"what": names.geom(g), "reason": f"unsupported geom type {kind}"})
    return False


def _shapes(model: Any, g: int) -> list[tuple[str, np.ndarray, np.ndarray]]:
    """URDF primitives for geom ``g``: (geometry xml, pos, rot) in its body
    frame."""
    kind = _KINDS[int(model.geom_type[g])]
    size = model.geom_size[g]
    pos = model.geom_pos[g].copy()
    rot = _quat_to_mat(model.geom_quat[g])
    if kind == "box":
        return [(f'<box size="{_f(2 * size[0])} {_f(2 * size[1])} {_f(2 * size[2])}"/>', pos, rot)]
    if kind == "sphere":
        return [(f'<sphere radius="{_f(size[0])}"/>', pos, rot)]
    if kind == "cylinder":
        return [(f'<cylinder radius="{_f(size[0])}" length="{_f(2 * size[1])}"/>', pos, rot)]
    if kind == "capsule":
        r, half = size[0], size[1]
        out = [(f'<cylinder radius="{_f(r)}" length="{_f(2 * half)}"/>', pos, rot)]
        for sign in (1, -1):
            out.append((f'<sphere radius="{_f(r)}"/>', pos + sign * half * rot[:, 2], rot))
        return out
    # plane: a slab whose top is the plane
    hx = size[0] if size[0] > 0 else FLOOR_EXTENT
    hy = size[1] if size[1] > 0 else FLOOR_EXTENT
    slab = pos - rot[:, 2] * FLOOR_THICKNESS / 2
    return [(f'<box size="{_f(2 * hx)} {_f(2 * hy)} {_f(FLOOR_THICKNESS)}"/>', slab, rot)]


def _box_world(model: Any, data: Any, g: int) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """A box-like geom's (center, rotation, half-sizes) in the world, or None."""
    kind = _KINDS.get(int(model.geom_type[g]))
    pos, rot = data.geom_xpos[g].copy(), data.geom_xmat[g].reshape(3, 3).copy()
    size = model.geom_size[g]
    if kind == "box":
        return pos, rot, size.copy()
    if kind == "plane":
        hx = size[0] if size[0] > 0 else FLOOR_EXTENT
        hy = size[1] if size[1] > 0 else FLOOR_EXTENT
        return pos - rot[:, 2] * FLOOR_THICKNESS / 2, rot, np.array([hx, hy, FLOOR_THICKNESS / 2])
    return None


def _face(rot: np.ndarray, half: np.ndarray, center: np.ndarray, up: bool):
    """The box face whose outward normal is world +z (``up``) or -z, as
    (world corners counter-clockwise seen from that side's outside, height),
    or None if no face is horizontal."""
    want = 1.0 if up else -1.0
    axis = int(np.argmax(np.abs(rot[2])))
    cos = rot[2, axis]
    if abs(cos) < UP_TOL:
        return None
    sign = want * math.copysign(1.0, cos)
    normal_local = np.zeros(3)
    normal_local[axis] = sign
    u, v = [a for a in range(3) if a != axis]
    face_center = center + rot @ (normal_local * half[axis])
    corners = []
    for su, sv in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        corners.append(face_center + rot[:, u] * su * half[u] + rot[:, v] * sv * half[v])
    corners = np.array(corners)
    if _signed_area(corners[:, :2]) < 0:
        corners = corners[::-1]
    return corners, float(face_center[2])


def _signed_area(poly: np.ndarray) -> float:
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def _clip(subject: np.ndarray, clipper: np.ndarray) -> np.ndarray:
    """Sutherland-Hodgman: convex ``subject`` clipped by convex ``clipper``,
    both 2D and counter-clockwise."""
    out = [p for p in subject]
    n = len(clipper)
    for i in range(n):
        a, b = clipper[i], clipper[(i + 1) % n]
        edge = b - a

        def inside(p: np.ndarray, a: np.ndarray = a, edge: np.ndarray = edge) -> bool:
            return edge[0] * (p[1] - a[1]) - edge[1] * (p[0] - a[0]) >= -1e-12

        polygon, out = out, []
        for k, p in enumerate(polygon):
            q = polygon[k - 1]
            if inside(p):
                if not inside(q):
                    out.append(_intersect(q, p, a, b))
                out.append(p)
            elif inside(q):
                out.append(_intersect(q, p, a, b))
        if not out:
            return np.zeros((0, 2))
    return np.array(out)


def _intersect(p: np.ndarray, q: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    d1, d2 = q - p, b - a
    denom = d1[0] * d2[1] - d1[1] * d2[0]
    t = ((a[0] - p[0]) * d2[1] - (a[1] - p[1]) * d2[0]) / denom
    return p + t * d1


def _region_support(
    model: Any, data: Any, names: _Names, region: str, geoms: list[int], links: dict[int, str]
) -> dict[str, Any]:
    """The face that supports ``region``: a horizontal upward face of the
    region site's own body if one fits, else of its fixture, else of any
    static body (the table region's 4 m footprint covers the fixtures on
    the table too: they must not win over the table)."""
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, region)
    info: dict[str, Any] = {"region": region, "points": None}
    if site < 0:
        info["reason"] = "no such site"
        return info
    center = data.site_xpos[site].copy()
    rot = data.site_xmat[site].reshape(3, 3).copy()
    size = model.site_size[site].copy()
    info["site"] = {
        "pos": center.tolist(),
        "quat": _mat_to_quat(rot).tolist(),
        "size": size.tolist(),
        "body": names.body(model.site_bodyid[site]),
    }
    # LIBERO's SiteObject.in_box: a world axis-aligned box, half-extents
    # |R| size, that an object's center must be inside (z: 2 cm lower and
    # 5 cm higher). That box, not the site's own frame, is the region.
    extent = np.abs(rot) @ size
    info["box"] = {
        "center": center.tolist(),
        "half_extents": extent.tolist(),
        "z_range": [float(center[2] - extent[2] - 0.02), float(center[2] + extent[2] + 0.05)],
    }
    if extent[2] > min(extent[0], extent[1]):
        # Thinner across than it is tall: a slab along a wall (the storage
        # box's left/right sides), not a surface to place on.
        info["reason"] = "region is a vertical slab, not a horizontal footprint"
        return info
    footprint = np.array(
        [center[:2] + np.array([su * extent[0], sv * extent[1]])
         for su, sv in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
    )
    site_body = int(model.site_bodyid[site])
    tiers = (
        [g for g in geoms if model.geom_bodyid[g] == site_body],
        [g for g in geoms if _root(model, model.geom_bodyid[g]) == _root(model, site_body)],
        geoms,
    )
    for tier in tiers:
        candidates = _supports(model, data, tier, footprint, center[2])
        if candidates:
            break
    else:
        info["reason"] = "no horizontal support under the region"
        return info
    pick = max(candidates, key=lambda c: c[1])  # the highest
    _gap, height, g, overlap = pick
    b = int(model.geom_bodyid[g])
    world = np.column_stack([overlap, np.full(len(overlap), height)])
    body_pos, body_rot = data.xpos[b], data.xmat[b].reshape(3, 3)
    info.update(
        support_geom=names.geom(g),
        link=links[b],
        height=height,
        gap=float(height - center[2]),
        area=_signed_area(overlap),
        footprint_area=_signed_area(footprint),
        points=world.tolist(),
        points_link=((world - body_pos) @ body_rot).tolist(),
    )
    return info


def _supports(
    model: Any, data: Any, geoms: list[int], footprint: np.ndarray, z: float
) -> list[tuple[float, float, int, np.ndarray]]:
    """Horizontal upward faces among ``geoms`` near height ``z`` (a little
    above: the table regions are 4 mm under the top; or below: a
    compartment's floor) that overlap ``footprint``."""
    out = []
    for g in geoms:
        box = _box_world(model, data, g)
        if box is None:
            continue
        face = _face(box[1], box[2], box[0], up=True)
        if face is None:
            continue
        corners, height = face
        sides = np.linalg.norm(corners[1] - corners[0]), np.linalg.norm(corners[2] - corners[1])
        if min(sides) < MIN_SUPPORT_WIDTH:  # a wall's top edge
            continue
        overlap = _clip(footprint, corners[:, :2])
        enough = MIN_OVERLAP * min(_signed_area(footprint), _signed_area(corners[:, :2]))
        if len(overlap) < 3 or _signed_area(overlap) < enough:
            continue
        gap = height - z
        if -SUPPORT_BELOW <= gap <= SUPPORT_ABOVE:
            out.append((gap, height, g, overlap))
    return out


def _root(model: Any, body: int) -> int:
    """The body's top-level ancestor (a child of the world)."""
    while body and model.body_parentid[body] != 0:
        body = int(model.body_parentid[body])
    return int(body)


def _bottom_face(
    model: Any, data: Any, geoms: list[int], root_pos: np.ndarray, root_rot: np.ndarray
) -> tuple[np.ndarray, float] | None:
    """The object's resting face: the face of its bounding box (in its own
    frame) whose outward normal points most nearly down, as corners in the
    root body frame ordered so the normal (right hand) is outward, and that
    normal's tilt from straight down (rad)."""
    corners = []
    for g in geoms:
        pos = root_rot.T @ (data.geom_xpos[g] - root_pos)
        rot = root_rot.T @ data.geom_xmat[g].reshape(3, 3)
        for signs in np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3):
            corners.append(pos + rot @ (signs * _half_box(model, g)))
    if not corners:
        return None
    lo, hi = np.min(corners, axis=0), np.max(corners, axis=0)
    down = root_rot.T @ np.array([0.0, 0.0, -1.0])
    axis = int(np.argmax(np.abs(down)))
    sign = math.copysign(1.0, down[axis])
    normal = np.zeros(3)
    normal[axis] = sign
    u, v = [a for a in range(3) if a != axis]
    face = []
    for su, sv in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        p = np.where(sign > 0, hi, lo).astype(float).copy()
        p[u] = hi[u] if su > 0 else lo[u]
        p[v] = hi[v] if sv > 0 else lo[v]
        face.append(p)
    face = np.array(face)
    if np.dot(np.cross(face[1] - face[0], face[2] - face[0]), normal) < 0:
        face = face[::-1]
    tilt = math.acos(max(-1.0, min(1.0, float(np.dot(root_rot @ normal, [0, 0, -1.0])))))
    return face, tilt


def _half_box(model: Any, g: int) -> np.ndarray:
    """Half-extents of the box bounding geom ``g`` in its own frame."""
    kind, size = _KINDS[int(model.geom_type[g])], model.geom_size[g]
    if kind == "box":
        return size.copy()
    if kind == "sphere":
        return np.full(3, size[0])
    if kind == "cylinder":
        return np.array([size[0], size[0], size[1]])
    if kind == "capsule":
        return np.array([size[0], size[0], size[1] + size[0]])
    return np.array([size[0], size[1], 0.0])


def _contacts(data: Any, names: _Names, geoms: set[int]) -> list[dict[str, Any]]:
    """MuJoCo's contacts among ``geoms``: per geom pair, the deepest."""
    pairs: dict[tuple[int, int], float] = {}
    for c in data.contact[: data.ncon]:
        if c.geom1 in geoms and c.geom2 in geoms:
            key = (min(c.geom1, c.geom2), max(c.geom1, c.geom2))
            pairs[key] = min(pairs.get(key, np.inf), float(c.dist))
    return [
        {"geom1": names.geom(a), "geom2": names.geom(b), "dist": dist}
        for (a, b), dist in sorted(pairs.items())
    ]


def _joint_value(model: Any, data: Any, joint: str) -> Any:
    j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    adr = model.jnt_qposadr[j]
    width = {0: 7, 1: 4}.get(int(model.jnt_type[j]), 1)
    value = data.qpos[adr : adr + width]
    return float(value[0]) if width == 1 else value.tolist()


# -- XML --------------------------------------------------------------------


def _robot_xml(name: str, body: list[str], srdf: bool = False) -> str:
    kind = "SRDF" if srdf else "URDF"
    head = (
        '<?xml version="1.0"?>\n'
        f"<!-- GENERATED by script/robocerebra/scene_export.py from the compiled "
        f"MuJoCo model ({kind}). -->\n"
    )
    return head + f'<robot name="{name}">\n' + "\n".join(body) + "\n</robot>\n"


def _link_xml(model: Any, names: _Names, link: str, geoms: list[int]) -> str:
    parts = [f'  <link name="{link}">']
    for g in geoms:
        rgba = model.geom_rgba[g]
        color = f"{_f(rgba[0])} {_f(rgba[1])} {_f(rgba[2])} 1"
        for k, (geometry, pos, rot) in enumerate(_shapes(model, g)):
            origin = _origin(pos, rot)
            gname = _sanitize(names.geom(g)) + (f"_{k}" if k else "")
            parts.append(
                f'    <visual name="{gname}">\n      {origin}\n'
                f"      <geometry>{geometry}</geometry>\n"
                f'      <material name="{gname}_mat"><color rgba="{color}"/></material>\n'
                f"    </visual>\n"
                f'    <collision name="{gname}">\n      {origin}\n'
                f"      <geometry>{geometry}</geometry>\n    </collision>"
            )
    parts.append("  </link>")
    return "\n".join(parts)


def _record_shapes(
    model: Any,
    data: Any,
    names: _Names,
    link: str,
    body: int,
    geoms: list[int],
    out: dict[str, dict[str, Any]],
) -> None:
    """The world pose of each collision shape :func:`_link_xml` writes for
    ``geoms``, keyed as pinocchio names them (``<link>_<i>``, in order)."""
    pos_b, rot_b = data.xpos[body], data.xmat[body].reshape(3, 3)
    i = 0
    for g in geoms:
        for _geometry, pos, rot in _shapes(model, g):
            out[f"{link}_{i}"] = {
                "geom": names.geom(g),
                "pose": _xyzquat(pos_b + rot_b @ pos, rot_b @ rot),
            }
            i += 1


def _contact_xml(name: str, link: str, points: np.ndarray) -> str:
    pts = "\n".join("      " + " ".join(_f(x) for x in p) for p in points)
    shape = " ".join(str(i) for i in range(len(points)))
    return (
        f'  <contact name="{name}">\n    <link name="{link}"/>\n'
        f"    <point>\n{pts}\n    </point>\n"
        f"    <shape>{len(points)} {shape}</shape>\n  </contact>"
    )


def _origin(pos: np.ndarray, rot: np.ndarray) -> str:
    rpy = _mat_to_rpy(rot)
    return (
        f'<origin xyz="{" ".join(_f(x) for x in pos)}" '
        f'rpy="{" ".join(_f(x) for x in rpy)}"/>'
    )


def _sanitize(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]", "_", name)


def _f(x: float) -> str:
    text = f"{float(x):.9g}"
    return "0" if text == "-0" else text


# -- rotations (MuJoCo quaternions are w x y z) -----------------------------


def _quat_to_mat(quat: np.ndarray) -> np.ndarray:
    mat = np.zeros(9)
    mujoco.mju_quat2Mat(mat, np.asarray(quat, dtype=float))
    return mat.reshape(3, 3)


def _mat_to_quat(rot: np.ndarray) -> np.ndarray:
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.asarray(rot, dtype=float).reshape(9))
    return quat


def _xyzquat(pos: np.ndarray, rot: np.ndarray) -> list[float]:
    """long_tamp's pose convention: x y z qx qy qz qw."""
    w, x, y, z = _mat_to_quat(rot)
    return [float(v) for v in (*pos, x, y, z, w)]


def _mat_to_rpy(rot: np.ndarray) -> np.ndarray:
    """URDF roll-pitch-yaw: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    if abs(rot[2, 0]) < 1 - 1e-9:
        pitch = -math.asin(rot[2, 0])
        roll = math.atan2(rot[2, 1], rot[2, 2])
        yaw = math.atan2(rot[1, 0], rot[0, 0])
    else:  # gimbal lock: fold the roll into the yaw
        pitch = math.copysign(math.pi / 2, -rot[2, 0])
        roll = 0.0
        yaw = math.atan2(-rot[0, 1], rot[1, 1])
    return np.array([roll, pitch, yaw])
