"""Export a task's planning scene to MuJoCo (MJCF).

``export_mjcf(config, out_dir)`` reads the same YAML task config the planner
loads and writes one self-contained MJCF file:

- every robot, environment and object URDF is imported with MuJoCo's own URDF
  parser and attached under its scene name, so bodies and joints keep HPP's
  names (``ur10_left/shoulder_pan_joint``, ``part1/base_link``);
- robots and environments are fixed at their configured pose (identity unless
  the config gives ``pose``); objects get a free joint at their
  ``initial_pose_xyzquat``;
- ``<mimic>`` joints become joint equality constraints (MuJoCo's URDF parser
  drops them before 3.14), and lose their own limits, which can contradict the
  mimic (the leader's limits govern them);
- meshes are resolved like the planner resolves them
  (``backends._urdf_paths``), converted to STL when MuJoCo can't read the
  format (COLLADA ``.dae``, via trimesh), and copied next to the MJCF under
  ``meshes/``;
- a link named ``world`` becomes ``world_link`` (MuJoCo reserves the name
  and would otherwise drop every transform above it);
- joint velocity limits (which MJCF has no field for) are kept as custom
  numerics ``velocity_limit:<joint>``;
- visual geometry is kept, in geom group 1 and without contacts, as MuJoCo's
  URDF import does.

Needs the ``sim`` extra (``pip install long-tamp[sim]``). Kinematics match HPP
by construction; ``fk_mismatch`` measures it (see ``tests/test_sim_mjcf.py``).
"""

from __future__ import annotations

import hashlib
import shutil
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from long_tamp.backends._urdf_paths import resolve_mesh_path
from long_tamp.config.yaml_loader import YamlTaskLoader

#: Mesh formats MuJoCo reads directly; anything else is converted to STL.
NATIVE_MESH_FORMATS = {".stl", ".obj", ".msh"}

#: Custom numeric holding a joint's URDF velocity limit: prefix + joint name.
VELOCITY_LIMIT_PREFIX = "velocity_limit:"

#: What a URDF link named "world" is called in MJCF (MuJoCo reserves the name).
WORLD_LINK_RENAME = "world_link"

_COMPILER = (
    '<mujoco><compiler discardvisual="false" fusestatic="false" '
    'strippath="false"/></mujoco>'
)


@dataclass
class MjcfExport:
    """Where the MJCF was written and what it contains."""

    path: Path
    #: Scene element -> "robot" | "environment" | "object".
    kinds: dict[str, str] = field(default_factory=dict)
    #: Object name -> its free joint's name.
    free_joints: dict[str, str] = field(default_factory=dict)
    #: (joint, joint it mimics, multiplier, offset), with scene prefixes.
    mimics: list[tuple[str, str, float, float]] = field(default_factory=list)

    def load(self):
        """The compiled ``mujoco.MjModel``."""
        import mujoco

        return mujoco.MjModel.from_xml_path(str(self.path))


def _mujoco():
    try:
        import mujoco
    except ImportError as error:  # pragma: no cover - depends on the extra
        raise ImportError(
            "long_tamp.sim needs MuJoCo: pip install long-tamp[sim]"
        ) from error
    return mujoco


def _mesh_file(source: Path, mesh_dir: Path) -> Path:
    """``source`` as a mesh MuJoCo reads, written under ``mesh_dir`` once."""
    digest = hashlib.sha1(str(source).encode()).hexdigest()[:8]
    ext = source.suffix.lower()
    target_ext = ext if ext in NATIVE_MESH_FORMATS else ".stl"
    target = mesh_dir / f"{source.stem}-{digest}{target_ext}"
    if target.exists():
        return target
    mesh_dir.mkdir(parents=True, exist_ok=True)
    if ext in NATIVE_MESH_FORMATS:
        shutil.copyfile(source, target)
        return target
    try:
        import trimesh
    except ImportError as error:  # pragma: no cover - depends on the extra
        raise ImportError(
            f"converting {source.name} needs trimesh: pip install long-tamp[sim]"
        ) from error
    trimesh.load(source, force="mesh").export(target)
    return target


def _prepare_urdf(
    urdf: Path, mesh_dir: Path
) -> tuple[str, list[tuple[str, str, float, float]], dict[str, float]]:
    """The URDF text MuJoCo should parse, its mimic joints and its joints'
    velocity limits."""
    tree = ET.parse(urdf)
    root = tree.getroot()
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename", "")
        resolved = Path(resolve_mesh_path(filename, urdf.parent))
        if not resolved.is_file():
            raise FileNotFoundError(f"{urdf.name}: mesh not found: {filename}")
        mesh.set("filename", str(_mesh_file(resolved, mesh_dir).resolve()))
    # MuJoCo's URDF parser takes a link named "world" for its world body and
    # drops every transform above it (e.g. a pedestal the arm stands on).
    for link in root.iter("link"):
        if link.get("name") == "world":
            link.set("name", WORLD_LINK_RENAME)
    for tag in ("parent", "child"):
        for ref in root.iter(tag):
            if ref.get("link") == "world":
                ref.set("link", WORLD_LINK_RENAME)
    velocity_limits = {}
    for joint in root.iter("joint"):
        limit = joint.find("limit")
        if limit is not None and float(limit.get("velocity", 0) or 0) > 0:
            velocity_limits[joint.get("name")] = float(limit.get("velocity"))
    mimics = []
    for joint in root.iter("joint"):
        mimic = joint.find("mimic")
        if mimic is not None:
            mimics.append(
                (
                    joint.get("name"),
                    mimic.get("joint"),
                    float(mimic.get("multiplier", 1.0)),
                    float(mimic.get("offset", 0.0)),
                )
            )
    for old in root.findall("mujoco"):
        root.remove(old)
    root.insert(0, ET.fromstring(_COMPILER))
    return ET.tostring(root, encoding="unicode"), mimics, velocity_limits


def _wxyz(xyzquat: Sequence[float]) -> tuple[list[float], list[float]]:
    x, y, z, qx, qy, qz, qw = (float(v) for v in xyzquat)
    return [x, y, z], [qw, qx, qy, qz]


def export_mjcf(
    config: str | Path,
    out_dir: str | Path,
    *,
    object_poses: Mapping[str, Sequence[float]] | None = None,
    filename: str | None = None,
) -> MjcfExport:
    """Write the scene of the YAML task ``config`` as MJCF under ``out_dir``.

    ``object_poses`` overrides objects' ``initial_pose_xyzquat`` (x, y, z,
    qx, qy, qz, qw). Returns where the file went and what it holds.
    """
    mujoco = _mujoco()
    config = Path(config)
    out_dir = Path(out_dir)
    mesh_dir = out_dir / "meshes"
    paths = YamlTaskLoader(config).file_paths
    with open(config) as fh:
        data: dict[str, Any] = yaml.safe_load(fh)
    poses = {
        name: spec.get("initial_pose_xyzquat")
        for name, spec in (data.get("objects") or {}).items()
    }
    poses.update(object_poses or {})

    world = mujoco.MjSpec()
    world.modelname = data.get("task", config.stem)
    world.compiler.degree = False
    export = MjcfExport(path=out_dir / (filename or f"{world.modelname}.xml"))

    entries: list[tuple[str, str, str, Sequence[float] | None]] = []
    for name, entry in paths.get("robot", {}).items():
        entries.append((name, "robot", entry["urdf"], entry.get("pose")))
    for name, urdf in paths.get("environment", {}).items():
        entries.append((name, "environment", urdf, None))
    for name, entry in paths.get("objects", {}).items():
        pose = poses.get(name)
        if pose is None:
            raise ValueError(f"object {name} has no initial_pose_xyzquat")
        entries.append((name, "object", entry["urdf"], pose))

    for name, kind, urdf, pose in entries:
        text, mimics, velocity_limits = _prepare_urdf(Path(urdf), mesh_dir)
        child = mujoco.MjSpec.from_string(text)
        prefix = f"{name}/"
        if kind == "object":
            roots = [b for b in child.worldbody.bodies]
            if len(roots) != 1:
                raise ValueError(f"object {name}: expected one root link")
            joint = roots[0].add_freejoint()
            joint.name = "root_joint"
            export.free_joints[name] = prefix + "root_joint"
        pos, quat = _wxyz(pose) if pose is not None else ([0, 0, 0], [1, 0, 0, 0])
        frame = world.worldbody.add_frame(pos=pos, quat=quat)
        world.attach(child, frame=frame, prefix=prefix)
        export.kinds[name] = kind
        # MuJoCo has no joint velocity limit; keep the URDF's, for controllers.
        for joint, limit in velocity_limits.items():
            numeric = world.add_numeric()
            numeric.name = f"{VELOCITY_LIMIT_PREFIX}{prefix}{joint}"
            numeric.size = 1
            numeric.data = [limit]
        # Newer MuJoCo (3.14) imports <mimic> itself; older versions drop it.
        imported = {eq.name1 for eq in child.equalities}
        for joint, parent, multiplier, offset in mimics:
            export.mimics.append((prefix + joint, prefix + parent, multiplier, offset))
            if joint in imported or prefix + joint in imported:
                continue
            eq = world.add_equality()
            eq.name = f"{prefix}{joint}_mimic"
            eq.type = mujoco.mjtEq.mjEQ_JOINT
            eq.objtype = mujoco.mjtObj.mjOBJ_JOINT
            eq.name1 = prefix + joint
            eq.name2 = prefix + parent
            eq.data[:5] = [offset, multiplier, 0.0, 0.0, 0.0]

    # A follower's own limits can contradict its mimic (the Robotiq URDF gives
    # the inner fingers [0, 0.88] with multiplier -1; HPP's config overrides
    # them); the leader's limits govern it.
    followers = {joint for joint, _, _, _ in export.mimics}
    for joint in world.joints:
        if joint.name in followers:
            joint.limited = mujoco.mjtLimited.mjLIMITED_FALSE

    world.compile()
    xml = world.to_xml()
    # Mesh files were given by absolute path; point them next to the MJCF so
    # the export folder can move.
    xml = xml.replace(f'file="{mesh_dir.resolve()}/', 'file="meshes/')
    out_dir.mkdir(parents=True, exist_ok=True)
    export.path.write_text(xml)
    return export


class QposMap:
    """Maps configurations of a Pinocchio model (HPP's device) to MuJoCo ``qpos``.

    Joints are matched by name; a Pinocchio free-flyer ``<obj>/root_joint``
    maps to the MuJoCo free joint of the same name (xyz, then the quaternion
    reordered from xyzw to wxyz). MuJoCo joints with no Pinocchio match keep
    their reference position. Built once, then cheap to call (every physics
    step, in the MuJoCo backend).
    """

    def __init__(self, model: Any, pin_model: Any) -> None:
        import mujoco

        self._qpos0 = model.qpos0.copy()
        scalar_src, scalar_dst, free_src, free_dst, planar_src, planar_dst = (
            [] for _ in range(6)
        )
        for jid in range(1, pin_model.njoints):
            mj = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, pin_model.names[jid]
            )
            if mj < 0:
                continue
            joint = pin_model.joints[jid]
            adr = int(model.jnt_qposadr[mj])
            if model.jnt_type[mj] == mujoco.mjtJoint.mjJNT_FREE:
                free_src.append(joint.idx_q)
                free_dst.append(adr)
            elif joint.nq == 1:
                scalar_src.append(joint.idx_q)
                scalar_dst.append(adr)
            elif joint.nq == 2:  # continuous joint: (cos, sin)
                planar_src.append(joint.idx_q)
                planar_dst.append(adr)
        self._scalar = (np.array(scalar_src, int), np.array(scalar_dst, int))
        self._planar = (np.array(planar_src, int), np.array(planar_dst, int))
        # xyz qx qy qz qw -> xyz qw qx qy qz
        order = np.array([0, 1, 2, 6, 3, 4, 5])
        src = [s + order for s in free_src]
        dst = [d + np.arange(7) for d in free_dst]
        self._free = (
            np.concatenate(src) if src else np.zeros(0, int),
            np.concatenate(dst) if dst else np.zeros(0, int),
        )

    def __call__(self, pin_q: Any) -> np.ndarray:
        q = np.asarray(pin_q, dtype=float)
        qpos = self._qpos0.copy()
        qpos[self._scalar[1]] = q[self._scalar[0]]
        qpos[self._free[1]] = q[self._free[0]]
        if len(self._planar[0]):
            qpos[self._planar[1]] = np.arctan2(
                q[self._planar[0] + 1], q[self._planar[0]]
            )
        return qpos


def qpos_from_pinocchio(model: Any, pin_model: Any, pin_q: np.ndarray) -> np.ndarray:
    """MuJoCo ``qpos`` for a configuration ``pin_q`` of ``pin_model`` (see
    :class:`QposMap`, which does the same without rebuilding the map)."""
    return QposMap(model, pin_model)(pin_q)


def fk_mismatch(model: Any, pin_model: Any, pin_q: np.ndarray) -> dict[str, float]:
    """Per-body pose error between MuJoCo and Pinocchio at the same config.

    Returns, for every MuJoCo body whose name is also a Pinocchio body frame,
    ``max(position error [m], orientation error [rad])``.
    """
    import mujoco
    import pinocchio as pin

    data = mujoco.MjData(model)
    data.qpos[:] = qpos_from_pinocchio(model, pin_model, pin_q)
    mujoco.mj_kinematics(model, data)
    pdata = pin_model.createData()
    pin.framesForwardKinematics(pin_model, pdata, pin_q)
    errors: dict[str, float] = {}
    for bid in range(1, model.nbody):
        name = model.body(bid).name
        if not pin_model.existFrame(name, pin.FrameType.BODY):
            continue
        placement = pdata.oMf[pin_model.getFrameId(name, pin.FrameType.BODY)]
        rotation = data.xmat[bid].reshape(3, 3)
        dp = np.linalg.norm(placement.translation - data.xpos[bid])
        dr = np.linalg.norm(pin.log3(placement.rotation.T @ rotation))
        errors[name] = float(max(dp, dr))
    return errors


def main(argv: Sequence[str] | None = None) -> int:
    """``python -m long_tamp.sim.mjcf CONFIG -o OUT``: export and summarize."""
    import argparse

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("config", help="YAML task config (the planner's)")
    ap.add_argument("-o", "--out", default="mjcf", help="output folder")
    args = ap.parse_args(argv)
    export = export_mjcf(args.config, args.out)
    model = export.load()
    print(
        f"wrote {export.path}: {model.nbody - 1} bodies, {model.njnt} joints "
        f"({len(export.free_joints)} free), {model.neq} mimic equalities, "
        f"{model.nmesh} meshes"
    )
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
