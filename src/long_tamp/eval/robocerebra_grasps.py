"""Grasp handles for RoboCerebra's exported object scenes (M11 R3, #158).

RoboCerebra object models have collision geometry but no SRDF handles. This
module derives object-relative handles from exported collision primitives and
checks the open-hand approach against the scene. When clutter blocks every
candidate, it records an object-valid fallback that still requires scene-level
HPP planning before execution.
"""

from __future__ import annotations

import copy
import itertools
import json
import re
from collections.abc import Iterable
from pathlib import Path
from typing import Any

import numpy as np

from long_tamp.grasping import (
    PANDA_HAND,
    Box,
    GraspPlanner,
    GraspPlannerParams,
    GraspableObject,
    load_urdf_primitives,
    pose_to_xyzquat,
    xyzquat_to_pose,
)
from long_tamp.grasping.geometry import inverse

from .robocerebra_scene import ExportedScene


def object_class(instance: str) -> str:
    """Return the LIBERO class from an instance name such as ``plate_1``."""
    match = re.fullmatch(r"(.+)_\d+", instance)
    if not match:
        raise ValueError(
            f"RoboCerebra object name lacks an instance suffix: {instance!r}"
        )
    return match.group(1)


def add_grasp_handles(
    scene: ExportedScene,
    *,
    max_handles: int = 6,
    max_tilted: int = 4,
    planner_params: GraspPlannerParams | None = None,
    objects: Iterable[str] | None = None,
) -> dict[str, Any]:
    """Generate and persist collision-free Panda handles for every object
    (or only ``objects``; the others' handles are kept).

    The object geometry and obstacle geometry are read from the exported
    URDFs. The planner samples grasps and top-down pinches on thin horizontal
    rims or flanges.

    Up to ``max_tilted`` handles more follow: the top-down grasps pitched
    about their closing axis (:data:`TILTS`), the pads on the same contacts.
    By an object close to the robot's base, a top-down grasp folds the elbow
    to its stop; leaning the hand toward the robot unfolds it.

    The returned and persisted report contains a per-instance row with its
    class and generated handles. Raises if any object has no feasible handle
    so a partially covered benchmark case cannot silently pass.
    """
    if max_handles < 1:
        raise ValueError("max_handles must be positive")

    object_paths = scene.file_paths()["objects"]
    environment_path = scene.root / f"{scene.environment_name}.urdf"
    environment = load_urdf_primitives(environment_path)
    all_object_primitives = {
        name: load_urdf_primitives(paths["urdf"])
        for name, paths in object_paths.items()
    }

    report: dict[str, Any] = {}
    planner_params = planner_params or GraspPlannerParams(
        samples_along=3,
        depths=(0.25, 0.5),
        cylinder_angles=12,
        rays=5,
        approach_clearance=0.04,
        preferred_approach=(0.0, 0.0, -1.0),
    )
    selected = set(object_paths) if objects is None else set(objects)
    unknown = selected - set(object_paths)
    if unknown:
        raise ValueError(f"not in the scene: {sorted(unknown)}")
    for name, paths in sorted(object_paths.items()):
        if name not in selected:
            continue
        obj = GraspableObject.from_urdf(name, paths["urdf"], paths["srdf"])
        if not obj.primitives:
            raise ValueError(f"{name} has no supported collision primitives")

        target_pose = xyzquat_to_pose(scene.object_config(name))
        world_down_in_object = target_pose[:3, :3].T @ np.array([0.0, 0.0, -1.0])

        obstacle_primitives = [*environment]
        for other, primitives in all_object_primitives.items():
            if other != name:
                obstacle_primitives.extend(primitives)
        to_object = inverse(target_pose)
        obstacles = []
        for primitive in obstacle_primitives:
            obstacle = copy.copy(primitive)
            obstacle.pose = to_object @ primitive.pose
            obstacles.append(obstacle)

        candidates_by_pose: dict[tuple[float, ...], Any] = {}
        params = copy.copy(planner_params)
        params.preferred_approach = tuple(float(v) for v in world_down_in_object)
        planner = GraspPlanner(PANDA_HAND, params)
        samples = list(
            itertools.islice(
                planner._samples(obj.primitives), max(400, max_handles * 40)
            )
        )
        samples.extend(_rim_samples(planner, obj.primitives))
        for checked_obstacles in (obstacles, []):
            for grasp_pose, source in samples:
                candidate = _accept(
                    planner,
                    obj,
                    grasp_pose,
                    source,
                    checked_obstacles,
                    world_down_in_object,
                )
                if candidate is not None:
                    candidate.requires_scene_planning = (
                        checked_obstacles == [] and bool(obstacles)
                    )
                    _keep(candidates_by_pose, candidate)
            if candidates_by_pose:
                break

        candidates = list(candidates_by_pose.values())
        candidates.sort(
            key=lambda c: (
                float(np.dot(c.approach, world_down_in_object)),
                c.score,
                c.evaluation.contact_ratio,
            ),
            reverse=True,
        )
        candidates = candidates[:max_handles]
        if not candidates:
            raise ValueError(
                f"{name} ({object_class(name)}) has no feasible grasp handle"
            )
        # Tilted variants of the top-down grasps go after them, leaning
        # toward the robot first: tried only when no top-down grasp plans
        # or OSC can track it.
        checked = [] if candidates[0].requires_scene_planning else obstacles
        tilted: dict[tuple[float, ...], Any] = {}
        for candidate in candidates:
            if float(candidate.approach @ world_down_in_object) < TOP_DOWN:
                continue
            for grasp_pose in _tilted(planner, candidate):
                found = _accept(
                    planner,
                    obj,
                    grasp_pose,
                    f"{candidate.source} tilted",
                    checked,
                    world_down_in_object,
                )
                if found is not None:
                    found.requires_scene_planning = candidate.requires_scene_planning
                    _keep(tilted, found)
        to_robot = to_object[:3, :3] @ _horizontal(
            np.asarray(scene.robot_pose()[:3]) - target_pose[:3, 3]
        )
        candidates.extend(
            sorted(
                tilted.values(),
                key=lambda c: (float(c.approach @ to_robot), c.score),
                reverse=True,
            )[:max_tilted]
        )

        handles_xml = []
        rows = []
        for index, candidate in enumerate(candidates):
            handle_name = f"grasp_{index}"
            handles_xml.append(
                candidate.srdf_handle(
                    handle_name,
                    link="base_link",
                    clearance=0.05,
                    comment=(
                        f"RoboCerebra {object_class(name)}; generated {candidate.source}; "
                        f"score {candidate.score:.3f}; approach aligned with resting pose"
                    ),
                )
            )
            rows.append(
                {
                    "name": handle_name,
                    "source": candidate.source,
                    "requires_scene_planning": bool(
                        getattr(candidate, "requires_scene_planning", False)
                    ),
                    "score": round(float(candidate.score), 5),
                    "approach": [round(float(v), 5) for v in candidate.approach],
                    "evaluation": candidate.evaluation.to_dict(),
                    "handle_xyzquat": [
                        round(float(v), 7)
                        for v in pose_to_xyzquat(candidate.handle_pose)
                    ],
                }
            )

        write_handles(paths["srdf"], handles_xml)
        report[name] = {"class": object_class(name), "handles": rows}

    scene.manifest["grasp_handles"] = {
        **scene.manifest.get("grasp_handles", {}),
        **report,
    }
    (scene.root / "manifest.json").write_text(json.dumps(scene.manifest, indent=1))
    return report


def write_handles(srdf_path: str | Path, handles_xml: Iterable[str]) -> None:
    """Replace the generated handle section of an object SRDF with
    ``handles_xml`` (SRDF ``<handle>`` elements, one string each)."""
    srdf_path = Path(srdf_path)
    srdf = srdf_path.read_text()
    if "</robot>" not in srdf:
        raise ValueError(f"{srdf_path} has no closing robot tag")
    # Idempotent generation: drop only the previous generated section.
    start = srdf.find(_SECTION_START)
    if start >= 0:
        end = srdf.find(_SECTION_END, start)
        if end < 0:
            raise ValueError(f"{srdf_path} has an unterminated generated section")
        srdf = srdf[:start] + srdf[end + len(_SECTION_END) :]
    section = _SECTION_START + "\n" + "\n".join(handles_xml) + _SECTION_END + "\n"
    srdf_path.write_text(srdf.replace("</robot>", section + "</robot>", 1))


_SECTION_START = "  <!-- RoboCerebra grasp handles: generated by long_tamp -->"
_SECTION_END = "  <!-- End RoboCerebra grasp handles -->"


def _accept(planner, obj, grasp_pose, source, obstacles, down):
    """The candidate grasp of ``obj`` at ``grasp_pose``, scored toward the
    world's ``down`` (object frame), or ``None`` if the open hand or the
    palm hits ``obstacles`` or the object."""
    ev = planner.evaluate(grasp_pose, obj.primitives, obstacles, full=True)
    if source.startswith("top-rim") and not ev.feasible:
        # The analytic rim frame is centered on the wall's top
        # face; shift by measured pad asymmetry before checking.
        grasp_pose = grasp_pose.copy()
        grasp_pose[:3, 3] += grasp_pose[:3, 1] * ev.offset
        ev = planner.evaluate(grasp_pose, obj.primitives, obstacles, full=True)
    # Closed finger contact with the target is intentional. The
    # open-hand approach and palm/knuckles must stay clear.
    bad = [
        label
        for label in ev.collisions
        if not (
            label.startswith("closed finger_")
            and any(f":{p.name}" in label for p in obj.primitives)
        )
    ]
    if bad or any(not r.startswith("collision:") for r in ev.reasons):
        return None
    ev.reasons = []
    ev.feasible = True
    score = max(0.0, float(grasp_pose[:3, 0] @ down)) + ev.contact_ratio
    return _candidate(planner, grasp_pose, ev, source, score)


def _keep(found, candidate):
    """Keep ``candidate`` in ``found`` unless a better one has its handle."""
    key = tuple(np.round(candidate.handle_pose.ravel(), 5))
    current = found.get(key)
    if current is None or candidate.score > current.score:
        found[key] = candidate


def _candidate(planner, grasp_pose, evaluation, source, score):
    """Build a candidate while retaining the evaluator's contact evidence."""
    from long_tamp.grasping.planner import GraspCandidate

    return GraspCandidate(
        pose=grasp_pose,
        evaluation=evaluation,
        score=score,
        source=source,
        handle_pose=planner.handle_from_canonical(grasp_pose),
    )


# A bowl's rim is a ring of thin wall boxes leaning out ~15 degrees: their
# thickness axis tilts as much from horizontal (RoboCerebra's bowls).
RIM_TILT = 0.35  # sin of the largest tilt of a rim's closing axis, ~20 degrees


#: Pitches of a top-down grasp about its closing axis, both ways [rad]
#: (#158). From Ideal/case1's start posture, OSC tracks no top-down grasp of
#: an object 21 cm in front of the base; leaning 30 degrees or more toward
#: the robot, it does.
TILTS = (np.radians(30.0), np.radians(45.0))
#: A grasp is top-down when its approach is within ~25 degrees of down.
TOP_DOWN = 0.9


def _tilted(planner, candidate):
    """``candidate``'s grasp pitched by each of :data:`TILTS`, both ways,
    about its closing axis through the pads' centre."""
    pose = candidate.pose
    closing = pose[:3, 1]
    pad_x = planner.gripper.pad_x_for_width(candidate.evaluation.width)
    pads = pose[:3, 3] + pad_x * pose[:3, 0]
    for tilt in TILTS:
        for angle in (tilt, -tilt):
            out = np.eye(4)
            out[:3, :3] = _rotation(closing, angle) @ pose[:3, :3]
            out[:3, 3] = pads - pad_x * out[:3, 0]
            yield out


def _rotation(axis, angle):
    """Rotation by ``angle`` about the unit ``axis`` (Rodrigues)."""
    x, y, z = axis
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * K @ K


def _horizontal(v):
    """``v`` with no vertical part, unit length (zero if vertical)."""
    v = np.array([v[0], v[1], 0.0])
    norm = np.linalg.norm(v)
    return v / norm if norm > 1e-9 else v


def _rim_samples(planner, primitives):
    """Sample top-down pinches, both ways round, on thin box rims and
    flanges, horizontal or leaning (a bowl's wall), sunk down the wall
    first and then flush with its top."""
    for primitive in primitives:
        if not isinstance(primitive, Box):
            continue
        half = primitive.half_extents()
        rotation = primitive.pose[:3, :3]
        up = int(np.argmax(np.abs(rotation[2])))
        rim_up = rotation[:, up] * np.sign(rotation[2, up])
        top_center = primitive.pose[:3, 3] + rim_up * half[up]
        # Flush, the pads overhang the rim by half their length; sunk by
        # that half down a tall enough wall, they pinch it over their whole
        # length (a 2 mm bowl wall slips out of a half-pad pinch, #158).
        sink = planner.gripper.pad_length / 2
        centers = [top_center]
        if 2 * half[up] >= 2 * sink:
            centers.insert(0, top_center - rim_up * sink)
        for axis in range(3):
            closing = rotation[:, axis]
            if axis == up or abs(closing[2]) > RIM_TILT:
                continue
            # The pads close horizontally: across a leaning wall, wider.
            width = 2 * half[axis] / np.sqrt(1.0 - closing[2] ** 2)
            if width <= 0 or width > planner.gripper.max_width - 0.01:
                continue
            for center in centers:
                yield from planner._emit(
                    (0.0, 0.0, -1.0), closing, center, width, "top-rim"
                )


__all__ = ["add_grasp_handles", "object_class", "write_handles"]
