"""Graspable objects and the per-(gripper, handle) finger closure table.

:class:`GraspableObject` is an object as the grasp planner sees it: its
collision primitives and its SRDF handles, all in the root-link frame.

:class:`FingerClosureTable` is what an executor needs from the grasp planner
at run time: for every ``(gripper, handle)`` pair a task may grasp, the
finger joint values that close the gripper on that handle. It is built once
from a task's YAML (the same file the motion planner loads) and queried by
the viewer, a trajectory exporter or a gripper controller when a grasp phase
ends.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from long_tamp.logging import get_logger

from .geometry import Primitive, link_poses, load_srdf_frames, load_urdf_primitives
from .gripper import ParallelGripperModel
from .planner import GraspCandidate, GraspEvaluation, GraspPlanner, GraspPlannerParams

logger = get_logger("grasping.objects")


@dataclass
class GraspableObject:
    """An object's collision primitives and handles (root-link frame)."""

    name: str
    primitives: list[Primitive]
    handles: dict[str, np.ndarray] = field(default_factory=dict)
    urdf: Path | None = None

    @classmethod
    def from_urdf(
        cls, name: str, urdf: str | Path, srdf: str | Path | None = None
    ) -> GraspableObject:
        urdf = Path(urdf)
        prims = load_urdf_primitives(urdf)
        handles: dict[str, np.ndarray] = {}
        if srdf is not None and Path(srdf).exists():
            poses = link_poses(ET.parse(urdf).getroot())
            for frame in load_srdf_frames(srdf).values():
                if frame.kind == "handle":
                    handles[frame.name] = poses.get(frame.link, np.eye(4)) @ frame.pose
        return cls(name=name, primitives=prims, handles=handles, urdf=urdf)

    def handle_pose(self, handle: str) -> np.ndarray:
        """Pose of ``handle`` (``"h"`` or ``"object/h"``) in the object frame."""
        local = (
            handle.split("/", 1)[1] if handle.startswith(f"{self.name}/") else handle
        )
        try:
            return self.handles[local]
        except KeyError as error:
            raise KeyError(f"{self.name} has no handle {local!r}") from error


@dataclass(frozen=True)
class GripperBinding:
    """A robot's ``<gripper>`` bound to the model of its physical hand.

    ``gripper`` is the HPP gripper name (``"ur10_left/gripper"``) and
    ``prefix`` the robot name its finger joints carry (``"ur10_left"``).
    """

    gripper: str
    model: ParallelGripperModel
    prefix: str = ""

    @property
    def joint_prefix(self) -> str:
        return self.prefix or self.gripper.split("/", 1)[0]


class FingerClosureTable:
    """Finger closures for every grasp a task can make with a real hand.

    Grippers without a binding (a tool tip, a jig clamp: virtual grippers
    with no fingers) are simply not in the table.
    """

    def __init__(
        self,
        bindings: Sequence[GripperBinding],
        objects: Mapping[str, GraspableObject],
        valid_pairs: Mapping[str, Sequence[str]] | None = None,
        params: GraspPlannerParams | None = None,
    ) -> None:
        self.bindings = {b.gripper: b for b in bindings}
        self.objects = dict(objects)
        self.planners = {
            g: GraspPlanner(b.model, params) for g, b in self.bindings.items()
        }
        self._cache: dict[tuple[str, str], GraspEvaluation] = {}
        for gripper, handles in (valid_pairs or {}).items():
            if gripper in self.bindings:
                for handle in handles:
                    self.evaluation(gripper, handle)

    @classmethod
    def from_task_yaml(
        cls,
        config_path: str | Path,
        models: Mapping[str, ParallelGripperModel],
        params: GraspPlannerParams | None = None,
    ) -> FingerClosureTable:
        """Build from a task YAML. ``models`` maps an HPP gripper name
        (``"ur10_left/gripper"``) to its hand model."""
        from long_tamp.config.yaml_loader import YamlTaskLoader

        loader = YamlTaskLoader(config_path)
        objects = {
            name: GraspableObject.from_urdf(name, paths["urdf"], paths.get("srdf"))
            for name, paths in loader.file_paths.get("objects", {}).items()
        }
        bindings = [GripperBinding(g, m) for g, m in models.items()]
        return cls(bindings, objects, loader.task_config.VALID_PAIRS, params)

    def has(self, gripper: str | None) -> bool:
        return gripper in self.bindings

    def pairs(self) -> list[tuple[str, str]]:
        """The (gripper, handle) pairs evaluated so far (the task's valid
        pairs with a hand, once built from a task YAML)."""
        return sorted(self._cache)

    def evaluation(self, gripper: str, handle: str) -> GraspEvaluation:
        """Close ``gripper`` on ``handle`` (``"object/handle"``), cached."""
        key = (gripper, handle)
        if key not in self._cache:
            obj = self.objects[handle.split("/", 1)[0]]
            ev = self.planners[gripper].evaluate_handle(
                obj.handle_pose(handle), obj.primitives
            )
            if not ev.feasible:
                logger.warning(
                    "grasp %s > %s: %s", gripper, handle, "; ".join(ev.reasons)
                )
            self._cache[key] = ev
        return self._cache[key]

    def closed_values(self, gripper: str, handle: str) -> dict[str, float]:
        """Finger joint values (prefixed names) holding ``handle``."""
        b = self.bindings[gripper]
        ev = self.evaluation(gripper, handle)
        q = ev.q if np.isfinite(ev.q) else b.model.q_open
        return b.model.joint_values(q, b.joint_prefix)

    def open_values(self, gripper: str) -> dict[str, float]:
        b = self.bindings[gripper]
        return b.model.open_joint_values(b.joint_prefix)

    def plan(self, gripper: str, object_name: str, **kwargs) -> list[GraspCandidate]:
        """Ranked new grasps of ``object_name`` for ``gripper``'s hand."""
        return self.planners[gripper].plan(
            self.objects[object_name].primitives, **kwargs
        )

    def report(self) -> list[dict]:
        rows = []
        for (gripper, handle), ev in sorted(self._cache.items()):
            rows.append({"gripper": gripper, "handle": handle, **ev.to_dict()})
        return rows


def apply_joint_values(
    q, rank: Mapping[str, int], values: Mapping[str, float]
) -> np.ndarray:
    """Copy of configuration ``q`` with ``values`` written at their ranks."""
    out = np.array(q, dtype=float, copy=True)
    for joint, value in values.items():
        out[rank[joint]] = value
    return out
