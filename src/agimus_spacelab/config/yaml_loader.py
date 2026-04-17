"""YAML-based task configuration loader.

Provides :class:`YamlTaskLoader` which reads a YAML config file and
produces the runtime objects that :class:`ManipulationTask` and
:class:`SceneBuilder` expect:

* ``file_paths``         — ``FILE_PATHS`` dict for ``SceneBuilder``
* ``joint_bounds_class`` — class with ``freeflyer_bounds()`` / ``all_robot_bounds()``
* ``task_config``        — class compatible with ``ManipulationTask.task_config``
* ``build_initial_config(objects)`` — flat joint + pose list

Example usage in a task script::

    from agimus_spacelab.config.yaml_loader import YamlTaskLoader

    _YAML = Path(__file__).parent.parent / "config" / "my_task.yaml"
    _loader = YamlTaskLoader(_YAML)

    class MyTask(ManipulationTask):
        def __init__(self, backend="pyhpp"):
            super().__init__(
                task_name="My Task",
                backend=backend,
                FILE_PATHS=_loader.file_paths,
                joint_bounds=_loader.joint_bounds_class,
            )
            self.task_config = _loader.task_config.with_grasp_goals([
                "ur5/gripper grasps pokeball/handle"
            ])
            self.use_factory = True

        def build_initial_config(self):
            return _loader.build_initial_config(
                objects=self.task_config.OBJECTS
            )
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

from agimus_spacelab.utils.transforms import xyzrpy_to_xyzquat


# ---------------------------------------------------------------------------
# Module-level helpers used as classmethods on the dynamic config class.
# Defined here (not inside _build_task_config) so they can be pickled and
# their __qualname__ is clean.
# ---------------------------------------------------------------------------

def _yaml_get_constraint_defs(cls):
    """YAML configs do not declare explicit constraint defs (factory mode)."""
    return []


def _yaml_init_poses(cls):
    """No-op: YAML configs store poses as literal values, nothing to derive."""


def _yaml_feasible_grasp_goal_states(cls):
    """Enumerate all '<gripper> grasps <handle>' strings from VALID_PAIRS."""
    goals = []
    for gripper, handles in cls.VALID_PAIRS.items():
        for handle in handles:
            goals.append(f"{gripper} grasps {handle}")
    return goals


def _yaml_with_grasp_goals(cls, goal_states):
    """Return a derived config filtered to the requested gripper-handle pairs.

    Mirrors the logic in ``TaskConfigurations.DisplayAllStates.with_grasp_goals``
    in ``script/config/spacelab_config.py``.

    Args:
        goal_states: Iterable of strings matching ``"<gripper> grasps <handle>"``.

    Returns:
        A new class (subclass of *cls*) with ``OBJECTS``, ``GRIPPERS``,
        ``HANDLES_PER_OBJECT``, ``CONTACT_SURFACES_PER_OBJECT``,
        ``VALID_PAIRS``, and ``TOOL_NAME`` filtered to the requested goals.
    """
    pattern = re.compile(r"^(.+)\s+grasps\s+(.+)$")
    needed_pairs: Dict[str, set] = {}
    for goal in goal_states:
        m = pattern.match(goal.strip())
        if m:
            gripper, handle = m.group(1), m.group(2)
            needed_pairs.setdefault(gripper, set()).add(handle)

    if not needed_pairs:
        raise ValueError(
            "No valid grasp goals found. "
            "Expected format: '<gripper> grasps <handle>'"
        )

    # Map each handle to its parent object.
    handle_to_object: Dict[str, str] = {}
    for obj, info in cls.OBJECTS_INFO.items():
        for h in info.get("handles", []):
            handle_to_object[h] = obj

    needed_objects: set = set()
    for handles in needed_pairs.values():
        for h in handles:
            obj = handle_to_object.get(h)
            if obj:
                needed_objects.add(obj)

    if not needed_objects:
        raise ValueError(
            f"Could not find objects for handles: {needed_pairs}"
        )

    # Preserve YAML ordering.
    objects = [o for o in cls.OBJECTS if o in needed_objects]
    grippers = [g for g in cls.GRIPPERS if g in needed_pairs]

    handles_per_object = [
        cls.OBJECTS_INFO[obj]["handles"] for obj in objects
    ]
    contacts_per_object = [
        cls.OBJECTS_INFO[obj]["contact_surfaces"] for obj in objects
    ]
    valid_pairs = {g: list(needed_pairs[g]) for g in grippers}
    tool_name = objects[0] if objects else cls.TOOL_NAME

    return type(
        "YamlTaskConfigFiltered",
        (cls,),
        {
            "OBJECTS": objects,
            "GRIPPERS": grippers,
            "HANDLES_PER_OBJECT": handles_per_object,
            "CONTACT_SURFACES_PER_OBJECT": contacts_per_object,
            "VALID_PAIRS": valid_pairs,
            "TOOL_NAME": tool_name,
        },
    )


# ---------------------------------------------------------------------------
# Main loader class
# ---------------------------------------------------------------------------

class YamlTaskLoader:
    """Load a YAML task config file and expose ManipulationTask-compatible objects.

    The YAML schema is documented in ``script/config/graspball_config.yaml``
    and ``script/config/spacelab_config.yaml``.

    Attributes are lazily built on first access.
    """

    def __init__(self, yaml_path: "str | Path"):
        yaml_path = Path(yaml_path)
        if not yaml_path.exists():
            raise FileNotFoundError(f"YAML config not found: {yaml_path}")
        with open(yaml_path) as fh:
            self._data: Dict[str, Any] = yaml.safe_load(fh)
        self._yaml_path = yaml_path
        self._file_paths: Optional[Dict[str, Any]] = None
        self._joint_bounds_class = None
        self._task_config = None

    # ------------------------------------------------------------------
    # Public properties
    # ------------------------------------------------------------------

    @property
    def file_paths(self) -> Dict[str, Any]:
        """``FILE_PATHS`` dict compatible with ``SceneBuilder``."""
        if self._file_paths is None:
            self._file_paths = self._build_file_paths()
        return self._file_paths

    @property
    def joint_bounds_class(self):
        """Class with ``freeflyer_bounds()`` and ``all_robot_bounds()``."""
        if self._joint_bounds_class is None:
            self._joint_bounds_class = self._build_joint_bounds_class()
        return self._joint_bounds_class

    @property
    def task_config(self):
        """``ManipulationTask``-compatible config class built from YAML data."""
        if self._task_config is None:
            self._task_config = self._build_task_config()
        return self._task_config

    def build_initial_config(
        self,
        objects: Optional[List[str]] = None,
    ) -> List[float]:
        """Build a flat initial configuration list.

        Concatenates robot joint initial values (in joint_groups order) with
        object initial poses (XYZQUAT).

        Args:
            objects: Subset of object names to include. When *None* all objects
                     declared in the YAML are included. Pass
                     ``self.task_config.OBJECTS`` to match whatever subset was
                     selected by ``with_grasp_goals()``.

        Returns:
            Flat ``List[float]`` of joint values + object poses.
        """
        q: List[float] = []

        # Robot joints (all groups, in declaration order).
        for _group, joints in self._data.get("joint_groups", {}).items():
            for jspec in joints:
                q.append(float(jspec["initial"]))

        # Object poses.
        objects_raw: Dict[str, Any] = self._data.get("objects", {})
        selected = objects if objects is not None else list(objects_raw.keys())

        for obj_name in selected:
            if obj_name not in objects_raw:
                # Append identity-at-origin as a safe fallback.
                q.extend([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
                continue
            obj_data = objects_raw[obj_name]
            if "initial_pose_xyzquat" in obj_data:
                q.extend([float(v) for v in obj_data["initial_pose_xyzquat"]])
            elif "initial_pose_xyzrpy" in obj_data:
                rpy = [float(v) for v in obj_data["initial_pose_xyzrpy"]]
                q.extend(xyzrpy_to_xyzquat(rpy).tolist())
            else:
                q.extend([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

        return q

    # ------------------------------------------------------------------
    # Private builders
    # ------------------------------------------------------------------

    def _build_file_paths(self) -> Dict[str, Any]:
        paths = self._data.get("paths", {})

        robot_paths: Dict[str, Dict[str, str]] = {}
        for name, rdata in paths.get("robot", {}).items():
            robot_paths[name] = {
                "urdf": rdata.get("urdf", ""),
                "srdf": rdata.get("srdf", ""),
            }

        env_paths: Dict[str, str] = {}
        for name, urdf in paths.get("environment", {}).items():
            env_paths[name] = urdf

        obj_paths: Dict[str, Dict[str, str]] = {}
        for name, odata in paths.get("objects", {}).items():
            if isinstance(odata, str):
                obj_paths[name] = {"urdf": odata, "srdf": ""}
            else:
                obj_paths[name] = {
                    "urdf": odata.get("urdf", ""),
                    "srdf": odata.get("srdf", ""),
                }

        return {
            "robot": robot_paths,
            "environment": env_paths,
            "objects": obj_paths,
        }

    def _build_joint_bounds_class(self):
        data = self._data
        ff_data = data.get("freeflyer_bounds", {})

        translation: List[List[float]] = [
            list(b) for b in ff_data.get("translation", [])
        ]
        quaternion: List[List[float]] = [
            list(b) for b in ff_data.get("quaternion", [])
        ]

        # Flat freeflyer bounds: [xmin,xmax, ymin,ymax, zmin,zmax, qxmin,qxmax, ...]
        ff_flat: List[float] = []
        for b in translation:
            ff_flat.extend(b)
        for b in quaternion:
            ff_flat.extend(b)

        # Per-joint bounds: {joint_name: [lo, hi]}
        robot_bounds: Dict[str, List[float]] = {}
        for _group, joints in data.get("joint_groups", {}).items():
            for jspec in joints:
                robot_bounds[jspec["joint"]] = list(jspec["bounds"])

        _ff_flat = ff_flat
        _robot_bounds = robot_bounds

        class _JointBounds:
            """Dynamically built joint bounds from YAML."""
            FF_FLAT = _ff_flat
            ROBOT_BOUNDS = _robot_bounds

            @classmethod
            def freeflyer_bounds(cls) -> List[float]:
                return list(cls.FF_FLAT)

            @classmethod
            def all_robot_bounds(cls) -> Dict[str, List[float]]:
                return dict(cls.ROBOT_BOUNDS)

        return _JointBounds

    def _build_task_config(self):
        data = self._data
        objects_raw: Dict[str, Any] = data.get("objects", {})
        object_names: List[str] = list(objects_raw.keys())

        handles_per_object = [
            list(objects_raw[obj].get("handles", []))
            for obj in object_names
        ]
        contacts_per_object = [
            list(objects_raw[obj].get("contact_surfaces", []))
            for obj in object_names
        ]

        # OBJECTS_INFO is a stable dict used by with_grasp_goals() to map
        # handles back to their parent objects regardless of filtering.
        objects_info: Dict[str, Dict[str, List[str]]] = {
            obj: {
                "handles": list(objects_raw[obj].get("handles", [])),
                "contact_surfaces": list(
                    objects_raw[obj].get("contact_surfaces", [])
                ),
            }
            for obj in object_names
        }

        planning = data.get("planning", {})

        # Arm groups (optional) — derive GRIPPER_TO_ARM_KEYWORD and ALL_ARM_KEYWORDS
        # so GraspSequencePlanner auto-freeze works without any hard-coded robot names.
        arm_groups_raw: Dict[str, Any] = data.get("arm_groups", {})
        gripper_to_arm_keyword: Dict[str, str] = {}
        for arm_name, arm_cfg in arm_groups_raw.items():
            keyword = arm_cfg.get("joint_keyword", arm_name)
            for gripper in arm_cfg.get("grippers", []):
                gripper_to_arm_keyword[gripper] = keyword
        all_arm_keywords: List[str] = [
            arm_cfg.get("joint_keyword", arm_name)
            for arm_name, arm_cfg in arm_groups_raw.items()
        ]

        namespace: Dict[str, Any] = {
            # Scene
            "ROBOT_NAMES": list(data.get("robots", [])),
            "ENVIRONMENT_NAMES": list(data.get("environments", [])),
            # Joint groups used by get_joint_groups()
            "ROBOTS": list(data.get("joint_groups", {}).keys()),
            # Objects & handles
            "OBJECTS": list(object_names),
            "HANDLES_PER_OBJECT": handles_per_object,
            "CONTACT_SURFACES_PER_OBJECT": contacts_per_object,
            "OBJECTS_INFO": objects_info,
            # Grippers & pairs
            "GRIPPERS": list(data.get("grippers", [])),
            "VALID_PAIRS": {
                k: list(v)
                for k, v in data.get("valid_pairs", {}).items()
            },
            # Arm groups for auto-freeze
            "ARM_GROUPS": dict(arm_groups_raw),
            "GRIPPER_TO_ARM_KEYWORD": gripper_to_arm_keyword,
            "ALL_ARM_KEYWORDS": all_arm_keywords,
            # Environment
            "ENVIRONMENT_CONTACTS": dict(
                data.get("environment_contacts", {})
            ),
            # Misc task metadata
            "TOOL_NAME": object_names[0] if object_names else "",
            "RULES": None,
            # Planning defaults
            "PATH_VALIDATION_STEP": planning.get("validation_step", 0.01),
            "PATH_PROJECTOR_STEP": planning.get("projector_step", 0.1),
            "MAX_ITERATIONS": planning.get("max_iterations", 1000),
            "MAX_RANDOM_ATTEMPTS": planning.get("max_random_attempts", 1000),
            # Classmethods
            "get_constraint_defs": classmethod(_yaml_get_constraint_defs),
            "init_poses": classmethod(_yaml_init_poses),
            "feasible_grasp_goal_states": classmethod(
                _yaml_feasible_grasp_goal_states
            ),
            "with_grasp_goals": classmethod(_yaml_with_grasp_goals),
        }

        return type("YamlTaskConfig", (), namespace)
