#!/usr/bin/env python3
"""
Scene setup utilities for manipulation tasks.

Provides SceneBuilder for loading robots, environment, and objects.
"""

from typing import Any, Dict, List, Optional, Tuple

from long_tamp.logging import get_logger
from long_tamp.planning import create_planner
from long_tamp.utils.transforms import xyzquat_to_se3

logger = get_logger("planning.scene")

# Import unified backend interfaces
try:
    from long_tamp.backends import HAS_PYHPP, PyHPPBackend
except ImportError:
    HAS_PYHPP = False
    PyHPPBackend = None


class SceneBuilder:
    """
    Builder class for setting up scenes with robots and objects.

    Handles loading robots, environment, objects, and configuring collision checking.
    """

    def __init__(
        self,
        joint_bounds=None,
        FILE_PATHS: Optional[Dict[str, Any]] = None,
        planner: Optional[Any] = None,
        backend: str = "pyhpp",
        viewer_type: str = "auto",
    ):
        """
        Initialize scene builder.

        Args:
            planner: Existing planner instance, or None to create new one
            backend: "pyhpp" - which backend to use
            viewer_type: Viewer to use — "viser", "gepetto", or "auto" (default).
        """
        self.backend = backend.lower()
        self.loaded_objects = []

        if FILE_PATHS is None:
            raise ValueError(
                "FILE_PATHS is required. Pass a dict with 'robot', "
                "'environment', and 'objects' keys."
            )
        self.FILE_PATHS = FILE_PATHS

        if joint_bounds is None:
            raise ValueError(
                "joint_bounds is required. Pass a class or object that "
                "provides joint bound information."
            )
        self.joint_bounds = joint_bounds

        if self.backend == "pyhpp":
            if not HAS_PYHPP:
                # Instantiating the backend raises with the specific missing
                # symbol / install guidance instead of a blank message.
                if PyHPPBackend is not None:
                    PyHPPBackend()
                raise ImportError(
                    "PyHPP backend not available and PyHPPBackend could not be "
                    "imported; see long_tamp.backends.pyhpp import errors."
                )
            self.planner = planner or create_planner(
                backend=self.backend, viewer_type=viewer_type
            )
        else:
            raise ValueError(f"Unknown backend: {backend}. Use 'pyhpp'")

    def load_robot(
        self, composite_names: List[str], robot_names: List[str]
    ) -> "SceneBuilder":
        """Load one or more robots into the scene.

        Each robot in `robot_names` is loaded into the same composite
        device (see `BackendBase.load_robot`) — the first call creates it,
        later calls insert into it. An optional `pose` (xyzquat list, e.g.
        `[x, y, z, qx, qy, qz, qw]`) in that robot's `FILE_PATHS["robot"]`
        entry places its root; robots loading without one default to world
        identity, which is only correct for a single robot or one whose
        placement is already baked into its own URDF.
        """
        logger.info("Loading robot (%s)...", robot_names)
        for id, rb_name in enumerate(robot_names):
            if rb_name in self.FILE_PATHS["robot"]:
                robot_paths = self.FILE_PATHS["robot"][rb_name]
                raw_pose = robot_paths.get("pose")
                pose = xyzquat_to_se3(raw_pose) if raw_pose else None
                self.planner.load_robot(
                    robot_name=rb_name,
                    urdf_path=robot_paths["urdf"],
                    srdf_path=robot_paths["srdf"],
                    root_joint_type="anchor",
                    composite_name=composite_names[id],
                    pose=pose,
                )
            else:
                logger.warning("Unknown robot: %s", rb_name)
        return self

    def load_environment(
        self, environment_names: List[str], pose=None
    ) -> "SceneBuilder":
        """Load the environment (dispenser, ground, etc.)."""
        logger.info("Loading environment (%s)...", environment_names)
        for id, env_name in enumerate(environment_names):
            if env_name in self.FILE_PATHS["environment"]:
                logger.debug("Loading environment: %s", env_name)
                logger.debug("  from: %s", self.FILE_PATHS["environment"][env_name])
                self.planner.load_environment(
                    name=env_name,
                    urdf_path=self.FILE_PATHS["environment"][env_name],
                    pose=pose[id] if pose is not None else None,
                )
            else:
                logger.warning("Unknown environment: %s", env_name)
        return self

    def load_objects(self, object_names: List[str]) -> "SceneBuilder":
        """
        Load multiple objects.

        Args:
            object_names: List of object names to load
        """
        logger.info("Loading %d object(s)...", len(object_names))
        for obj_name in object_names:
            if obj_name not in self.FILE_PATHS["objects"]:
                logger.warning("Unknown object: %s", obj_name)
                continue

            obj_config = self.FILE_PATHS["objects"][obj_name]

            # Handle both old format (string) and new format (dict)
            if isinstance(obj_config, str):
                # Old format: just URDF path
                urdf_path = obj_config
                srdf_path = None
            else:
                # New format: dict with urdf and srdf
                urdf_path = obj_config.get("urdf", obj_config)
                srdf_path = obj_config.get("srdf")

            self.planner.load_object(
                name=obj_name,
                urdf_path=urdf_path,
                srdf_path=srdf_path,
                root_joint_type="freeflyer",
            )
            self.loaded_objects.append(obj_name)

        return self

    def set_joint_bounds(self) -> "SceneBuilder":
        """Set joint bounds for robot joints and loaded freeflyer objects.

        Config-defined robot bounds (``joint_bounds.all_robot_bounds()``) are
        the primary source and overwrite whatever limit pinocchio parsed
        from the URDF. Joints the config doesn't mention are left untouched,
        so they fall back to their URDF-parsed limit.
        """
        logger.info("Setting joint bounds...")

        robot_bounds = self.joint_bounds.all_robot_bounds()
        for joint_name, bounds in robot_bounds.items():
            self.planner.set_joint_bounds(joint_name, bounds)

        bounds = self.joint_bounds.freeflyer_bounds()
        for obj_name in self.loaded_objects:
            joint_name = f"{obj_name}/root_joint"
            self.planner.set_joint_bounds(joint_name, bounds)

        return self

    def configure_path_validation(
        self, validation_step: float = 0.01, projector_step: float = 0.1
    ) -> "SceneBuilder":
        """Configure path validation parameters."""
        logger.info("Configuring path validation...")
        self.planner.configure_path_validation(
            validation_step=validation_step, projector_step=projector_step
        )
        return self

    def disable_collision_pair(
        self,
        obstacle_name: str,
        joint_name: str,
        remove_collision: bool = True,
        remove_distance: bool = False,
    ) -> "SceneBuilder":
        """
        Disable collision checking for a specific obstacle-joint pair.

        Args:
            obstacle_name: Name of the obstacle body
            joint_name: Name of the joint
            remove_collision: Remove from collision checking
            remove_distance: Remove from distance checking
        """
        logger.info("Disabling collision: %s <-> %s", obstacle_name, joint_name)
        # Remove pairs from the pinocchio GeometryModel directly.
        # ground_demo and objects are loaded INTO the device geomModel so
        # addAllCollisionPairs() creates robot-vs-environment pairs; we
        # remove them here.
        device = self.planner.device
        m = device.model()
        gm = device.geomModel()
        try:
            from pinocchio import CollisionPair
        except ImportError:
            logger.warning("pinocchio not available; cannot remove collision pairs")
            return self

        # Find obstacle geometry object indices (exact name or _N suffix)
        obs_ids = [
            i
            for i, go in enumerate(gm.geometryObjects)
            if go.name == obstacle_name
            or (
                go.name.startswith(obstacle_name + "_")
                and go.name[len(obstacle_name) + 1 :].isdigit()
            )
        ]
        if not obs_ids:
            logger.warning("No geometry found matching %r", obstacle_name)

        # Resolve joint_name → pinocchio joint index
        joint_id = None
        if m.existJointName(joint_name):
            joint_id = m.getJointId(joint_name)
        elif m.existFrame(joint_name):
            fid = m.getFrameId(joint_name)
            joint_id = int(m.frames[fid].parentJoint)
        if joint_id is None:
            logger.warning("%r not found in device model", joint_name)
            return self

        # Find geometry objects directly attached to this joint
        robot_ids = [
            i for i, go in enumerate(gm.geometryObjects) if go.parentJoint == joint_id
        ]

        # Remove cross-pairs from the geometry model
        removed = 0
        for oid in obs_ids:
            for rid in robot_ids:
                if oid == rid:
                    continue  # CollisionPair requires distinct indices
                cp = CollisionPair(oid, rid)
                if gm.existCollisionPair(cp):
                    gm.removeCollisionPair(cp)
                    removed += 1
        logger.info(
            "removed %d collision pair(s) (%r <-> %r)",
            removed,
            obstacle_name,
            joint_name,
        )
        return self

    def disable_collisions_between_subtrees(
        self,
        robot_frame_or_joint: str,
        obstacle_root_joint: str,
        remove_collision: bool = True,
        remove_distance: bool = False,
        verbose: bool = False,
        max_pairs: int = 80,
    ) -> "SceneBuilder":
        """Disable collisions between a robot subtree and an obstacle subtree.

        This is intended to handle common setups where SRDF grippers/handles are
        defined on fixed/fake links: collisions may happen between collision
        geometries attached to those links, and a simple single-pair exclusion is
        insufficient.

        Args:
            robot_frame_or_joint: A joint name or a frame/link name on the robot.
                If a frame/link is provided, it is converted to its parent joint.
            obstacle_root_joint: Root joint of the obstacle/object (e.g.
                `frame_gripper/root_joint`). Child joints are included.
        """
        logger.info(
            "Disabling collisions (subtrees): %s <-> %s",
            robot_frame_or_joint,
            obstacle_root_joint,
        )

        return self._disable_collisions_pyhpp(
            robot_frame_or_joint, obstacle_root_joint, verbose, max_pairs
        )

    def _disable_collisions_pyhpp(
        self,
        robot_frame_or_joint: str,
        obstacle_root_joint: str,
        verbose: bool,
        max_pairs: int,
    ) -> "SceneBuilder":
        """pyhpp backend: remove cross-pairs between subtrees from the
        pinocchio GeometryModel.  Both `robot_frame_or_joint` and
        `obstacle_root_joint` may be joint names, frame/link names, or
        prefixes (e.g. "ground_demo/joint_world_NYX").

        Environment objects (loaded as "anchor") all attach to joint 0
        (universe) — there are no distinct joints for them in the model.
        For such cases we fall back to geometry name prefix matching:
        the prefix is inferred as everything up to the first "/" in the
        argument (e.g. "ground_demo/joint_world_NYX" → "ground_demo/").
        """
        device = self.planner.device
        m = device.model()
        gm = device.geomModel()
        try:
            from pinocchio import CollisionPair
        except ImportError:
            logger.warning("pinocchio not available; cannot remove collision pairs")
            return self

        def _resolve_joint_id(name):
            """Return pinocchio joint index for a joint name or frame name."""
            if m.existJointName(name):
                return m.getJointId(name)
            if m.existFrame(name):
                fid = m.getFrameId(name)
                return int(m.frames[fid].parentJoint)
            return None

        def _subtree_joint_ids(root_id):
            """Return the set of joint IDs in the subtree rooted at root_id.

            Relies on pinocchio's topological ordering guarantee:
            parents[j] < j for all j > 0.
            """
            subtree = {root_id}
            for j in range(root_id + 1, m.njoints):
                if m.parents[j] in subtree:
                    subtree.add(j)
            return subtree

        def _geom_ids_for(name):
            """Return geometry indices for a joint/frame name or prefix.

            Falls back to geometry name prefix matching when the name is
            not a known joint or frame (happens for anchor-loaded URDF
            models where all links collapse onto universe joint 0), or
            when the resolved joint is the universe joint (id=0) which
            would incorrectly capture ALL geometry objects.
            """
            jid = _resolve_joint_id(name)
            if jid is not None and jid != 0:
                subtree = _subtree_joint_ids(jid)
                return [
                    i
                    for i, go in enumerate(gm.geometryObjects)
                    if go.parentJoint in subtree
                ]
            # Prefix fallback: "ground_demo/joint_world_NYX" → "ground_demo/"
            prefix = name.split("/")[0] + "/"
            return [
                i
                for i, go in enumerate(gm.geometryObjects)
                if go.name.startswith(prefix)
            ]

        robot_geom_ids = _geom_ids_for(robot_frame_or_joint)
        obs_geom_ids = _geom_ids_for(obstacle_root_joint)

        if not robot_geom_ids:
            logger.warning("No geometry found for %r", robot_frame_or_joint)
            return self
        if not obs_geom_ids:
            logger.warning("No geometry found for %r", obstacle_root_joint)
            return self

        if verbose:
            robot_names = [gm.geometryObjects[i].name for i in robot_geom_ids]
            obs_names = [gm.geometryObjects[i].name for i in obs_geom_ids]
            logger.debug(
                "Robot geoms (%d): %s", len(robot_geom_ids), robot_names[:max_pairs]
            )
            logger.debug(
                "Obstacle geoms (%d): %s", len(obs_geom_ids), obs_names[:max_pairs]
            )

        removed = 0
        for rid in robot_geom_ids:
            for oid in obs_geom_ids:
                if rid == oid:
                    continue  # CollisionPair requires distinct indices
                cp = CollisionPair(rid, oid)
                if gm.existCollisionPair(cp):
                    gm.removeCollisionPair(cp)
                    removed += 1
        logger.info(
            "[pyhpp] removed %d collision pair(s) between %r subtree and %r subtree",
            removed,
            robot_frame_or_joint,
            obstacle_root_joint,
        )
        return self

    def move_obstacle(
        self, obstacle_name: str, position: List[float], orientation: List[float]
    ) -> "SceneBuilder":
        """
        Move an object to a specified position and orientation.
        Args:
            object_name: Name of the object to move
            position: [x, y, z] position
            orientation: [qx, qy, qz, qw] quaternion orientation
        """
        # PYHPP-GAP: pyhpp Problem has no moveObstacle binding.
        # Obstacle placement must be set before building the problem
        # via the pinocchio model's placement map.
        logger.warning(
            "[PYHPP-GAP] move_obstacle(%r) is not yet implemented for PyHPP.",
            obstacle_name,
        )
        return self

    def get_instances(self) -> Tuple[Any, Any, Any]:
        """
        Get planner, robot, and problem solver instances.

        Returns:
            Tuple of (planner, robot, ps/problem)
        """
        robot = self.planner.get_robot()
        ps = self.planner.get_problem()
        return self.planner, robot, ps

    def build(
        self,
        robot_names: List[str],
        composite_names: List[str],
        environment_names: List[str],
        object_names: List[str],
        validation_step: float = 0.01,
        projector_step: float = 0.1,
    ) -> Tuple[Any, Any, Any]:
        """
        Complete scene setup with default configuration.

        Args:
            objects: List of object names to load
            validation_step: Path validation discretization step
            projector_step: Path projector step

        Returns:
            Tuple of (planner, robot, ps)
        """
        logger.info("1. Setting up scene...")
        (
            self.load_robot(composite_names=composite_names, robot_names=robot_names)
            .load_environment(environment_names=environment_names)
            .load_objects(object_names=object_names)
            .set_joint_bounds()
            .configure_path_validation(validation_step, projector_step)
        )

        logger.info("✓ Scene setup complete")
        return self.get_instances()


__all__ = [
    "SceneBuilder",
]
