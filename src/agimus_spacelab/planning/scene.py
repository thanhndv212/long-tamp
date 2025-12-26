#!/usr/bin/env python3
"""
Scene setup utilities for Spacelab manipulation tasks.

Provides SceneBuilder for loading robots, environment, and objects.
"""

from typing import Dict, List, Tuple, Optional, Any

# Import from package config
from agimus_spacelab.config.spacelab_config import JointBounds, DEFAULT_PATHS
from agimus_spacelab.planning import create_planner
# Import unified backend interfaces
try:
    from agimus_spacelab.backends import CorbaBackend, HAS_CORBA
except ImportError:
    HAS_CORBA = False
    CorbaBackend = None

try:
    from agimus_spacelab.backends import PyHPPBackend, HAS_PYHPP
except ImportError:
    HAS_PYHPP = False
    PyHPPBackend = None


class SceneBuilder:
    """
    Builder class for setting up Spacelab scenes with robots and objects.
    
    Handles loading robots, environment, objects, and configuring collision checking.
    """
    
    def __init__(
        self,
        joint_bounds=None,
        FILE_PATHS: Optional[Dict[str, Any]] = None,
        planner: Optional[Any] = None,
        backend: str = "corba",
    ):
        """
        Initialize scene builder.
        
        Args:
            planner: Existing planner instance, or None to create new one
            backend: "corba" or "pyhpp" - which backend to use
        """
        self.backend = backend.lower()
        self.loaded_objects = []
        
        if FILE_PATHS is None:
            self.FILE_PATHS = DEFAULT_PATHS
        else:
            self.FILE_PATHS = FILE_PATHS

        if joint_bounds is None:
            self.joint_bounds = JointBounds
        else:
            self.joint_bounds = joint_bounds

        if self.backend == "corba":
            if not HAS_CORBA:
                raise ImportError("CORBA backend not available")
            self.planner = planner or create_planner(backend=self.backend)
        elif self.backend == "pyhpp":
            if not HAS_PYHPP:
                raise ImportError("PyHPP backend not available")
            self.planner = planner or create_planner(backend=self.backend)
        else:
            raise ValueError(f"Unknown backend: {backend}. Use 'corba' or 'pyhpp'")
        
    def load_robot(self, composite_names: List[str],
                   robot_names: List[str]) -> 'SceneBuilder':
        """Load the composite robot (UR10 + VISPA)."""
        print(f"   Loading robot ({robot_names})...")
        for id, rb_name in enumerate(robot_names):
            if rb_name in self.FILE_PATHS["robot"]:
                self.planner.load_robot(
                    robot_name=rb_name,
                    urdf_path=self.FILE_PATHS["robot"][rb_name]["urdf"],
                    srdf_path=self.FILE_PATHS["robot"][rb_name]["srdf"],
                    root_joint_type="anchor",
                    composite_name=composite_names[id]
            )
            else:
                print(f"      ⚠ Unknown robot: {rb_name}")
        return self
        
    def load_environment(self, environment_names: List[str], pose=None) -> 'SceneBuilder':
        """Load the environment (dispenser, ground, etc.)."""
        print(f"   Loading environment ({environment_names})...")
        for id, env_name in enumerate(environment_names):
            if env_name in self.FILE_PATHS["environment"]:
                print(f"      Loading environment: {env_name}")
                print(f"         from: {self.FILE_PATHS['environment'][env_name]}")
                self.planner.load_environment(
                    name=env_name,
                    urdf_path=self.FILE_PATHS["environment"][env_name],
                    pose=pose[id] if pose is not None else None
                )
            else:
                print(f"      ⚠ Unknown environment: {env_name}")
        return self
        
    def load_objects(self, object_names: List[str]) -> 'SceneBuilder':
        """
        Load multiple objects.
        
        Args:
            object_names: List of object names to load
        """
        print(f"   Loading {len(object_names)} object(s)...")
        for obj_name in object_names:
            if obj_name not in self.FILE_PATHS["objects"]:
                print(f"      ⚠ Unknown object: {obj_name}")
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
                root_joint_type="freeflyer"
            )
            self.loaded_objects.append(obj_name)
            
        return self
        
    def set_joint_bounds(self) -> 'SceneBuilder':
        """Set joint bounds for all loaded freeflyer objects."""
        print("   Setting joint bounds...")
        bounds = self.joint_bounds.freeflyer_bounds()
        
        for obj_name in self.loaded_objects:
            joint_name = f"{obj_name}/root_joint"
            self.planner.set_joint_bounds(joint_name, bounds)
            
        return self
        
    def configure_path_validation(self,
                                   validation_step: float = 0.01,
                                   projector_step: float = 0.1) -> 'SceneBuilder':
        """Configure path validation parameters."""
        print("   Configuring path validation...")
        self.planner.configure_path_validation(
            validation_step=validation_step,
            projector_step=projector_step
        )
        return self
        
    def disable_collision_pair(self,
                               obstacle_name: str,
                               joint_name: str,
                               remove_collision: bool = True,
                               remove_distance: bool = False) -> 'SceneBuilder':
        """
        Disable collision checking for a specific obstacle-joint pair.
        
        Args:
            obstacle_name: Name of the obstacle body
            joint_name: Name of the joint
            remove_collision: Remove from collision checking
            remove_distance: Remove from distance checking
        """
        print(f"   Disabling collision: {obstacle_name} <-> {joint_name}")
        if self.backend == "corba":
            ps = self.planner.get_problem()
            ps.removeObstacleFromJoint(
                obstacle_name,
                joint_name,
                remove_collision,
                remove_distance
            )
        else:
            # PyHPP collision management is handled differently
            # Would need to use device collision pairs API
            print("      (PyHPP: collision management not yet implemented)")
        return self

    def disable_collisions_between_subtrees(
        self,
        robot_frame_or_joint: str,
        obstacle_root_joint: str,
        remove_collision: bool = True,
        remove_distance: bool = False,
        verbose: bool = False,
        max_pairs: int = 80,
    ) -> 'SceneBuilder':
        """Disable collisions between a robot subtree and an obstacle subtree.

        This is intended to handle common setups where SRDF grippers/handles are
        defined on fixed/fake links: collisions may happen between collision
        geometries attached to those links, and a simple single-pair exclusion is
        insufficient.

        For CORBA, this expands to repeated calls to
        `ProblemSolver.removeObstacleFromJoint(obstacle, joint, ...)`.

        Args:
            robot_frame_or_joint: A joint name or a frame/link name on the robot.
                If a frame/link is provided, it is converted to its parent joint.
            obstacle_root_joint: Root joint of the obstacle/object (e.g.
                `frame_gripper/root_joint`). Child joints are included.
        """
        print(
            "   Disabling collisions (subtrees): %s <-> %s"
            % (robot_frame_or_joint, obstacle_root_joint)
        )

        if self.backend != "corba":
            print("      (PyHPP: collision management not yet implemented)")
            return self

        robot = self.planner.get_robot()
        ps = self.planner.get_problem()

        # Resolve robot joint
        robot_joint = robot_frame_or_joint
        try:
            all_joints = set(robot.getAllJointNames())
        except Exception:
            all_joints = set()

        if robot_joint not in all_joints:
            try:
                robot_joint = robot.getParentFrame(robot_frame_or_joint)
            except Exception:
                # Keep original; better to attempt than to silently ignore.
                robot_joint = robot_frame_or_joint

        # Robot subtree joints (including fixed/fake-link joints)
        robot_joints = [robot_joint]
        try:
            robot_joints += list(robot.getChildJoints(robot_joint, True))
        except Exception:
            pass

        # Obstacle subtree joints
        obstacle_joints = [obstacle_root_joint]
        try:
            obstacle_joints += list(robot.getChildJoints(obstacle_root_joint, True))
        except Exception:
            pass

        # Prefer filtering based on current outer-objects lists to avoid relying
        # on exact naming conventions of inner collision objects (which may be
        # indexed like `..._0`).
        obstacle_prefix = obstacle_root_joint.split("/", 1)[0] + "/"

        def _pairs_between_by_prefix() -> List[tuple[str, str]]:
            pairs: List[tuple[str, str]] = []
            for rj in robot_joints:
                try:
                    outer = robot.getJointOuterObjects(rj)
                except Exception:
                    outer = []
                for obj in outer:
                    if isinstance(obj, str) and obj.startswith(obstacle_prefix):
                        pairs.append((rj, obj))
            return pairs

        before_pairs: List[tuple[str, str]] = _pairs_between_by_prefix()
        if verbose:
            print(
                "      Target obstacle prefix: %s"
                % obstacle_prefix
            )
            print(
                "      Found %d existing collision pairs to filter"
                % len(before_pairs)
            )
            for rj, obj in before_pairs[: max_pairs]:
                print(f"        - {rj} <-> {obj}")
            if len(before_pairs) > max_pairs:
                print(f"        ... ({len(before_pairs) - max_pairs} more)")

        removed = 0
        for rj, obj in before_pairs:
            try:
                ps.removeObstacleFromJoint(obj, rj, remove_collision, remove_distance)
                removed += 1
            except Exception:
                continue

        if verbose:
            print(f"      removeObstacleFromJoint calls attempted: {removed}")
            after_pairs = _pairs_between_by_prefix()
            print(
                "      Remaining collision pairs after filtering: %d"
                % len(after_pairs)
            )
            for rj, obj in after_pairs[: max_pairs]:
                print(f"        - {rj} <-> {obj}")
            if len(after_pairs) > max_pairs:
                print(f"        ... ({len(after_pairs) - max_pairs} more)")

        return self

    def move_obstacle(self,
                      obstacle_name: str,
                      position: List[float],
                      orientation: List[float]) -> 'SceneBuilder':
        
        """
        Move an object to a specified position and orientation.
        Args:
            object_name: Name of the object to move
            position: [x, y, z] position
            orientation: [qx, qy, qz, qw] quaternion orientation
        """
        if self.backend == "corba":
            ps = self.planner.get_problem()
            ps.moveObstacle(
                obstacle_name,
                position + orientation
            )
        else:
            print("      (PyHPP: object movement not yet implemented)")
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
        
    def build(self,
              robot_names: List[str],
              composite_names: List[str],
              environment_names: List[str],
              object_names: List[str],
              validation_step: float = 0.01,
              projector_step: float = 0.1) -> Tuple[Any, Any, Any]:
        """
        Complete scene setup with default configuration.
        
        Args:
            objects: List of object names to load
            validation_step: Path validation discretization step
            projector_step: Path projector step
            
        Returns:
            Tuple of (planner, robot, ps)
        """
        print("\n1. Setting up scene...")
        (self.load_robot(composite_names=composite_names,
                         robot_names=robot_names)
            .load_environment(environment_names=environment_names)
            .load_objects(object_names=object_names)
            .set_joint_bounds()
            .configure_path_validation(validation_step, projector_step))
        
        print("   ✓ Scene setup complete")
        return self.get_instances()


__all__ = [
    "SceneBuilder",
]
