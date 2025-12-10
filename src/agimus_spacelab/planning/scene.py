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
    
    def __init__(self, joint_bounds=None, FILE_PATHS: Optional[Dict[str, Any]] = None, planner: Optional[Any] = None,
                 backend: str = "corba"):
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
            
            self.planner.load_object(
                name=obj_name,
                urdf_path=self.FILE_PATHS["objects"][obj_name],
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
