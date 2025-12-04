#!/usr/bin/env python3
"""
Scene setup utilities for Spacelab manipulation tasks.

Provides SceneBuilder for loading robots, environment, and objects.
"""

from typing import Dict, List, Tuple, Optional, Any

# Import from package config
from agimus_spacelab.config.spacelab_config import JointBounds, DEFAULT_PATHS

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
    
    def __init__(self, planner: Optional[Any] = None,
                 backend: str = "corba"):
        """
        Initialize scene builder.
        
        Args:
            planner: Existing planner instance, or None to create new one
            backend: "corba" or "pyhpp" - which backend to use
        """
        self.backend = backend.lower()
        self.loaded_objects = []
        self.DEFAULT_PATHS = DEFAULT_PATHS
        
        if self.backend == "corba":
            if not HAS_CORBA:
                raise ImportError("CORBA backend not available")
            self.planner = planner or CorbaBackend()
        elif self.backend == "pyhpp":
            if not HAS_PYHPP:
                raise ImportError("PyHPP backend not available")
            self.planner = planner or PyHPPBackend()
        else:
            raise ValueError(f"Unknown backend: {backend}. Use 'corba' or 'pyhpp'")
        
    def load_robot(self, composite_name: str = "spacelab",
                   robot_name: str = "spacelab") -> 'SceneBuilder':
        """Load the composite robot (UR10 + VISPA)."""
        print("   Loading robot...")
        if self.backend == "corba":
            self.planner.load_robot(
                composite_name=composite_name,
                robot_name=robot_name,
                urdf_path=self.DEFAULT_PATHS["robot_urdf"],
                srdf_path=self.DEFAULT_PATHS["robot_srdf"],
            )
        else:  # pyhpp
            self.planner.load_robot(
                name=robot_name,
                urdf_path=self.DEFAULT_PATHS["robot_urdf"],
                srdf_path=self.DEFAULT_PATHS["robot_srdf"],
                root_joint_type="anchor"
            )
        return self
        
    def load_environment(self, name: str = "ground_demo") -> 'SceneBuilder':
        """Load the environment (dispenser, ground, etc.)."""
        print("   Loading environment...")
        self.planner.load_environment(
            name=name,
            urdf_path=self.DEFAULT_PATHS["environment"]
        )
        return self
        
    def load_objects(self, object_names: List[str]) -> 'SceneBuilder':
        """
        Load multiple objects.
        
        Args:
            object_names: List of object names to load
        """
        print(f"   Loading {len(object_names)} object(s)...")
        for obj_name in object_names:
            if obj_name not in self.DEFAULT_PATHS["objects"]:
                print(f"      ⚠ Unknown object: {obj_name}")
                continue
            
            self.planner.load_object(
                name=obj_name,
                urdf_path=self.DEFAULT_PATHS["objects"][obj_name],
                root_joint_type="freeflyer"
            )
            self.loaded_objects.append(obj_name)
            
        return self
        
    def set_joint_bounds(self) -> 'SceneBuilder':
        """Set joint bounds for all loaded freeflyer objects."""
        print("   Setting joint bounds...")
        bounds = JointBounds.freeflyer_bounds()
        
        for obj_name in self.loaded_objects:
            joint_name = f"{obj_name}/root_joint"
            self.planner.set_joint_bounds(joint_name, bounds)
            
        return self
        
    def configure_path_validation(self,
                                   validation_step: float = 0.01,
                                   projector_step: float = 0.1) -> 'SceneBuilder':
        """Configure path validation parameters."""
        print("   Configuring path validation...")
        if self.backend == "corba":
            ps = self.planner.get_problem_solver()
            ps.selectPathValidation("Discretized", validation_step)
            ps.selectPathProjector("Progressive", projector_step)
        else:  # pyhpp
            # PyHPP uses dichotomy and progressive projector
            self.planner.set_dichotomy(True)
            self.planner.set_progressive_projector(True)
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
            ps = self.planner.get_problem_solver()
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
        
    def get_instances(self) -> Tuple[Any, Any, Any]:
        """
        Get planner, robot, and problem solver instances.
        
        Returns:
            Tuple of (planner, robot, ps/problem)
        """
        robot = self.planner.get_robot()
        if self.backend == "corba":
            ps = self.planner.get_problem_solver()
        else:  # pyhpp
            ps = self.planner.get_problem()
        return self.planner, robot, ps
        
    def build(self, 
              objects: List[str],
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
        (self.load_robot()
            .load_environment()
            .load_objects(objects)
            .set_joint_bounds()
            .configure_path_validation(validation_step, projector_step))
        
        print("   ✓ Scene setup complete")
        return self.get_instances()


__all__ = [
    "SceneBuilder",
]
