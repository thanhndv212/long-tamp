#!/usr/bin/env python3
"""
Configuration generation and management for manipulation tasks.

Provides ConfigGenerator for generating and validating configurations.
"""

from typing import Dict, List, Tuple, Optional
import numpy as np

# Import from package config
from agimus_spacelab.config.spacelab_config import InitialConfigurations

# Import transformation utilities
try:
    from agimus_spacelab.utils import xyzrpy_to_xyzquat
except ImportError:
    xyzrpy_to_xyzquat = None


class ConfigGenerator:
    """
    Generate and validate configurations for manipulation tasks.
    
    Handles projection, random sampling, and waypoint generation.
    """
    
    def __init__(self, robot, graph, ps, backend: str = "corba",
                 max_attempts: int = 1000):
        """
        Initialize configuration generator.
        
        Args:
            robot: Robot instance
            graph: ConstraintGraph or Graph instance
            ps: ProblemSolver or Problem instance
            backend: "corba" or "pyhpp"
            max_attempts: Maximum random sampling attempts
        """
        self.robot = robot
        self.graph = graph
        self.ps = ps
        self.backend = backend.lower()
        self.max_attempts = max_attempts
        self.configs = {}
        
    def project_on_node(
        self, node_name: str, q: List[float],
        config_label: Optional[str] = None
    ) -> Tuple[bool, List[float]]:
        """
        Project configuration onto node constraints.
        
        Args:
            node_name: Name of the graph node
            q: Configuration to project
            config_label: Optional label to store result
            
        Returns:
            Tuple of (success, projected_config)
        """
        if self.backend == "corba":
            res, q_proj, err = self.graph.applyNodeConstraints(
                node_name, list(q)
            )
        else:  # pyhpp
            res, q_proj, err = self.graph.applyConstraints(node_name, list(q))
        
        if config_label:
            if res:
                self.configs[config_label] = q_proj
                print(f"       ✓ {config_label} projected onto "
                      f"'{node_name}'")
            else:
                self.configs[config_label] = list(q)
                print(f"       ⚠ Projection failed (error: {err:.3f}), "
                      f"using input")
                
        return res, q_proj if res else list(q)
        
    def generate_via_edge(
        self, edge_name: str, q_from: List[float],
        config_label: Optional[str] = None,
        verbose: bool = True
    ) -> Tuple[bool, Optional[List[float]]]:
        """
        Generate target configuration by shooting random configs along edge.
        
        Args:
            edge_name: Name of the edge
            q_from: Source configuration
            config_label: Optional label to store result
            verbose: Print progress every 200 attempts
            
        Returns:
            Tuple of (success, generated_config or None)
        """
        for i in range(self.max_attempts):
            # Generate random config - API is same for both backends
            if self.backend == "corba":
                q_rand = self.robot.shootRandomConfig()
            else:  # pyhpp
                q_rand = self.robot.randomConfiguration()
                if isinstance(q_rand, np.ndarray):
                    q_rand = q_rand.tolist()
            
            res, q_target, err = self.graph.generateTargetConfig(
                edge_name, q_from, q_rand
            )
            if res:
                if config_label:
                    self.configs[config_label] = q_target
                    print(f"       ✓ {config_label} generated via "
                          f"'{edge_name}'")
                return True, q_target
                
            if verbose and (i + 1) % 200 == 0:
                print(f"       Attempt {i + 1}/{self.max_attempts}...")
                
        if config_label:
            print(f"       ⚠ Failed after {self.max_attempts} attempts")
        return False, None
        
    def build_robot_config(
        self, ur10_angles: List[float] = None,
        vispa_base: List[float] = None,
        vispa_arm: List[float] = None
    ) -> List[float]:
        """
        Build robot configuration from joint angles.
        
        Args:
            ur10_angles: UR10 joint angles
            vispa_base: VISPA base config
            vispa_arm: VISPA arm config
            
        Returns:
            Combined robot configuration
        """
        ur10 = ur10_angles if ur10_angles else InitialConfigurations.UR10
        vb = vispa_base if vispa_base else InitialConfigurations.VISPA_BASE
        va = vispa_arm if vispa_arm else InitialConfigurations.VISPA_ARM
        
        return list(ur10) + list(vb) + list(va)
        
    def build_object_configs(self, object_names: List[str]) -> List[float]:
        """
        Build object configurations from initial poses.
        
        Args:
            object_names: List of object names
            
        Returns:
            Concatenated object configurations in XYZQUAT format
        """
        q_objects = []
        
        for obj_name in object_names:
            # Get initial pose in XYZRPY
            obj_attr = obj_name.replace("-", "_").replace(" ", "_").upper()
            if hasattr(InitialConfigurations, obj_attr):
                pose_xyzrpy = getattr(InitialConfigurations, obj_attr)
                pose_xyzquat = xyzrpy_to_xyzquat(pose_xyzrpy)
                q_objects.extend(pose_xyzquat.tolist())
            else:
                print(f"       ⚠ No initial config for {obj_name}")
                # Add zero configuration as fallback
                q_objects.extend([0.0] * 7)
                
        return q_objects
        
    def get_robot_dof(self) -> int:
        """Get total robot DOF (UR10 + VISPA_BASE + VISPA_ARM)."""
        return len(InitialConfigurations.UR10 + 
                   InitialConfigurations.VISPA_BASE + 
                   InitialConfigurations.VISPA_ARM)
        
    def modify_object_pose(
        self, q: List[float], object_index: int,
        translation_delta: Optional[List[float]] = None,
        quaternion: Optional[List[float]] = None
    ) -> List[float]:
        """
        Modify object pose in configuration.
        
        Args:
            q: Full configuration
            object_index: Index of object (0 for first object after robot)
            translation_delta: [dx, dy, dz] to add to position
            quaternion: New quaternion [qx, qy, qz, qw] (replaces existing)
            
        Returns:
            Modified configuration
        """
        q_new = list(q)
        robot_dof = self.get_robot_dof()
        obj_start = robot_dof + object_index * 7
        
        if translation_delta:
            for i in range(3):
                q_new[obj_start + i] += translation_delta[i]
                
        if quaternion:
            for i in range(4):
                q_new[obj_start + 3 + i] = quaternion[i]
                
        return q_new


# Alias for backward compatibility
ConfigurationGenerator = ConfigGenerator


__all__ = [
    "ConfigGenerator",
    "ConfigurationGenerator",
]
