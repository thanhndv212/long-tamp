"""
Configuration management utilities.
"""

import numpy as np
from typing import Dict, List, Optional, Union


class ConfigBuilder:
    """Helper class for building robot configurations."""
    
    def __init__(self):
        """Initialize configuration builder."""
        self.parts = []
        
    def add_joint_config(self, values: Union[List, np.ndarray]):
        """
        Add joint configuration values.
        
        Args:
            values: Joint values
        """
        self.parts.append(np.array(values))
        
    def add_freeflyer_config(self, xyzquat: Union[List, np.ndarray]):
        """
        Add freeflyer configuration [x, y, z, qx, qy, qz, qw].
        
        Args:
            xyzquat: 7D freeflyer configuration
        """
        if len(xyzquat) != 7:
            raise ValueError("Freeflyer config must have 7 values")
        self.parts.append(np.array(xyzquat))
        
    def build(self) -> np.ndarray:
        """
        Build the complete configuration.
        
        Returns:
            Complete configuration vector
        """
        if not self.parts:
            return np.array([])
        return np.concatenate(self.parts)
    
    def reset(self):
        """Reset the builder."""
        self.parts = []


class BoundsManager:
    """Helper class for managing joint bounds."""
    
    @staticmethod
    def freeflyer_bounds(
        translation_bounds: Optional[List] = None,
        quaternion_bounds: Optional[List] = None
    ) -> List:
        """
        Create bounds for a freeflyer joint.
        
        Args:
            translation_bounds: [(x_min, x_max), (y_min, y_max), (z_min, z_max)]
            quaternion_bounds: Optional quaternion bounds (default allows unit quaternions)
            
        Returns:
            List of bounds for freeflyer [x, y, z, qx, qy, qz, qw]
        """
        if translation_bounds is None:
            translation_bounds = [(-2.0, 2.0), (-3.0, 3.0), (-2.0, 2.0)]
            
        if quaternion_bounds is None:
            quaternion_bounds = [(-1.0001, 1.0001)] * 4
            
        bounds = []
        for t_bound in translation_bounds:
            bounds.extend(t_bound)
        for q_bound in quaternion_bounds:
            bounds.extend(q_bound)
            
        return bounds
    
    @staticmethod
    def revolute_bounds(
        min_angle: float = -np.pi,
        max_angle: float = np.pi
    ) -> List:
        """
        Create bounds for a revolute joint.
        
        Args:
            min_angle: Minimum angle
            max_angle: Maximum angle
            
        Returns:
            List [min_angle, max_angle]
        """
        return [min_angle, max_angle]
    
    @staticmethod
    def prismatic_bounds(
        min_pos: float = -1.0,
        max_pos: float = 1.0
    ) -> List:
        """
        Create bounds for a prismatic joint.
        
        Args:
            min_pos: Minimum position
            max_pos: Maximum position
            
        Returns:
            List [min_pos, max_pos]
        """
        return [min_pos, max_pos]


__all__ = [
    "ConfigBuilder",
    "BoundsManager",
]
