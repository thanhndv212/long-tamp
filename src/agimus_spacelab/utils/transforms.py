"""
Transformation and configuration utilities for agimus_spacelab.
"""

import numpy as np
from typing import Dict, List, Optional, Union
from pinocchio import SE3, Quaternion
from pinocchio.rpy import rpyToMatrix


# ============================================================================
# Transformation Functions
# ============================================================================

def xyzrpy_to_se3(xyzrpy):
    """
    Convert [x, y, z, roll, pitch, yaw] to SE3 transformation.
    
    Args:
        xyzrpy: List or array of 6 values [x, y, z, roll, pitch, yaw]
        
    Returns:
        pinocchio.SE3: SE3 transformation
    """
    xyz = np.array(xyzrpy[:3])
    rpy = np.array(xyzrpy[3:])
    rotation_matrix = rpyToMatrix(rpy)
    return SE3(rotation_matrix, xyz)


def se3_to_xyzquat(se3):
    """
    Convert SE3 to [x, y, z, qx, qy, qz, qw] configuration.
    
    Args:
        se3: pinocchio.SE3 transformation
        
    Returns:
        numpy.array: 7 values [x, y, z, qx, qy, qz, qw]
    """
    xyz = se3.translation
    quat = Quaternion(se3.rotation)
    return np.concatenate([xyz, quat.coeffs()])


def xyzrpy_to_xyzquat(xyzrpy):
    """
    Convert [x, y, z, roll, pitch, yaw] to [x, y, z, qx, qy, qz, qw].
    
    Args:
        xyzrpy: List or array of 6 values [x, y, z, roll, pitch, yaw]
        
    Returns:
        numpy.array: 7 values [x, y, z, qx, qy, qz, qw]
    """
    se3 = xyzrpy_to_se3(xyzrpy)
    return se3_to_xyzquat(se3)


def xyzquat_to_xyzrpy(xyzquat):
    """
    Convert [x, y, z, qx, qy, qz, qw] to [x, y, z, roll, pitch, yaw].
    
    Args:
        xyzquat: List or array of 7 values [x, y, z, qx, qy, qz, qw]
        
    Returns:
        numpy.array: 6 values [x, y, z, roll, pitch, yaw]
    """
    se3 = xyzquat_to_se3(xyzquat)
    xyz = se3.translation
    rpy = se3.rotation.eulerAngles(0, 1, 2)  # roll, pitch, yaw
    return np.concatenate([xyz, rpy])


def xyzquat_to_se3(xyzquat):
    """
    Convert [x, y, z, qx, qy, qz, qw] to SE3 transformation.
    
    Args:
        xyzquat: List or array of 7 values [x, y, z, qx, qy, qz, qw]
        
    Returns:
        pinocchio.SE3: SE3 transformation
    """
    xyz = np.array(xyzquat[:3])
    quat = Quaternion(xyzquat[6], xyzquat[3], xyzquat[4], xyzquat[5])
    return SE3(quat.matrix(), xyz)


def normalize_quaternion(quat):
    """
    Normalize a quaternion [qx, qy, qz, qw] or [w, x, y, z].
    
    Args:
        quat: Quaternion as list or array
        
    Returns:
        numpy.array: Normalized quaternion
    """
    quat = np.array(quat)
    norm = np.linalg.norm(quat)
    if norm < 1e-10:
        raise ValueError("Cannot normalize zero quaternion")
    return quat / norm


def merge_configurations(*configs):
    """
    Merge multiple configuration vectors into one.
    
    Args:
        *configs: Variable number of configuration arrays
        
    Returns:
        numpy.array: Merged configuration
    """
    return np.concatenate([np.array(c) for c in configs])


def split_configuration(q, sizes):
    """
    Split a configuration vector into parts.
    
    Args:
        q: Configuration vector
        sizes: List of sizes for each part
        
    Returns:
        list: List of configuration parts
    """
    q = np.array(q)
    parts = []
    start = 0
    for size in sizes:
        parts.append(q[start:start+size])
        start += size
    return parts


def parse_package_uri(uri):
    """
    Parse a package:// URI to extract package path and file name.
    
    Args:
        uri: Package URI string
        
    Returns:
        tuple: (package_path, file_name)
    """
    if not uri.startswith("package://"):
        raise ValueError(f"URI must start with 'package://': {uri}")
    
    # Remove package:// prefix
    path = uri[len("package://"):]
    
    # Split into parts
    parts = path.split("/")
    
    # Find file name
    if "urdf" in parts:
        urdf_index = parts.index("urdf")
        file_with_ext = "/".join(parts[urdf_index+1:])
    elif "robots" in parts:
        robots_index = parts.index("robots")
        file_with_ext = "/".join(parts[robots_index+1:])
    else:
        file_with_ext = parts[-1]
    file_name = file_with_ext.rsplit(".", 1)[0]  # Remove extension
    
    # Find package path
    if "urdf" in parts:
        urdf_index = parts.index("urdf")
        package_path = "/".join(parts[:urdf_index])
    else:
        package_path = "/".join(parts[:-1])
    
    return package_path, file_name


# ============================================================================
# Configuration Builder
# ============================================================================

class ConfigBuilder:
    """Helper class for building robot configurations."""
    
    def __init__(self):
        """Initialize configuration builder."""
        self.parts = []
        
    def add_joint_config(self, values: Union[List, np.ndarray]):
        """Add joint configuration values."""
        self.parts.append(np.array(values))
        
    def add_freeflyer_config(self, xyzquat: Union[List, np.ndarray]):
        """Add freeflyer configuration [x, y, z, qx, qy, qz, qw]."""
        if len(xyzquat) != 7:
            raise ValueError("Freeflyer config must have 7 values")
        self.parts.append(np.array(xyzquat))
        
    def build(self) -> np.ndarray:
        """Build the complete configuration."""
        if not self.parts:
            return np.array([])
        return np.concatenate(self.parts)
    
    def reset(self):
        """Reset the builder."""
        self.parts = []


# ============================================================================
# Bounds Manager
# ============================================================================

class BoundsManager:
    """Helper class for managing joint bounds."""
    
    @staticmethod
    def freeflyer_bounds(
        translation_bounds: Optional[List] = None,
        quaternion_bounds: Optional[List] = None
    ) -> List:
        """Create bounds for a freeflyer joint."""
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
        """Create bounds for a revolute joint."""
        return [min_angle, max_angle]
    
    @staticmethod
    def prismatic_bounds(
        min_pos: float = -1.0,
        max_pos: float = 1.0
    ) -> List:
        """Create bounds for a prismatic joint."""
        return [min_pos, max_pos]


__all__ = [
    # Transform functions
    "xyzrpy_to_se3",
    "se3_to_xyzquat",
    "xyzrpy_to_xyzquat",
    "xyzquat_to_se3",
    "normalize_quaternion",
    "merge_configurations",
    "split_configuration",
    "parse_package_uri",
    # Config utilities
    "ConfigBuilder",
    "BoundsManager",
]
