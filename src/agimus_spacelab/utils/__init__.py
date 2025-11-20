"""
Utility functions for agimus_spacelab.

This module provides transformation utilities and helper functions.
"""

import numpy as np
from pinocchio import SE3, Quaternion
from pinocchio.rpy import rpyToMatrix


def xyzrpy_to_se3(xyzrpy):
    """
    Convert [x, y, z, roll, pitch, yaw] to SE3 transformation.
    
    Args:
        xyzrpy: List or array of 6 values [x, y, z, roll, pitch, yaw]
        
    Returns:
        pinocchio.SE3: SE3 transformation
        
    Example:
        >>> se3 = xyzrpy_to_se3([1.0, 2.0, 3.0, 0.0, 0.0, 1.57])
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
        
    Example:
        >>> xyzquat = se3_to_xyzquat(SE3.Identity())
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
        
    Example:
        >>> xyzquat = xyzrpy_to_xyzquat([1.0, 2.0, 3.0, 0.0, 0.0, 1.57])
    """
    se3 = xyzrpy_to_se3(xyzrpy)
    return se3_to_xyzquat(se3)


def xyzquat_to_se3(xyzquat):
    """
    Convert [x, y, z, qx, qy, qz, qw] to SE3 transformation.
    
    Args:
        xyzquat: List or array of 7 values [x, y, z, qx, qy, qz, qw]
        
    Returns:
        pinocchio.SE3: SE3 transformation
        
    Example:
        >>> se3 = xyzquat_to_se3([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0])
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
        
    Example:
        >>> q_norm = normalize_quaternion([0.5, 0.5, 0.5, 0.5])
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
        
    Example:
        >>> q_full = merge_configurations(q_robot, q_object1, q_object2)
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
        
    Example:
        >>> q_robot, q_obj1, q_obj2 = split_configuration(q_full, [6, 7, 7])
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
        uri: Package URI string (e.g., "package://spacelab_mock_hardware/description/urdf/RS.urdf")
        
    Returns:
        tuple: (package_path, file_name) where:
            - package_path is the package name with subdirectories (e.g., "spacelab_mock_hardware/description")
            - file_name is the base name without extension (e.g., "RS")
            
    Example:
        >>> parse_package_uri("package://spacelab_mock_hardware/description/urdf/RS.urdf")
        ('spacelab_mock_hardware/description', 'RS')
        
        >>> parse_package_uri("package://example_robot_data/robots/ur5_gripper.urdf")
        ('example_robot_data/robots', 'ur5_gripper')
    """
    if not uri.startswith("package://"):
        raise ValueError(f"URI must start with 'package://': {uri}")
    
    # Remove package:// prefix
    path = uri[len("package://"):]
    
    # Split into parts
    parts = path.split("/")
    
    # Everything after "urdf" or "robots" is considered the file name not including "urdf" or "robots"
    if "urdf" in parts:
        urdf_index = parts.index("urdf")
        file_with_ext = "/".join(parts[urdf_index+1:])
    elif "robots" in parts:
        robots_index = parts.index("robots")
        file_with_ext = "/".join(parts[robots_index+1:])
    else:
        file_with_ext = parts[-1]
    file_name = file_with_ext.rsplit(".", 1)[0]  # Remove extension
    
    # Package path is everything except "urdf" or "robots" and the last part (file name)
    if "urdf" in parts:
        urdf_index = parts.index("urdf")
        package_path = "/".join(parts[:urdf_index])
    else:
        package_path = "/".join(parts[:-1])
    
    return package_path, file_name


__all__ = [
    "xyzrpy_to_se3",
    "se3_to_xyzquat",
    "xyzrpy_to_xyzquat",
    "xyzquat_to_se3",
    "normalize_quaternion",
    "merge_configurations",
    "split_configuration",
    "parse_package_uri",
]
