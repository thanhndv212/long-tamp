"""
Utility functions for agimus_spacelab.

This module provides transformation utilities and helper functions.
"""

import numpy as np
from pinocchio import SE3, Quaternion
from pinocchio.rpy import rpyToMatrix

from .transforms import (
    xyzrpy_to_se3,
    se3_to_xyzquat,
    xyzrpy_to_xyzquat,
    xyzquat_to_se3,
    normalize_quaternion,
    merge_configurations,
    split_configuration,
    parse_package_uri,
    ConfigBuilder,
    BoundsManager,
)


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
