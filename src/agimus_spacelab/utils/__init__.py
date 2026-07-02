"""
Utility functions for agimus_spacelab.

This module provides transformation utilities, helper functions,
and interactive terminal utilities.
"""

import numpy as np
from pinocchio import SE3, Quaternion
from pinocchio.rpy import rpyToMatrix

from .interactive import (
    clear_line,
    hide_cursor,
    interactive_menu,
    move_cursor_up,
    show_cursor,
)
from .transforms import (
    BoundsManager,
    ConfigBuilder,
    merge_configurations,
    normalize_quaternion,
    parse_package_uri,
    se3_to_xyzquat,
    split_configuration,
    xyzquat_to_se3,
    xyzquat_to_xyzrpy,
    xyzrpy_to_se3,
    xyzrpy_to_xyzquat,
)

__all__ = [
    # Transform functions
    "xyzrpy_to_se3",
    "se3_to_xyzquat",
    "xyzrpy_to_xyzquat",
    "xyzquat_to_xyzrpy",
    "xyzquat_to_se3",
    "normalize_quaternion",
    "merge_configurations",
    "split_configuration",
    "parse_package_uri",
    # Config utilities
    "ConfigBuilder",
    "BoundsManager",
    # Interactive utilities
    "interactive_menu",
    "clear_line",
    "move_cursor_up",
    "hide_cursor",
    "show_cursor",
]
