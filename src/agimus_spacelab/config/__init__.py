"""
Configuration utilities for agimus_spacelab.

This module provides:
- RuleGenerator: For generating constraint graph rules

Usage:
    from agimus_spacelab.config import RuleGenerator
"""

from .rules import RuleGenerator
from .spacelab_config import (
    DEFAULT_PATHS,
    RobotJoints,
    InitialConfigurations,
    JointBounds,
    ManipulationConfig
)

__all__ = [
    "RuleGenerator",
    "DEFAULT_PATHS",
    "RobotJoints",
    "InitialConfigurations",
    "JointBounds",
    "ManipulationConfig"
]
