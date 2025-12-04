"""
Configuration utilities for agimus_spacelab.

This module provides:
- RuleGenerator: For generating constraint graph rules

Usage:
    from agimus_spacelab.config import RuleGenerator
"""

from .rules import RuleGenerator
from .spacelab_config import (
    RobotJoints,
    InitialConfigurations,
    JointBounds,
    ManipulationConfig
)

__all__ = [
    "RuleGenerator",
    "RobotJoints",
    "InitialConfigurations",
    "JointBounds",
    "ManipulationConfig"
]
