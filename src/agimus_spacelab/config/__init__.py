"""
Configuration utilities for agimus_spacelab.

This module provides:
- Base configuration classes for task definitions
- RuleGenerator: For generating constraint graph rules
- SpaceLab-specific configuration

Usage:
    from agimus_spacelab.config import BaseTaskConfig, Defaults, RuleGenerator
    from agimus_spacelab.config import (
        ModelPaths, ConstraintDef, StateDef, EdgeDef
    )
"""

from .rules import RuleGenerator
from .base_config import (
    Defaults,
    ModelPaths,
    TransformConfig,
    ConstraintDef,
    EdgeDef,
    StateDef,
    BaseTaskConfig,
    merge_configs,
)
from .spacelab_config import (
    DEFAULT_PATHS,
    RobotJoints,
    InitialConfigurations,
    JointBounds,
    ManipulationConfig,
)

__all__ = [
    # Base classes
    "Defaults",
    "ModelPaths",
    "TransformConfig",
    "ConstraintDef",
    "EdgeDef",
    "StateDef",
    "BaseTaskConfig",
    # Utilities
    "merge_configs",
    "RuleGenerator",
    # SpaceLab config (backward compatibility)
    "DEFAULT_PATHS",
    "RobotJoints",
    "InitialConfigurations",
    "JointBounds",
    "ManipulationConfig",
]
