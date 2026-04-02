"""Configuration utilities for ``agimus_spacelab``.

This module provides:

- Base configuration classes for task definitions
- ``RuleGenerator`` for generating constraint-graph rules
- SpaceLab-specific configuration

Usage::

    from agimus_spacelab.config import BaseTaskConfig, Defaults, RuleGenerator
    from agimus_spacelab.config import ModelPaths, ConstraintDef, StateDef, EdgeDef
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
# SpaceLab-specific exports — kept for backward compatibility with existing
# task scripts that do ``from agimus_spacelab.config import ManipulationConfig``.
# New code should import directly from agimus_spacelab.config.spacelab_config
# or use YamlTaskLoader for framework-agnostic configuration.
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
    # SpaceLab config (backward compatibility — prefer direct imports)
    "DEFAULT_PATHS",
    "RobotJoints",
    "InitialConfigurations",
    "JointBounds",
    "ManipulationConfig",
]
