"""Configuration utilities for ``agimus_spacelab``.

This module provides:

- Base configuration classes for task definitions
- ``RuleGenerator`` for generating constraint-graph rules
- SpaceLab-specific configuration

Usage::

    from agimus_spacelab.config import BaseTaskConfig, Defaults, RuleGenerator
    from agimus_spacelab.config import ModelPaths, ConstraintDef, StateDef, EdgeDef
"""

from .base_config import (
    BaseTaskConfig,
    ConstraintDef,
    Defaults,
    EdgeDef,
    ModelPaths,
    StateDef,
    TransformConfig,
    merge_configs,
)
from .rules import RuleGenerator

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
]
