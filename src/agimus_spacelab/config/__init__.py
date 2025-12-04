"""
Configuration utilities for agimus_spacelab.

This module provides:
- RuleGenerator: For generating constraint graph rules
- SpaceLabScenario: Application-specific scenario configuration

Usage:
    from agimus_spacelab.config import RuleGenerator, SpaceLabScenario
"""

from .rules import RuleGenerator
from .spacelab_config import SpaceLabScenario

__all__ = [
    "RuleGenerator",
    "SpaceLabScenario",
]
