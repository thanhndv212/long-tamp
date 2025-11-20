"""
Generic configuration utilities for agimus_spacelab.

This module provides reusable base classes and utilities for configuration.
Task-specific configurations (like Spacelab) should be kept separate.
"""

from .rules import RuleGenerator

__all__ = [
    "RuleGenerator",
]
