"""
Task abstractions for long_tamp manipulation planning.

This module provides high-level task management:
- ManipulationTask: Base class for defining manipulation tasks
- GraspSequencePlanner: Multi-phase grasp sequence planning
- InteractiveGraspSequenceBuilder: Interactive menu-driven sequence builder
- run_sequence: External orchestrator over GraspSequencePlanner's grasp()/
  release() capability primitives -- an alternative to plan_sequence()'s
  built-in sequencing policy, not a replacement for it

Usage:
    from long_tamp.tasks import (
        ManipulationTask,
        GraspSequencePlanner,
        InteractiveGraspSequenceBuilder,
        run_sequence,
    )
"""

from .base import ManipulationTask
from .grasp_sequence import GraspSequencePlanner
from .interactive_grasp_sequence_builder import InteractiveGraspSequenceBuilder
from .sequence_orchestrator import run_sequence

__all__ = [
    # Grasp sequence
    "GraspSequencePlanner",
    "InteractiveGraspSequenceBuilder",
    "run_sequence",
    # Base task
    "ManipulationTask",
]
