"""
Task abstractions for agimus_spacelab manipulation planning.

This module provides high-level task management:
- ManipulationTask: Base class for defining manipulation tasks
- GraspSequencePlanner: Multi-phase grasp sequence planning
- InteractiveGraspSequenceBuilder: Interactive menu-driven sequence builder

Usage:
    from agimus_spacelab.tasks import (
        ManipulationTask,
        GraspSequencePlanner,
        InteractiveGraspSequenceBuilder,
    )
"""

from .base import ManipulationTask
from .grasp_sequence import (
    GraspSequencePlanner,
    InteractiveGraspSequenceBuilder,
)


__all__ = [
    # Base task
    "ManipulationTask",
    # Grasp sequence
    "GraspSequencePlanner",
    "InteractiveGraspSequenceBuilder",
]
