"""
Task abstractions for agimus_spacelab manipulation planning.

This module provides high-level task management:
- ManipulationTask: Base class for defining manipulation tasks
- TaskOrchestrator: Multi-task execution with dependency management
- PlanningBridge: Bridge between task definitions and motion planning
- TaskBuilder: Fluent builder for creating atomic tasks

Usage:
    from agimus_spacelab.tasks import (
        ManipulationTask,
        TaskOrchestrator,
        TaskBuilder,
        PlanningBridge,
    )
"""

from .base import ManipulationTask
from .orchestration import (
    TaskStatus,
    ResourceType,
    Resource,
    TaskPrecondition,
    TaskPostcondition,
    AtomicTask,
    ResourceManager,
    TaskDependencyGraph,
    TaskOrchestrator,
    TaskBuilder,
)
from .bridge import (
    PlanningContext,
    PlanningBridge,
    create_grasp_task,
    create_place_task,
)


__all__ = [
    # Base task
    "ManipulationTask",
    # Orchestration
    "TaskStatus",
    "ResourceType",
    "Resource",
    "TaskPrecondition",
    "TaskPostcondition",
    "AtomicTask",
    "ResourceManager",
    "TaskDependencyGraph",
    "TaskOrchestrator",
    "TaskBuilder",
    # Planning bridge
    "PlanningContext",
    "PlanningBridge",
    "create_grasp_task",
    "create_place_task",
]
