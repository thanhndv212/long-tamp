"""Execution: running planned motion on a robot, a simulator or a viewer (ADR-0004).

ROS-free. A backend implements :class:`ExecutionBackend` (start / poll /
cancel); :func:`run_command` supervises one command with heartbeats, a
duration-scaled deadline and BUSY retries; :class:`ExecutionControl` pauses,
resumes and stops an executor at step boundaries. The Python TaskPlan executor
(roadmap M2) is built on these.
"""

from .contract import (
    ExecutionBackend,
    ExecutionCommand,
    ExecutionPolicy,
    ExecutionResult,
    ExecutionStatus,
    Feedback,
)
from .control import ExecutionControl
from .mock import MockBackend
from .supervisor import run_command

__all__ = [
    "ExecutionBackend",
    "ExecutionCommand",
    "ExecutionControl",
    "ExecutionPolicy",
    "ExecutionResult",
    "ExecutionStatus",
    "Feedback",
    "MockBackend",
    "run_command",
]
