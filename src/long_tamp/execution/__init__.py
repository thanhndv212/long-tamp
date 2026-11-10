"""Execution: running planned motion on a robot, a simulator or a viewer (ADR-0004).

ROS-free. A backend implements :class:`ExecutionBackend` (start / poll /
cancel), and :class:`GripperActuation` when it drives a hand;
:func:`run_command` supervises one command with heartbeats, a
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
from .executor import PlanExecutor, StepExecution
from .gripper import ExpectedGrasp, GripperActuation, GripperState
from .mock import MockBackend
from .playback import PathPlaybackBackend
from .process import ProcessBackend
from .remote import RemoteBackend, serve_backend
from .sampled import SampledPath
from .supervisor import run_command

__all__ = [
    "ExecutionBackend",
    "ExecutionCommand",
    "ExecutionControl",
    "ExecutionPolicy",
    "ExecutionResult",
    "ExecutionStatus",
    "ExpectedGrasp",
    "Feedback",
    "GripperActuation",
    "GripperState",
    "MockBackend",
    "PathPlaybackBackend",
    "PlanExecutor",
    "ProcessBackend",
    "RemoteBackend",
    "SampledPath",
    "StepExecution",
    "run_command",
    "serve_backend",
]
