"""
TypedDict schemas for all structured log event types.

These exist for IDE completion and documentation.  They are NOT enforced at
runtime; ``RunLogger.log()`` accepts arbitrary keyword arguments.
"""

from typing import Any

try:
    from typing import Literal, TypedDict
except ImportError:  # Python < 3.8
    from typing import Literal  # type: ignore[assignment]

    from typing_extensions import TypedDict


# ---------------------------------------------------------------------------
# Task-level events
# ---------------------------------------------------------------------------


class RunStartEvent(TypedDict):
    """Emitted once in ``ManipulationTask.__init__`` when ``log_dir`` is set."""

    event: Literal["run_start"]
    run_id: str
    timestamp: str
    task_name: str
    backend: str
    hostname: str
    python_version: str


class ConfigSnapshotEvent(TypedDict):
    """Emitted at the start of ``ManipulationTask.setup()``."""

    event: Literal["config_snapshot"]
    run_id: str
    timestamp: str
    task_config: dict[str, Any]  # Serialised BaseTaskConfig fields
    setup_params: dict[str, Any]  # validation_step, skip_graph, …
    backend: str
    task_name: str


class RunEndEvent(TypedDict):
    """Emitted when ``ManipulationTask.run()`` or ``plan_sequence()`` exits."""

    event: Literal["run_end"]
    run_id: str
    timestamp: str
    success: bool
    total_time: float
    total_planning_time: float
    phase_count: int
    final_config: list[float] | None
    error: str | None


# ---------------------------------------------------------------------------
# Sequence-level events (GraspSequencePlanner)
# ---------------------------------------------------------------------------


class SequenceStartEvent(TypedDict):
    """Emitted at the top of ``plan_sequence()``."""

    event: Literal["sequence_start"]
    run_id: str
    timestamp: str
    grasp_sequence: list[list[str | None]]  # [[gripper, handle|None], …]
    q_init: list[float]
    validate: bool
    max_iterations_per_edge: int
    timeout_per_edge: float
    frozen_arms_mode: str
    time_parameterize: bool
    reset_roadmap: bool


class PhaseStartEvent(TypedDict):
    """Emitted at the start of each grasp phase (before graph build)."""

    event: Literal["phase_start"]
    run_id: str
    timestamp: str
    phase: int  # 1-based
    gripper: str
    handle: str | None
    q_start: list[float]
    held_grasps: dict[str, str]


class PhaseEndEvent(TypedDict):
    """Emitted after each successfully completed grasp phase."""

    event: Literal["phase_end"]
    run_id: str
    timestamp: str
    phase: int
    gripper: str
    handle: str | None
    success: bool
    phase_time: float
    phase_gen_time: float
    phase_plan_time: float
    final_config: list[float] | None
    state_after: str | None
    saved_files: list[str]
    error: str | None


# ---------------------------------------------------------------------------
# Edge-level events
# ---------------------------------------------------------------------------


class EdgeStartEvent(TypedDict):
    """Emitted before target-config generation for each transition edge."""

    event: Literal["edge_start"]
    run_id: str
    timestamp: str
    phase: int
    edge_idx: int
    edge_name: str
    q_from: list[float]


class EdgeEndEvent(TypedDict):
    """Emitted after each edge attempt (success or failure)."""

    event: Literal["edge_end"]
    run_id: str
    timestamp: str
    phase: int
    edge_idx: int
    edge_name: str
    success: bool
    gen_time: float
    plan_time: float
    total_time: float
    q_to: list[float] | None
    error: str | None
