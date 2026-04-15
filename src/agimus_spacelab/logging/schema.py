"""
TypedDict schemas for all structured log event types.

These exist for IDE completion and documentation.  They are NOT enforced at
runtime; ``RunLogger.log()`` accepts arbitrary keyword arguments.
"""

from typing import Any, Dict, List, Optional

try:
    from typing import TypedDict, Literal
except ImportError:  # Python < 3.8
    from typing_extensions import TypedDict, Literal  # type: ignore[assignment]


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
    task_config: Dict[str, Any]   # Serialised BaseTaskConfig fields
    setup_params: Dict[str, Any]  # validation_step, skip_graph, …
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
    final_config: Optional[List[float]]
    error: Optional[str]


# ---------------------------------------------------------------------------
# Sequence-level events (GraspSequencePlanner)
# ---------------------------------------------------------------------------


class SequenceStartEvent(TypedDict):
    """Emitted at the top of ``plan_sequence()``."""

    event: Literal["sequence_start"]
    run_id: str
    timestamp: str
    grasp_sequence: List[List[Optional[str]]]  # [[gripper, handle|None], …]
    q_init: List[float]
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
    phase: int               # 1-based
    gripper: str
    handle: Optional[str]
    q_start: List[float]
    held_grasps: Dict[str, str]


class PhaseEndEvent(TypedDict):
    """Emitted after each successfully completed grasp phase."""

    event: Literal["phase_end"]
    run_id: str
    timestamp: str
    phase: int
    gripper: str
    handle: Optional[str]
    success: bool
    phase_time: float
    phase_gen_time: float
    phase_plan_time: float
    final_config: Optional[List[float]]
    state_after: Optional[str]
    saved_files: List[str]
    error: Optional[str]


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
    q_from: List[float]


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
    q_to: Optional[List[float]]
    error: Optional[str]
