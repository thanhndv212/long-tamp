"""
long_tamp - Long-Horizon Task-and-Motion Planning

A flexible, long-horizon multi-arm manipulation planning library using the
native Python (pyhpp) backend for motion planning and task orchestration.

Package Structure:
    - backends: Backend implementations (PyHPP) with unified interface
    - planning: Motion planning tools (create_planner, SceneBuilder, GraphBuilder)
    - tasks: Task orchestration and manipulation task definitions
    - visualization: Constraint graph and handle frame visualization
    - config: Configuration utilities and scenario definitions
    - utils: Transformation and helper utilities

Basic Usage:
    from long_tamp import create_planner, get_available_backends

    # Check available backends
    backends = get_available_backends()

    # Create planner with specified backend
    planner = create_planner(backend="pyhpp")

Advanced Usage:
    from long_tamp.backends import PyHPPBackend
    from long_tamp.planning import SceneBuilder, GraphBuilder
    from long_tamp.tasks import TaskOrchestrator, ManipulationTask
    from long_tamp.visualization import visualize_constraint_graph
"""

__version__ = "0.1.0"
__author__ = "Thanh Nguyen"
__email__ = "dvtnguyen@laas.fr"
__license__ = "MIT"

# Import from new module structure
from .backends import (
    BackendBase,
    ConstraintResult,
    get_available_backends,
    get_backend,
)

# Conditionally import backends that may not be available
try:
    from .backends.pyhpp import PyHPPBackend
except ImportError:
    PyHPPBackend = None  # type: ignore[assignment,misc]

from .logging import (
    RunLogger,
    configure_logging,
    get_logger,
    get_replay_config,
    iter_events,
    load_run_log,
    print_run_summary,
)
from .planning import (
    ConfigGenerator,
    ConstraintBuilder,
    GraphBuilder,
    SceneBuilder,
    create_planner,
)
from .tasks import (
    ManipulationTask,
)
from .visualization import (
    print_joint_info,
    visualize_constraint_graph,
)


def check_backend(backend: str) -> bool:
    """Check if a backend is available.

    Args:
        backend: Backend name to check ('pyhpp').

    Returns:
        True if the backend is installed and importable.

    Raises:
        ValueError: If ``backend`` is not a recognised backend name.
    """
    valid = {"pyhpp"}
    if backend not in valid:
        raise ValueError(
            f"Invalid backend {backend!r}. Must be one of: {sorted(valid)}"
        )
    return backend in get_available_backends()


__all__ = [
    "BackendBase",
    "ConfigGenerator",
    "ConstraintBuilder",
    "ConstraintResult",
    "GraphBuilder",
    # Tasks
    "ManipulationTask",
    "PlanningBridge",
    "PyHPPBackend",
    # Run logging
    "RunLogger",
    "SceneBuilder",
    "TaskBuilder",
    "TaskOrchestrator",
    # Version
    "__version__",
    "check_backend",
    "configure_logging",
    # Planning
    "create_planner",
    # Backend utilities
    "get_available_backends",
    "get_backend",
    "get_logger",
    "get_replay_config",
    "iter_events",
    "load_run_log",
    "print_joint_info",
    "print_run_summary",
    # Visualization
    "visualize_constraint_graph",
]
