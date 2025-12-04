"""
agimus_spacelab - Manipulation Planning for SpaceLab Tasks

A flexible manipulation planning library supporting both CORBA (hpp-manipulation)
and native Python (pyhpp) backends for motion planning and task orchestration.

Package Structure:
    - backends: Backend implementations (CORBA, PyHPP) with unified interface
    - planning: Motion planning tools (create_planner, SceneBuilder, GraphBuilder)
    - tasks: Task orchestration and manipulation task definitions
    - visualization: Constraint graph and handle frame visualization
    - config: Configuration utilities and scenario definitions
    - utils: Transformation and helper utilities

Basic Usage:
    from agimus_spacelab import create_planner, get_available_backends
    
    # Check available backends
    backends = get_available_backends()
    
    # Create planner with specified backend
    planner = create_planner(backend="corba")

Advanced Usage:
    from agimus_spacelab.backends import CorbaBackend, PyHPPBackend
    from agimus_spacelab.planning import SceneBuilder, GraphBuilder
    from agimus_spacelab.tasks import TaskOrchestrator, ManipulationTask
    from agimus_spacelab.visualization import visualize_constraint_graph
"""

__version__ = "0.1.0"
__author__ = "Thanh Nguyen"
__email__ = "dvtnguyen@laas.fr"
__license__ = "LGPL-3.0"

# Import from new module structure
from .backends import (
    get_available_backends,
    get_backend,
    CorbaBackend,
    PyHPPBackend,
    ManipulationTaskBase,
)

from .planning import (
    create_planner,
    SceneBuilder,
    GraphBuilder,
    ConstraintBuilder,
    ConfigGenerator,
)

from .tasks import (
    ManipulationTask,
    TaskOrchestrator,
    TaskBuilder,
    PlanningBridge,
)

from .visualization import (
    visualize_constraint_graph,
    print_joint_info,
)


def check_backend(backend: str) -> bool:
    """Check if a backend is available."""
    return backend in get_available_backends()


__all__ = [
    # Version
    "__version__",
    # Backend utilities
    "get_available_backends",
    "get_backend",
    "check_backend",
    "CorbaBackend",
    "PyHPPBackend",
    "ManipulationTaskBase",
    # Planning
    "create_planner",
    "SceneBuilder",
    "GraphBuilder",
    "ConstraintBuilder",
    "ConfigGenerator",
    # Tasks
    "ManipulationTask",
    "TaskOrchestrator",
    "TaskBuilder",
    "PlanningBridge",
    # Visualization
    "visualize_constraint_graph",
    "print_joint_info",
]
__license__ = "LGPL-3.0"