"""
agimus_spacelab - Manipulation Planning for SpaceLab Tasks

A flexible manipulation planning library supporting both CORBA (hpp-manipulation)
and native Python (pyhpp) backends for motion planning and task orchestration.

Package Structure:
    - backends: Backend implementations (CORBA, PyHPP) with unified interface
    - planning: Motion planning tools (Planner, SceneBuilder, GraphBuilder)
    - tasks: Task orchestration and manipulation task definitions
    - visualization: Constraint graph and handle frame visualization
    - config: Configuration utilities and scenario definitions
    - utils: Transformation and helper utilities

Basic Usage:
    from agimus_spacelab import Planner, get_available_backends
    
    # Check available backends
    backends = get_available_backends()
    
    # Create planner with auto-detected backend
    planner = Planner(backend="auto")

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