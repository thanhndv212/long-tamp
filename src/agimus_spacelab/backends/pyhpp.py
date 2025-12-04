"""
PyHPP backend implementation for manipulation planning.

This backend uses hpp-python for direct Python bindings to HPP.
"""

from typing import Any, Dict, List, Optional
import numpy as np
from pinocchio import SE3


try:
    from pyhpp.manipulation import (
        Device,
        urdf,
        Graph,
        ManipulationPlanner as HPPManipulationPlanner,
        Problem,
        createProgressiveProjector,
    )
    from pyhpp.core import createDichotomy
    from pyhpp.gepetto.viewer import Viewer
    HAS_PYHPP = True
except ImportError:
    HAS_PYHPP = False


class ConstraintResult:
    """Result from applying state constraints."""
    
    def __init__(self, success: bool, configuration: np.ndarray, error: float):
        self.success = success
        self.configuration = configuration
        self.error = error


class PyHPPBackend:
    """PyHPP backend implementation for manipulation planning."""
    
    def __init__(self):
        """Initialize PyHPP backend."""
        if not HAS_PYHPP:
            raise ImportError(
                "PyHPP backend not available. "
                "Please install hpp-python."
            )
        
        self.device = None
        self.problem = None
        self.graph = None
        self.planner = None
        self.viewer = None
        self.path = None
        
        # Configuration options for path validation and projection
        self._use_dichotomy = True
        self._use_progressive_projector = True

    def model(self):
        """Get the Pinocchio model."""
        if self.device is None:
            raise RuntimeError("Robot not loaded yet")
        return self.device.model()

    def data(self):
        """Get the Pinocchio data."""
        if self.device is None:
            raise RuntimeError("Robot not loaded yet")
        return self.device.data()

    @property
    def nq(self) -> int:
        """Number of configuration variables."""
        if self.device is None:
            raise RuntimeError("Robot not loaded yet")
        return self.device.model().nq

    @property
    def nv(self) -> int:
        """Number of velocity variables."""
        if self.device is None:
            raise RuntimeError("Robot not loaded yet")
        return self.device.model().nv

    def neutral_config(self) -> np.ndarray:
        """Get neutral configuration."""
        if self.device is None:
            raise RuntimeError("Robot not loaded yet")
        return np.array(self.device.neutralConfiguration())

    def random_config(self) -> np.ndarray:
        """Generate a random configuration."""
        if self.device is None:
            raise RuntimeError("Robot not loaded yet")
        return np.array(self.device.randomConfiguration())
        
    def load_robot(
        self,
        name: str,
        urdf_path: str,
        srdf_path: Optional[str] = None,
        root_joint_type: str = "anchor"
    ):
        """Load robot using PyHPP."""
        self.device = Device(name)
        
        urdf.loadModel(
            self.device,
            0,
            name,
            root_joint_type,
            urdf_path,
            srdf_path or "",
            SE3.Identity()
        )
        
        # Create problem
        self.problem = Problem(self.device)
        
        return self.device
    
    def load_environment(
        self,
        name: str,
        urdf_path: str,
        pose: Optional[SE3] = None
    ):
        """Load environment model.
        
        Args:
            name: Name for the environment object
            urdf_path: Path to URDF file
            pose: Optional SE3 pose (defaults to identity)
        """
        if self.device is None:
            raise RuntimeError("Must load robot first")
        
        if pose is None:
            pose = SE3.Identity()
        
        urdf.loadModel(
            self.device,
            0,
            name,
            "anchor",
            urdf_path,
            "",
            pose
        )
        
        return name
    
    def load_object(
        self,
        name: str,
        urdf_path: str,
        root_joint_type: str = "freeflyer"
    ):
        """Load manipulable object."""
        if self.device is None:
            raise RuntimeError("Must load robot first")
        
        urdf.loadModel(
            self.device,
            0,
            name,
            root_joint_type,
            urdf_path,
            "",
            SE3.Identity()
        )
        
        return name
    
    def set_joint_bounds(self, joint_name: str, bounds: List[float]):
        """Set joint bounds."""
        if self.device is None:
            raise RuntimeError("Must load robot first")
        
        self.device.setJointBounds(joint_name, bounds)
    
    def set_initial_config(self, q: np.ndarray):
        """Set initial configuration."""
        if self.problem is None:
            raise RuntimeError("Must create problem first")
        
        self.problem.initConfig(q)
    
    def add_goal_config(self, q: np.ndarray):
        """Add goal configuration."""
        if self.problem is None:
            raise RuntimeError("Must create problem first")
        
        self.problem.addGoalConfig(q)
    
    def create_constraint_graph(
        self,
        name: str,
        grippers: List[str],
        objects: Dict[str, Dict],
        rules: str = "auto",
        **kwargs
    ) -> "Graph":
        """Create constraint graph using PyHPP."""
        if self.problem is None:
            raise RuntimeError("Must create problem first")
        
        # Note: PyHPP requires manual graph construction
        # This is a simplified version
        self.graph = Graph(name, self.device, self.problem)
        
        # Create basic states
        _ = self.graph.createState("free", False, 0)  # free_state created but not used
        
        # For each object, create grasp states
        for obj_name in objects:
            state_name = f"grasp_{obj_name}"
            self.graph.createState(state_name, False, 0)
        
        # Initialize graph
        self.graph.maxIterations(100)
        self.graph.errorThreshold(1e-5)
        self.graph.initialize()
        
        # Set graph in problem
        self.problem.constraintGraph(self.graph)
        
        return self.graph

    def create_state(
        self,
        name: str,
        is_waypoint: bool = False,
        priority: int = 0
    ) -> int:
        """Create a state manually.
        
        Args:
            name: State name
            is_waypoint: Whether this is a waypoint state
            priority: State priority (higher = more important)
            
        Returns:
            State ID
        """
        if self.graph is None:
            raise RuntimeError("Graph not initialized")
        
        state_id = self.graph.createState(name, is_waypoint, priority)
        return state_id

    def create_edge(
        self,
        from_state: str,
        to_state: str,
        name: str,
        weight: int = 1,
        containing_state: Optional[str] = None
    ) -> int:
        """Create an edge manually.
        
        Args:
            from_state: Source state name
            to_state: Target state name
            name: Edge name
            weight: Edge weight for planning
            containing_state: State whose constraints apply during edge
            
        Returns:
            Edge ID
        """
        if self.graph is None:
            raise RuntimeError("Graph not initialized")
        
        edge_id = self.graph.createEdge(
            from_state,
            to_state,
            name,
            weight,
            containing_state or from_state
        )
        return edge_id

    def apply_state_constraints(
        self,
        state: str,
        q: np.ndarray,
        max_iterations: int = 10000,
        error_threshold: float = 1e-4
    ) -> ConstraintResult:
        """Apply state constraints to project configuration.
        
        Args:
            state: State name
            q: Input configuration
            max_iterations: Maximum projection iterations
            error_threshold: Convergence threshold
            
        Returns:
            ConstraintResult with success status, projected config, and error
        """
        if self.graph is None:
            raise RuntimeError("Graph not initialized")
        
        # Store current parameters
        old_max_iter = self.graph.maxIterations()
        old_error = self.graph.errorThreshold()
        
        # Set temporary parameters
        self.graph.maxIterations(max_iterations)
        self.graph.errorThreshold(error_threshold)
        
        # Apply constraints
        success, q_proj, error = self.graph.applyConstraints(state, list(q))
        
        # Restore parameters
        self.graph.maxIterations(old_max_iter)
        self.graph.errorThreshold(old_error)
        
        return ConstraintResult(
            success=success,
            configuration=np.array(q_proj),
            error=error
        )
    
    def solve(self, max_iterations: int = 10000) -> bool:
        """Solve planning problem.
        
        Args:
            max_iterations: Maximum planning iterations
            
        Returns:
            True if solution found
        """
        if self.problem is None:
            raise RuntimeError("Must create problem first")
        
        try:
            # Configure path validation with dichotomy if enabled
            if self._use_dichotomy:
                createDichotomy(self.problem)
            
            # Configure path projection if enabled
            if self._use_progressive_projector:
                createProgressiveProjector(self.problem)
            
            self.planner = HPPManipulationPlanner(self.problem)
            self.planner.maxIterations(max_iterations)
            success = self.planner.solve()
            
            if success:
                self.path = self.planner.path()
            
            return success
        except Exception as e:
            print(f"Planning failed: {e}")
            return False
    
    def get_path(self) -> Optional[Any]:
        """Get computed path."""
        return self.path
    
    def visualize(self, q: Optional[np.ndarray] = None):
        """Visualize configuration."""
        if self.viewer is None:
            if self.device is None:
                raise RuntimeError("Must load robot first")
            self.viewer = Viewer(self.device)
        
        if q is not None:
            self.viewer(q)
        else:
            # Display current config or initial
            try:
                q_init = self.problem.initConfig()
                self.viewer(q_init)
            except Exception:
                pass
    
    def play_path(self, path_index: int = 0):
        """Play path in viewer."""
        if self.path is None:
            print("No path to play")
            return
        
        if self.viewer is None:
            self.visualize()
        
        # Animate path
        import time
        t = 0.0
        dt = 0.01
        length = self.path.length()
        
        while t <= length:
            q = self.path(t)
            self.viewer(q)
            time.sleep(dt)
            t += dt
    
    def get_robot(self):
        """Get device object."""
        return self.device
    
    def get_problem(self):
        """Get problem object."""
        return self.problem
    
    def get_graph(self):
        """Get constraint graph."""
        return self.graph

    def set_dichotomy(self, enabled: bool):
        """Enable or disable dichotomy path validation.
        
        Args:
            enabled: Whether to use dichotomy for path validation
        """
        self._use_dichotomy = enabled

    def set_progressive_projector(self, enabled: bool):
        """Enable or disable progressive path projector.
        
        Args:
            enabled: Whether to use progressive projector
        """
        self._use_progressive_projector = enabled

    def configure_graph_parameters(
        self,
        max_iterations: int = 10000,
        error_threshold: float = 1e-4
    ):
        """Configure constraint graph parameters before initialization.
        
        Args:
            max_iterations: Maximum iterations for constraint projection
            error_threshold: Error threshold for constraint satisfaction
        """
        if self.graph is None:
            raise RuntimeError("Graph not created yet")
        
        self.graph.setMaxIterations(max_iterations)
        self.graph.setErrorThreshold(error_threshold)


# Alias for backward compatibility
PyHPPManipulationPlanner = PyHPPBackend


__all__ = [
    "PyHPPBackend",
    "PyHPPManipulationPlanner",
    "ConstraintResult",
    "HAS_PYHPP",
]
