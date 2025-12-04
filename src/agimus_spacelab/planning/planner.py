"""
Unified API for manipulation planning.

This module provides a backend-agnostic interface for manipulation planning.
"""

from typing import Any, Dict, List, Optional
import numpy as np


def check_backend(backend: str) -> bool:
    """
    Check if a backend is available.
    
    Args:
        backend: Backend name ('corba' or 'pyhpp')
        
    Returns:
        bool: True if backend is available
        
    Raises:
        ValueError: If backend name is invalid
        ImportError: If backend is not available
    """
    if backend not in ["corba", "pyhpp"]:
        raise ValueError(f"Invalid backend: {backend}. Must be 'corba' or 'pyhpp'")
    
    if backend == "corba":
        try:
            import hpp.corbaserver  # noqa: F401
            return True
        except ImportError:
            raise ImportError(
                "CORBA backend not available. Install hpp-manipulation-corba."
            )
    elif backend == "pyhpp":
        try:
            import pyhpp  # noqa: F401
            return True
        except ImportError:
            raise ImportError(
                "PyHPP backend not available. Install hpp-python."
            )
    
    return False


class Planner:
    """
    Unified manipulation planner supporting multiple backends.
    
    This class provides a consistent API regardless of whether
    you're using CORBA server or PyHPP bindings.
    
    Example:
        >>> planner = Planner(backend="pyhpp")
        >>> planner.load_robot(robot_config)
        >>> planner.solve()
    """
    
    def __init__(self, backend: str = "pyhpp"):
        """
        Initialize manipulation planner.
        
        Args:
            backend: Backend to use ('corba' or 'pyhpp')
            
        Raises:
            ValueError: If backend is invalid or not available
        """
        if not check_backend(backend):
            raise ValueError(
                f"Backend '{backend}' is not available. "
                f"Please install the required dependencies."
            )
        
        self.backend = backend
        self._impl = None
        
        # Lazy import to avoid errors if backend not installed
        if backend == "corba":
            from agimus_spacelab.backends import CorbaBackend
            self._impl = CorbaBackend()
        elif backend == "pyhpp":
            from agimus_spacelab.backends import PyHPPBackend
            self._impl = PyHPPBackend()
    
    def load_robot(
        self,
        name: str,
        urdf_path: str,
        srdf_path: Optional[str] = None,
        root_joint_type: str = "anchor"
    ):
        """
        Load robot model.
        
        Args:
            name: Robot name
            urdf_path: Path to URDF file
            srdf_path: Path to SRDF file (optional)
            root_joint_type: Type of root joint
        """
        return self._impl.load_robot(
            name, urdf_path, srdf_path, root_joint_type
        )
    
    def load_environment(self, name: str, urdf_path: str):
        """
        Load environment model.
        
        Args:
            name: Environment name
            urdf_path: Path to URDF file
        """
        return self._impl.load_environment(name, urdf_path)
    
    def load_object(
        self,
        name: str,
        urdf_path: str,
        root_joint_type: str = "freeflyer"
    ):
        """
        Load manipulable object.
        
        Args:
            name: Object name
            urdf_path: Path to URDF file
            root_joint_type: Type of root joint
        """
        return self._impl.load_object(name, urdf_path, root_joint_type)
    
    def set_joint_bounds(self, joint_name: str, bounds: List[float]):
        """
        Set joint bounds.
        
        Args:
            joint_name: Name of the joint
            bounds: Bounds as flat list [min1, max1, min2, max2, ...]
        """
        return self._impl.set_joint_bounds(joint_name, bounds)
    
    def set_initial_config(self, q: np.ndarray):
        """
        Set initial configuration.
        
        Args:
            q: Initial configuration vector
        """
        return self._impl.set_initial_config(q)
    
    def add_goal_config(self, q: np.ndarray):
        """
        Add goal configuration.
        
        Args:
            q: Goal configuration vector
        """
        return self._impl.add_goal_config(q)
    
    def create_constraint_graph(
        self,
        name: str,
        grippers: List[str],
        objects: Dict[str, Dict],
        rules: str = "auto",
        **kwargs
    ) -> Any:
        """
        Create constraint graph for manipulation.
        
        Args:
            name: Graph name
            grippers: List of gripper names
            objects: Dict of object configs with handles/surfaces
            rules: Rule strategy ('auto', 'all', 'sequential', etc.)
            **kwargs: Additional backend-specific parameters
            
        Returns:
            Graph object (backend-specific)
        """
        return self._impl.create_constraint_graph(
            name, grippers, objects, rules, **kwargs
        )
    
    def solve(self, max_iterations: int = 10000) -> bool:
        """
        Solve the planning problem.
        
        Args:
            max_iterations: Maximum iterations
            
        Returns:
            True if solution found
        """
        return self._impl.solve(max_iterations)
    
    def get_path(self) -> Optional[Any]:
        """
        Get the computed path.
        
        Returns:
            Path object (backend-specific) or None
        """
        return self._impl.get_path()
    
    def visualize(self, q: Optional[np.ndarray] = None):
        """
        Visualize configuration or initial config.
        
        Args:
            q: Configuration to display (optional)
        """
        return self._impl.visualize(q)
    
    def play_path(self, path_index: int = 0):
        """
        Play the computed path in viewer.
        
        Args:
            path_index: Index of path to play
        """
        return self._impl.play_path(path_index)
    
    def get_robot(self) -> Any:
        """Get robot object (backend-specific)."""
        return self._impl.get_robot()
    
    def get_problem(self) -> Any:
        """Get problem object (backend-specific)."""
        return self._impl.get_problem()


# Alias for backward compatibility
ManipulationPlanner = Planner


__all__ = [
    "Planner",
    "ManipulationPlanner",
    "check_backend",
]
