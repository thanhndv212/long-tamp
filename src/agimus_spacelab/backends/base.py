"""
Core base classes for agimus_spacelab manipulation planning.

This module defines abstract base classes and interfaces for backend
implementations.
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional
import numpy as np


class ManipulationTaskBase(ABC):
    """Abstract base class for manipulation tasks."""
    
    @abstractmethod
    def load_robot(self, config: Dict[str, Any]):
        """Load robot model."""
        pass
    
    @abstractmethod
    def load_environment(self, config: Dict[str, Any]):
        """Load environment model."""
        pass
    
    @abstractmethod
    def load_objects(self, objects: List[Dict[str, Any]]):
        """Load manipulable objects."""
        pass
    
    @abstractmethod
    def set_initial_config(self, q: np.ndarray):
        """Set initial configuration."""
        pass
    
    @abstractmethod
    def add_goal_config(self, q: np.ndarray):
        """Add goal configuration."""
        pass
    
    @abstractmethod
    def solve(self, max_iterations: int = 10000) -> bool:
        """
        Solve the planning problem.
        
        Returns:
            True if solution found
        """
        pass
    
    @abstractmethod
    def get_path(self) -> Optional[Any]:
        """Get computed path."""
        pass


class RobotBase(ABC):
    """Abstract base class for robot representation."""
    
    def __init__(self, name: str):
        """
        Initialize robot.
        
        Args:
            name: Robot name
        """
        self.name = name
        self._device = None
        
    @abstractmethod
    def load_urdf(
        self,
        urdf_path: str,
        srdf_path: Optional[str] = None,
        root_joint_type: str = "anchor"
    ):
        """Load robot from URDF."""
        pass
    
    @abstractmethod
    def get_config_size(self) -> int:
        """Get configuration vector size."""
        pass
    
    @abstractmethod
    def get_neutral_config(self) -> np.ndarray:
        """Get neutral configuration."""
        pass
    
    @abstractmethod
    def set_joint_bounds(self, joint_name: str, bounds: List[float]):
        """Set joint bounds."""
        pass


class PlannerBase(ABC):
    """Abstract base class for motion planners."""
    
    @abstractmethod
    def set_problem(self, problem: Any):
        """Set planning problem."""
        pass
    
    @abstractmethod
    def solve(self, max_iterations: int = 10000) -> bool:
        """Solve planning problem."""
        pass
    
    @abstractmethod
    def get_path(self) -> Optional[Any]:
        """Get computed path."""
        pass


class GraphBase(ABC):
    """Abstract base class for constraint graphs."""
    
    @abstractmethod
    def create_state(self, name: str) -> Any:
        """Create a state in the graph."""
        pass
    
    @abstractmethod
    def create_transition(
        self,
        from_state: Any,
        to_state: Any,
        name: str
    ) -> Any:
        """Create a transition between states."""
        pass
    
    @abstractmethod
    def add_constraint(self, state: Any, constraint: Any):
        """Add constraint to a state."""
        pass
    
    @abstractmethod
    def initialize(self):
        """Initialize the graph."""
        pass


class ViewerBase(ABC):
    """Abstract base class for visualization."""
    
    @abstractmethod
    def display(self, q: np.ndarray):
        """Display configuration."""
        pass
    
    @abstractmethod
    def play_path(self, path: Any):
        """Play a path."""
        pass


__all__ = [
    "ManipulationTaskBase",
    "RobotBase",
    "PlannerBase",
    "GraphBase",
    "ViewerBase",
]
